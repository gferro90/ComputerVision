/****************************************************************************
 *                                                                           *
 *  OpenNI 1.x Alpha                                                         *
 *  Copyright (C) 2011 PrimeSense Ltd.                                       *
 *                                                                           *
 *  This file is part of OpenNI.                                             *
 *                                                                           *
 *  OpenNI is free software: you can redistribute it and/or modify           *
 *  it under the terms of the GNU Lesser General Public License as published *
 *  by the Free Software Foundation, either version 3 of the License, or     *
 *  (at your option) any later version.                                      *
 *                                                                           *
 *  OpenNI is distributed in the hope that it will be useful,                *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
 *  GNU Lesser General Public License for more details.                      *
 *                                                                           *
 *  You should have received a copy of the GNU Lesser General Public License *
 *  along with OpenNI. If not, see <http://www.gnu.org/licenses/>.           *
 *                                                                           *
 ****************************************************************************/
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <XnOS.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#if (XN_PLATFORM == XN_PLATFORM_MACOSX)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <math.h>

#include <XnCppWrapper.h>
#include <cv.h>
#include <highgui.h>
#include <cxcore.h>
#include <opencv2/core/core.hpp>
//#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

using namespace xn;
using namespace cv;
using namespace std;
//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define SAMPLE_XML_PATH "/home/pc/Kinect/kinect/SensorKinect/OpenNI/Data/SamplesConfig.xml"
//"/home/pc/Kinect/kinect/OpenNI-OpenNI-e263e59/Data/SamplesConfig.xml"

#define GL_WIN_SIZE_X 1280
#define GL_WIN_SIZE_Y 1024

#define DISPLAY_MODE_OVERLAY	1
#define DISPLAY_MODE_DEPTH		2
#define DISPLAY_MODE_IMAGE		3
#define DEFAULT_DISPLAY_MODE	DISPLAY_MODE_DEPTH

#define MAX_RADIUS 400
#define MIN_RADIUS 10
//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------

float* g_pDepthHist;
XnRGB24Pixel* g_pTexMap = NULL;
unsigned int g_nTexMapX = 0;
unsigned int g_nTexMapY = 0;
XnDepthPixel g_nZRes;

unsigned int g_nViewState = DEFAULT_DISPLAY_MODE;

Context g_context;
ScriptNode g_scriptNode;
DepthGenerator g_depth;
ImageGenerator g_image;
DepthMetaData g_depthMD;
ImageMetaData g_imageMD;

//---------------------------------------------------------------------------
// Functions
//---------------------------------------------------------------------------

// Let's try a more efficient visualization.
void convert_pixel_map_i(const XnRGB24Pixel* pImageMap,
                         Mat& cv_image,
                         int rows,
                         int cols) {
    int sizes[2] = { rows, cols };
    //cv_image= Mat(2, sizes, CV_8UC3, (void*) pImageMap).clone();

    Mat temp= Mat(2, sizes, CV_8UC3, (void*) pImageMap).clone();
    cvtColor(temp, cv_image, CV_BGR2RGB);
}

// Efficient!
void convert_pixel_map_d(const XnDepthPixel* pDepthMap,
                         Mat& cv_depth,
                         int rows,
                         int cols) {
    int sizes[2] = { rows, cols };
    cv_depth = Mat(2, sizes, CV_16UC1, (void*) pDepthMap).clone();
}

int main(int argc,
         char* argv[]) {

    // Kinect initialization

    XnMapOutputMode imageMapMode;
    XnMapOutputMode depthMapMode;

    XnStatus rc;

    EnumerationErrors errors;
    rc = g_context.InitFromXmlFile(SAMPLE_XML_PATH, g_scriptNode, &errors);
    if (rc == XN_STATUS_NO_NODE_PRESENT) {
        XnChar strError[1024];
        errors.ToString(strError, 1024);
        printf("%s\n", strError);
        return (rc);
    }
    else if (rc != XN_STATUS_OK) {
        printf("Open failed: %s\n", xnGetStatusString(rc));
        return (rc);
    }

    rc = g_context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_depth);
    if (rc != XN_STATUS_OK) {
        printf("No depth node exists! Check your XML.");
        return 1;
    }

    rc = g_context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_image);
    if (rc != XN_STATUS_OK) {
        printf("No image node exists! Check your XML.");
        return 1;
    }

    // Opencv initialization

    namedWindow("Display Image", CV_WINDOW_AUTOSIZE);
    namedWindow("Display Depth", CV_WINDOW_AUTOSIZE);

    int height, width, step, channels;  //parameters of the image we are working on
    int t1min = 0, t1max = 0, t2min = 0, t2max = 0, t3min = 0, t3max = 0; // other variables used

    CvMat* threshold_matrix = cvCreateMat(2, 3, CV_32FC1);

    // Load the previous values of the threshold if they exist
    if (cvOpenFileStorage("threshold_matrix.xml", NULL, CV_STORAGE_READ)) {
        threshold_matrix = (CvMat*) cvLoad("threshold_matrix.xml");
        t1min =(int) CV_MAT_ELEM(*threshold_matrix,float,0,0);
        t2min =(int) CV_MAT_ELEM(*threshold_matrix,float,0,1);
        t3min =(int) CV_MAT_ELEM(*threshold_matrix,float,0,2);
        t1max =(int) CV_MAT_ELEM(*threshold_matrix,float,1,0);
        t2max =(int) CV_MAT_ELEM(*threshold_matrix,float,1,1);
        t3max =(int) CV_MAT_ELEM(*threshold_matrix,float,1,2);
    }

// Create a window in which the captured images will be presented
    cvNamedWindow("Camera", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("HSV", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("F1", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("F2", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("F3", CV_WINDOW_AUTOSIZE);
//cvNamedWindow( "EdgeDetection", CV_WINDOW_AUTOSIZE );

/// Create Trackbars
    char TrackbarName1[50] = "t1min";
    char TrackbarName2[50] = "t1max";
    char TrackbarName3[50] = "t2min";
    char TrackbarName4[50] = "t2max";
    char TrackbarName5[50] = "t3min";
    char TrackbarName6[50] = "t3max";

    cvCreateTrackbar(TrackbarName1, "F1", &t1min, 260, NULL);
    cvCreateTrackbar(TrackbarName2, "F1", &t1max, 260, NULL);

    cvCreateTrackbar(TrackbarName3, "F2", &t2min, 260, NULL);
    cvCreateTrackbar(TrackbarName4, "F2", &t2max, 260, NULL);

    cvCreateTrackbar(TrackbarName5, "F3", &t3min, 260, NULL);
    cvCreateTrackbar(TrackbarName6, "F3", &t3max, 260, NULL);

// Load threshold from the slider bars in these 2 parameters
    Scalar hsv_min = cvScalar(t1min, t2min, t3min);
    Scalar hsv_max = cvScalar(t1max, t2max, t3max);

// get the image data
    Mat frame;
    Mat depth;

    rc = g_context.WaitAnyUpdateAll();

    g_image.GetMapOutputMode(imageMapMode);
    g_depth.GetMapOutputMode(depthMapMode);

    const XnDepthPixel* pDepthMap = g_depth.GetDepthMap();
    const XnRGB24Pixel* pImageMap = g_image.GetRGB24ImageMap();

    convert_pixel_map_d(pDepthMap, depth, depthMapMode.nYRes, depthMapMode.nXRes);
    convert_pixel_map_i(pImageMap, frame, imageMapMode.nYRes, imageMapMode.nXRes);

    height = frame.rows;
    width = frame.cols;

// capture size -
    Size size = Size(width, height);

// Initialize different images that are going to be used in the program
    Mat hsv_frame(size, CV_8UC3); // image converted to HSV plane
    Mat thresholded(size, CV_8UC1); // final thresholded image
    Mat thresholded1(size, CV_8UC1); // Component image threshold
    Mat thresholded2(size, CV_8UC1);
    Mat thresholded3(size, CV_8UC1);

    Mat thresArr[] = { thresholded1, thresholded2, thresholded3 };
    Mat filtered(size, CV_8UC1);  //smoothed image

    while (1) {
        rc = g_context.WaitAnyUpdateAll();

        g_image.GetMapOutputMode(imageMapMode);
        g_depth.GetMapOutputMode(depthMapMode);

        const XnDepthPixel* pDepthMap = g_depth.GetDepthMap();
        const XnRGB24Pixel* pImageMap = g_image.GetRGB24ImageMap();

        convert_pixel_map_d(pDepthMap, depth, depthMapMode.nYRes, depthMapMode.nXRes);
        convert_pixel_map_i(pImageMap, frame, imageMapMode.nYRes, imageMapMode.nXRes);

        //ok i have rgb and depth so call the ball detection algorithm

        // Load threshold from the slider bars in these 2 parameters
        hsv_min = Scalar(t1min, t2min, t3min);
        hsv_max = Scalar(t1max, t2max, t3max);

        // Covert color space to HSV as it is much easier to filter colors in the HSV color-space.
        cvtColor(frame, hsv_frame, CV_BGR2HSV);

        // Filter out colors which are out of range.
        inRange(hsv_frame, hsv_min, hsv_max, thresholded);

        // the below lines of code is for visual purpose only remove after calibration
        //--------------FROM HERE-----------------------------------
        //Split image into its 3 one dimensional images
        split(hsv_frame, thresArr);

        // Filter out colors which are out of range.
        inRange(thresholded1, Scalar(t1min, 0, 0), Scalar(t1max, 0, 0), thresholded1);
        inRange(thresholded2, Scalar(t2min, 0, 0), Scalar(t2max, 0, 0), thresholded2);
        inRange(thresholded3, Scalar(t3min, 0, 0), Scalar(t3max, 0, 0), thresholded3);

        //-------------REMOVE OR COMMENT AFTER CALIBRATION TILL HERE ------------------

        // hough detector works better with some smoothing of the image
        //cvSmooth(thresholded, thresholded, CV_GAUSSIAN, 9, 9);
        GaussianBlur(thresholded, thresholded, Size(9, 9), 0);

        //hough transform to detect circle
        vector < Vec3f > circles;
        HoughCircles(thresholded, circles, CV_HOUGH_GRADIENT, 2, thresholded.rows / 4, 100, 50, MIN_RADIUS, MAX_RADIUS);

        for (int i = 0; i < circles.size(); i++) {   //get the parameters of circles detected
            int z=depth.at<short int>(cvRound(circles[i][1]),cvRound(circles[i][0]));
            printf("Ball! x=%d y=%d z=%d r=%d\n\r", cvRound(circles[i][0]), cvRound(circles[i][1]), z, cvRound(circles[i][2]));

            // draw a circle with the centre and the radius obtained from the hough transform
            circle(frame, Point(cvRound(circles[i][0]), cvRound(circles[i][1])), 2, Scalar(255, 255, 255), -1, 8, 0);   //plot centre
            circle(frame, Point(cvRound(circles[i][0]), cvRound(circles[i][1])), cvRound(circles[i][2]), Scalar(0, 255, 0), 3, 8, 0);   //plot circle
        }

        /* for testing purpose you can show all the images but when done with calibration
         only show frame to keep the screen clean  */

        imshow("Camera", frame); // Original stream with detected ball overlay
        imshow("HSV", hsv_frame); // Original stream in the HSV color space
        imshow("After Color Filtering", thresholded); // The stream after color filtering
        imshow("F1", thresholded1); // individual filters
        imshow("F2", thresholded2);
        imshow("F3", thresholded3);

        //cvShowImage( "filtered", thresholded );

        imshow("Display Depth", depth);

        if ((cvWaitKey(10) & 255) == 27)
            break;

    }
// Use Mats...

// Important, since we initialized from remote pointers.
    depth.release();
    frame.release();

    return 0;
}
