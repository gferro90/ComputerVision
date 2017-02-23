/**
 * @file LineFollower.cpp
 * @brief Source file for class LineFollower
 * @date 22/feb/2017
 * @author pc
 *
 * @copyright Copyright 2015 F4E | European Joint Undertaking for ITER and
 * the Development of Fusion Energy ('Fusion for Energy').
 * Licensed under the EUPL, Version 1.1 or - as soon they will be approved
 * by the European Commission - subsequent versions of the EUPL (the "Licence")
 * You may not use this work except in compliance with the Licence.
 * You may obtain a copy of the Licence at: http://ec.europa.eu/idabc/eupl
 *
 * @warning Unless required by applicable law or agreed to in writing, 
 * software distributed under the Licence is distributed on an "AS IS"
 * basis, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
 * or implied. See the Licence permissions and limitations under the Licence.

 * @details This source file contains the definition of all the methods for
 * the class LineFollower (public, protected, and private). Be aware that some 
 * methods, such as those inline could be defined on the header file, instead.
 */

/*---------------------------------------------------------------------------*/
/*                         Standard header includes                          */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                         Project header includes                           */
/*---------------------------------------------------------------------------*/

#include "LineFollower.h"
#include "Utils.h"

/*---------------------------------------------------------------------------*/
/*                           Static definitions                              */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                           Method definitions                              */
/*---------------------------------------------------------------------------*/

using namespace cv;
using namespace std;


static stati status=FOLLOW_RIGHT;
static const char* signalAction[] = { "TurnLeft", "TurnRight", 0 };


int main(int argc,
         char* argv[]) {
    //parameters of the image we are working on
    int height = 0;
    int width = 0;
    int step;
    int channels = 0;
    // ranges of HSV boundaries
    int t1min = 0, t1max = 0, t2min = 0, t2max = 0, t3min = 0, t3max = 0;

    // Load the previous values of the threshold if they exist
    CvMat* threshold_matrix = cvCreateMat(2, 3, CV_32FC1);

    CvFileStorage* temp = cvOpenFileStorage("threshold_matrix.xml", NULL, CV_STORAGE_READ);

    if (temp) {
        threshold_matrix = (CvMat*) cvLoad("threshold_matrix.xml");
        t1min =(int) CV_MAT_ELEM(*threshold_matrix,float,0,0);
        t2min =(int) CV_MAT_ELEM(*threshold_matrix,float,0,1);
        t3min =(int) CV_MAT_ELEM(*threshold_matrix,float,0,2);
        t1max =(int) CV_MAT_ELEM(*threshold_matrix,float,1,0);
        t2max =(int) CV_MAT_ELEM(*threshold_matrix,float,1,1);
        t3max =(int) CV_MAT_ELEM(*threshold_matrix,float,1,2);
    }

// Open capture device. 0 is /dev/video0, 1 is /dev/video1, etc.
    CvCapture* capture = cvCaptureFromCAM(CAMERA);

    if (!capture) {
        fprintf(stderr, "ERROR: capture is NULL \n");
        getchar();
        return -1;
    }

// grab an image from the capture
    IplImage* frame = cvQueryFrame(capture);

// Create a window in which the captured images will be presented
    cvNamedWindow("Camera", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("HSV", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("F1", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("F2", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("F3", CV_WINDOW_AUTOSIZE);
    namedWindow("lineBand", 1);
    namedWindow("lineBandGrey", 1);
    namedWindow("lineBandGreyThres", 1);
    int threshold_value = 0;

/// Create Trackbars to filter HSV image to get signals colours
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

// Create trackbars to threshold the line for the line follower
    char* trackbar_type = "Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
    char* trackbar_value = "Value";
    int const max_value = 255;
    int const max_type = 4;
    int threshold_type = 1;
    createTrackbar(trackbar_type, "lineBandGrey", &threshold_type, max_type, NULL);

    createTrackbar(trackbar_value, "lineBandGrey", &threshold_value, max_value, NULL);

// Load threshold from the slider bars in these 2 parameters
    CvScalar hsv_min = cvScalar(t1min, t2min, t3min, 0);
    CvScalar hsv_max = cvScalar(t1max, t2max, t3max, 0);

// get the image data
    height = frame->height;
    width = frame->width;
    step = frame->widthStep;

// capture size -
    CvSize size = cvSize(width, height);

// Initialize different images that are going to be used in the program
    IplImage* hsv_frame = cvCreateImage(size, IPL_DEPTH_8U, 3); // image converted to HSV plane
    IplImage* thresholded = cvCreateImage(size, IPL_DEPTH_8U, 1); // final thresholded image
    IplImage* thresholded1 = cvCreateImage(size, IPL_DEPTH_8U, 1); // Component image threshold
    IplImage* thresholded2 = cvCreateImage(size, IPL_DEPTH_8U, 1);
    IplImage* thresholded3 = cvCreateImage(size, IPL_DEPTH_8U, 1);
    IplImage* filtered = cvCreateImage(size, IPL_DEPTH_8U, 1);  //smoothed image

//program loop
    while (1) {
#if 0

        //WIP

        // Get one frame
        frame = cvQueryFrame(capture);

        if (!frame) {
            fprintf(stderr, "ERROR: frame is null...\n");
            getchar();
            break;
        }

        int i = 0;
        int speedControl=0;
        int dirControl=0;
        while (signalAction[i] != NULL) {
            // query the color range
            ColorRange *signal=GetStreetSignalRanges(signalAction[i],signalColorRange, status);
            if(signal!=NULL){
                // filter the hsv image
                // detect the circle
                // depending on the signal detected do something
                speedControl=DetectSignal(frame, thresholded, *signal, status);
                //assume one signal per time
                break;
            }
            i++;
        }



        //cvCanny(thresholded, thresholded, 0,255);
        cv::Mat frameMat = cv::cvarrToMat(frame);

        //create the image band
        Rect rect(LINE_BAND_X(frameMat), LINE_BAND_Y(frameMat), LINE_BAND_WIDTH(frameMat), LINE_BAND_HEIGHT(frameMat));
        cv::Mat lineBand(frameMat, rect);

        //create images for gryscale and thresholded grayscale
        Mat lineBandGrey;
        Mat lineBandGreyThres;

        //convert RGB to grayscale
        cvtColor(lineBand, lineBandGrey, CV_BGR2GRAY);

        //threshold the grayscale to detect the line
        threshold(lineBandGrey, lineBandGreyThres, threshold_value, 255, threshold_type);


        dirControl=FollowLine(lineBandGreyThres, frame->width + 1, 0);

        short int controls=(speedControl<<8);
        controls|=dirControl;

        if(write(usbFD, &controls, sizeof(controls))<0){
            printf("\nUSB write ERROR!\n");
        }

#endif


        // Get one frame
        frame = cvQueryFrame(capture);

        if (!frame) {
            fprintf(stderr, "ERROR: frame is null...\n");
            getchar();
            break;
        }

        //cvCanny(thresholded, thresholded, 0,255);
        cv::Mat frameMat = cv::cvarrToMat(frame);

        //create the image band
        Rect rect(LINE_BAND_X(frameMat.rows,frameMat.cols), LINE_BAND_Y(frameMat.rows,frameMat.cols), LINE_BAND_WIDTH(frameMat.rows, frameMat.cols), LINE_BAND_HEIGHT(frameMat.rows,frameMat.cols));
        cv::Mat lineBand(frameMat, rect);

        //create images for gryscale and thresholded grayscale
        Mat lineBandGrey;
        Mat lineBandGreyThres;

        //convert RGB to grayscale
        cvtColor(lineBand, lineBandGrey, CV_BGR2GRAY);

        //threshold the grayscale to detect the line
        threshold(lineBandGrey, lineBandGreyThres, threshold_value, 255, threshold_type);

        //get the contours
        vector < vector<Point> > contours;
        vector < Vec4i > hierarchy;
        RNG rng(12345);
        Mat temp = lineBandGreyThres.clone();
        findContours(temp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

        double maxX = 0;
        double minX = frame->width + 1;
        //search the minimum and maximum x points on all contours
        for (int i = 0; i < contours.size(); i++) {
            for (int j = 0; j < contours[i].size(); j++) {

                if ((contours[i])[j].x < minX) {
                    minX = (contours[i])[j].x;
                }
                if ((contours[i])[j].x > maxX) {
                    maxX = (contours[i])[j].x;
                }
            }
        }

        printf("Min=%f Max=%f\n", minX, maxX);

        //draw contours on the band
        for (int i = 0; i < contours.size(); i++) {
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            drawContours(lineBand, contours, i, color, 2, 8, hierarchy, 0, Point());
        }
        imshow("lineBand", lineBand);
        imshow("lineBandGrey", lineBandGrey);
        imshow("lineBandGreyThres", lineBandGreyThres);

#if 0
        CvSeq* lines = cvHoughLines2(thresholded, storage, CV_HOUGH_PROBABILISTIC, 1., CV_PI / 180, 50, 50, 10);
        for (int i = 0; i < lines->total; i++) {   //get the parameters of circles detected
//            printf("Detected line %d %f %f %f %f\n",i,cvRound(p[0]), cvRound(p[1]),cvRound(p[2]), cvRound(p[3]));
            CvPoint* line = (CvPoint*) cvGetSeqElem(lines, i);
            cvLine(frame, line[0], line[1], CV_RGB(255, 0, 0), 3, 8);

            //printf("Ball! x=%f y=%f r=%f\n\r", p[0], p[1], p[2]);
            //cvLine( frame, cvPoint(cvRound(p[0]), cvRound(p[1])),cvPoint(cvRound(p[2]), cvRound(p[3])), CV_RGB(255, 0, 0), 5 );
        }
#endif



        //TODO USB Communication with the car

        // Load threshold from the slider bars in these 2 parameters
        hsv_min = cvScalar(t1min, t2min, t3min, 0);
        hsv_max = cvScalar(t1max, t2max, t3max, 0);

        // Covert color space to HSV as it is much easier to filter colors in the HSV color-space.
        cvCvtColor(frame, hsv_frame, CV_BGR2HSV);

        // Filter out colors which are out of range.
        cvInRangeS(hsv_frame, hsv_min, hsv_max, thresholded);

        // the below lines of code is for visual purpose only remove after calibration
        //--------------FROM HERE-----------------------------------
        //Split image into its 3 one dimensional images
        /*   cvSplit(hsv_frame, thresholded1, thresholded2, thresholded3, NULL);

         // Filter out colors which are out of range.
         cvInRangeS(thresholded1, cvScalar(t1min, 0, 0, 0), cvScalar(t1max, 0, 0, 0), thresholded1);
         cvInRangeS(thresholded2, cvScalar(t2min, 0, 0, 0), cvScalar(t2max, 0, 0, 0), thresholded2);
         cvInRangeS(thresholded3, cvScalar(t3min, 0, 0, 0), cvScalar(t3max, 0, 0, 0), thresholded3);
         */
        //-------------REMOVE OR COMMENT AFTER CALIBRATION TILL HERE ------------------
        // hough detector works better with some smoothing of the image
        cvSmooth(thresholded, thresholded, CV_GAUSSIAN, 9, 9);

        // circle detection
        // Memory for circles
        CvMemStorage* storage = cvCreateMemStorage(0);

        //hough transform to detect circle
        CvSeq* circles = cvHoughCircles(thresholded, storage, CV_HOUGH_GRADIENT, 2, thresholded->height / 4, 100, 50, 10, 400);

        for (int i = 0; i < circles->total; i++) {   //get the parameters of circles detected
            float* p = (float*) cvGetSeqElem(circles, i);
            //printf("Ball! x=%f y=%f r=%f\n\r",p[0],p[1],p[2] );

            //radius condition on the ball
            if (p[2] <= SIGNAL_MAX_RADIUS && p[2] >= SIGNAL_MIN_RADIUS) {

                // draw a circle with the centre and the radius obtained from the hough transform
                cvCircle(frame, cvPoint(cvRound(p[0]), cvRound(p[1])),  //plot centre
                         2, CV_RGB(255, 255, 255), -1, 8, 0);
                cvCircle(frame, cvPoint(cvRound(p[0]), cvRound(p[1])),  //plot circle
                         cvRound(p[2]), CV_RGB(0, 255, 0), 2, 8, 0);
                break;
            }
        }

        /* for testing purpose you can show all the images but when done with calibration
         only show frame to keep the screen clean  */

        cvShowImage("Camera", frame); // Original stream with detected ball overlay
        cvShowImage("HSV", hsv_frame); // Original stream in the HSV color space
        cvShowImage("After Color Filtering", thresholded); // The stream after color filtering
        /* cvShowImage( "F1", thresholded1 ); // individual filters
         cvShowImage( "F2", thresholded2 );
         cvShowImage( "F3", thresholded3 );
         */
        //cvShowImage( "filtered", thresholded );
        cvReleaseMemStorage(&storage);

        //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
        //remove higher bits using AND operator
        if ((cvWaitKey(10) & 255) == 27)
            break;
    }

//Save the threshold values before exiting
    *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 0, 0)) = t1min;
    *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 0, 1)) = t2min;
    *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 0, 2)) = t3min;
    *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 1, 0)) = t1max;
    *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 1, 1)) = t2max;
    *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 1, 2)) = t3max;
    cvSave("threshold_matrix.xml", threshold_matrix);

// Release the capture device housekeeping
    cvReleaseCapture(&capture);
    cvDestroyWindow("mywindow");
    return 0;
}
