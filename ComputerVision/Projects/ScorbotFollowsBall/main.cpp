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
#include<time.h>
#include <sys/time.h>
#include "PIDController.h"

#include </usr/include/sys/types.h>
#include </usr/include/sys/socket.h>
#include </usr/include/sys/signal.h>
#include </usr/include/netinet/in.h>
#include </usr/include/netdb.h>
#include </usr/include/fcntl.h>
#include </usr/include/arpa/inet.h>

using namespace xn;
using namespace cv;
using namespace std;
//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define SAMPLE_XML_PATH "/home/pc/Kinect/kinect/SensorKinect/OpenNI/Data/SamplesConfig.xml"
#define STORAGE_FILE "/home/pc/PHD/ComputerVision/ComputerVision/ComputerVision/Projects/ScorbotFollowsBall/threshold_matrix.xml"
//"/home/pc/Kinect/kinect/OpenNI-OpenNI-e263e59/Data/SamplesConfig.xml"

#define GL_WIN_SIZE_X 1280
#define GL_WIN_SIZE_Y 1024

#define DISPLAY_MODE_OVERLAY	1
#define DISPLAY_MODE_DEPTH		2
#define DISPLAY_MODE_IMAGE		3
#define DEFAULT_DISPLAY_MODE	DISPLAY_MODE_DEPTH

#define MAX_RADIUS 100
#define MIN_RADIUS 5

#define SPEED 500.f //100 pixel per second
#define RAD_TO_GRADE(x) ((x)/M_PI)*180
#define GRADE_TO_RAD(x) ((x)/180)*M_PI
#define L1 220
#define L2 220
#define L3 140

//gomito: alto=-1 basso=-1
#define GOMITO -1

#define KP 5

//define a deadzone radius of 2 cm
#define DEADZONE_RADIUS 50
#define X_CAM_DEADZONE 5
#define Y_CAM_DEADZONE 5
#define Z_CAM_DEADZONE 10

#define MAX_ARM_LOST_TIME 0.5
#define MAX_BALL_LOST_TIME 0.5

#define NUMBER_OF_ANGLES 6

#define IP "160.80.86.195"
#define PORT 4444

#define PITCH -45
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

struct myCoords {
    float x;
    float y;
    float z;
    float radius;
    float x_1;
    float y_1;
    float z_1;
    float radius_1;

    myCoords operator +(myCoords coords) const {
        myCoords ret;
        ret.x = x + coords.x;
        ret.y = y + coords.y;
        ret.z = z + coords.z;
        return ret;
    }

    myCoords operator -(myCoords coords) const {
        myCoords ret;
        ret.x = x - coords.x;
        ret.y = y - coords.y;
        ret.z = z - coords.z;
        return ret;
    }
    bool operator ==(const myCoords& coords) const {
        return (x == coords.x) && (y == coords.y) && (z == coords.z);
    }
    bool operator !=(const myCoords& coords) const {
        return !(*this == coords);
    }
};

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

    Mat temp = Mat(2, sizes, CV_8UC3, (void*) pImageMap).clone();
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

float absVal(float x) {
    return (x > 0) ? (x) : (-x);
}

bool IsGood(const myCoords &coords,
            float dt,
            float speed,
            int rows,
            int cols) {

    //printf("\ndt=%f\n",dt );
    // stay in the square
    float thres = speed * dt;
    float thres_x2 = ((coords.x_1 + thres) > rows) ? (rows) : (coords.x_1 + thres);
    float thres_x1 = ((coords.x_1 - thres) < 0) ? (0) : (coords.x_1 - thres);

    bool ret = (coords.x >= thres_x1) && (coords.x <= thres_x2);

    if (ret) {
        float thres_y2 = ((coords.y_1 + thres) > cols) ? (cols) : (coords.y_1 + thres);
        float thres_y1 = ((coords.y_1 - thres) < 0) ? (0) : (coords.y_1 - thres);

        ret = (coords.y >= thres_y1) && (coords.y <= thres_y2);
    }

    if (ret) {
        float thres_z2 = coords.z_1 + thres;
        float thres_z1 = ((coords.z_1 - thres) < 0) ? (0) : (coords.z_1 - thres);
        ret = (coords.z >= thres_z1) && (coords.z <= thres_z2);
    }

    //radius condition
    //TODO factor is linear??
    /*if (ret) {
     if (coords.z > coords.z_1) {
     ret = (coords.radius < coords.radius_1);
     }
     else {
     ret = (coords.radius > coords.radius_1);
     }
     }*/

    return ret;
}

void CinematicaInversa(myCoords coords,
                       float* angoli) {
    float xx, yy, zz, di, c2, s2, c1, s1, a, b;
    float diWrist = L3 * cos(GRADE_TO_RAD(PITCH));
    float xx_grip = coords.x;
    float yy_grip = coords.y;
    float zz_grip = coords.z;

    float yaw = atan2(yy_grip, xx_grip);

    xx = xx_grip - diWrist * cos(yaw);
    yy = yy_grip - diWrist * sin(yaw);
    zz = zz_grip - L3 * sin(GRADE_TO_RAD(PITCH));

    angoli[0] = RAD_TO_GRADE(yaw);

    //out of the workspace
    if ((xx * xx + yy * yy + zz * zz) >= (L1 * L1 + L2 * L2)) {
        angoli[1] = RAD_TO_GRADE(atan2(zz, xx));
        angoli[2] = RAD_TO_GRADE(atan2(zz, xx));
    }
    else {
        di = sqrt(xx * xx + yy * yy);
        c2 = (di * di + zz * zz - L1 * L1 - L2 * L2) / (2 * L1 * L2);
        s2 = GOMITO * sqrt(1 - c2 * c2);
        angoli[2] = RAD_TO_GRADE(atan2(s2, c2));
        a = c2 * L2 + L1;
        b = s2 * L2;
        c1 = (zz * b + di * a) / (a * a + b * b);
        s1 = (zz - c1 * b) / a;
        angoli[1] = RAD_TO_GRADE(atan2(s1, c1));
        angoli[2] += angoli[1];
    }
    angoli[3] = PITCH;
}

void CinematicaDiretta(float *angoli,
                       myCoords &coords) {

    float a0, a1, a2, di;
    //yaw
    a0 = GRADE_TO_RAD(angoli[0]);
    a1 = GRADE_TO_RAD(angoli[1]);
    a2 = GRADE_TO_RAD(angoli[2]);

    di = L1 * cos(a1) + L2 * cos(a2) + L3 * cos(GRADE_TO_RAD(PITCH));
    coords.z = L1 * sin(a1) + L2 * sin(a2) + L3 * sin(GRADE_TO_RAD(PITCH));
    coords.x = di * cos(a0);
    coords.y = di * sin(a0);
}

void ComputeDeadzone(const myCoords &gripperPos,
                     myCoords &newGripperRef,
                     float deadRadius) {
    myCoords dCoords = gripperPos - newGripperRef;
    float dist = dCoords.x * dCoords.x + dCoords.y * dCoords.y + dCoords.z * dCoords.z;
    //avoid vibrations that might be caused from computation errors
    //printf("d=%f", sqrt(dist));
    if (sqrt(dist) <= deadRadius) {
        newGripperRef = gripperPos;
    }
}

int Connetti() {
    struct sockaddr_in client;
    int lunghezza = sizeof(client);
    int idsock = socket(AF_INET, SOCK_STREAM, 0);
    client.sin_family = AF_INET;
    client.sin_port = htons(PORT);
    inet_aton(IP, &(client.sin_addr));
    if (connect(idsock, (struct sockaddr*) &client, sizeof(client)) == -1) {
        return -1;
    }
    else {
        return idsock;
    }
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


    int height, width, step, channels;  //parameters of the image we are working on
    int t1min_arm = 0, t1max_arm = 0, t2min_arm = 0, t2max_arm = 0, t3min_arm = 0, t3max_arm = 0; // other variables used
    int t1min_ball = 0, t1max_ball = 0, t2min_ball = 0, t2max_ball = 0, t3min_ball = 0, t3max_ball = 0; // other variables used
    int max_ball_radius, min_ball_radius, max_arm_radius, min_arm_radius;
    int ball_precision, arm_precision;
    int arm_min_dist, ball_min_dist;

    height = frame.rows;
    width = frame.cols;

    // capture size -
    Size size = Size(width, height);

    // Opencv initialization

    namedWindow("Display Depth", CV_WINDOW_AUTOSIZE);


    CvMat* threshold_matrix = cvCreateMat(7, 3, CV_32FC1);

    // Load the previous values of the threshold if they exist
    if (cvOpenFileStorage(STORAGE_FILE, NULL, CV_STORAGE_READ)) {
        threshold_matrix = (CvMat*) cvLoad(STORAGE_FILE);
//#if 0
        t1min_arm =(int) CV_MAT_ELEM(*threshold_matrix,float,0,0);
        t2min_arm =(int) CV_MAT_ELEM(*threshold_matrix,float,0,1);
        t3min_arm =(int) CV_MAT_ELEM(*threshold_matrix,float,0,2);
        t1max_arm =(int) CV_MAT_ELEM(*threshold_matrix,float,1,0);
        t2max_arm =(int) CV_MAT_ELEM(*threshold_matrix,float,1,1);
        t3max_arm =(int) CV_MAT_ELEM(*threshold_matrix,float,1,2);
        t1min_ball =(int) CV_MAT_ELEM(*threshold_matrix,float,2,0);
        t2min_ball =(int) CV_MAT_ELEM(*threshold_matrix,float,2,1);
        t3min_ball =(int) CV_MAT_ELEM(*threshold_matrix,float,2,2);
        t1max_ball =(int) CV_MAT_ELEM(*threshold_matrix,float,3,0);
        t2max_ball =(int) CV_MAT_ELEM(*threshold_matrix,float,3,1);
        t3max_ball =(int) CV_MAT_ELEM(*threshold_matrix,float,3,2);

        max_arm_radius =(int) CV_MAT_ELEM(*threshold_matrix,float,4,0);
        min_arm_radius =(int) CV_MAT_ELEM(*threshold_matrix,float,4,1);
        arm_min_dist =(int) CV_MAT_ELEM(*threshold_matrix,float,4,2);

        max_ball_radius =(int) CV_MAT_ELEM(*threshold_matrix,float,5,0);
        min_ball_radius =(int) CV_MAT_ELEM(*threshold_matrix,float,5,1);
        ball_min_dist =(int) CV_MAT_ELEM(*threshold_matrix,float,5,2);

        arm_precision =(int) CV_MAT_ELEM(*threshold_matrix,float,6,0);
        ball_precision =(int) CV_MAT_ELEM(*threshold_matrix,float,6,1);

//#endif
    }

// Create a window in which the captured images will be presented
    cvNamedWindow("Camera", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("HSV", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("F1_ARM", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("F2_ARM", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("F3_ARM", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("F1_BALL", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("F2_BALL", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("F3_BALL", CV_WINDOW_AUTOSIZE);

//cvNamedWindow( "EdgeDetection", CV_WINDOW_AUTOSIZE );

/// Create Trackbars
    char TrackbarName1[50] = "t1min_arm";
    char TrackbarName2[50] = "t1max_arm";
    char TrackbarName3[50] = "t2min_arm";
    char TrackbarName4[50] = "t2max_arm";
    char TrackbarName5[50] = "t3min_arm";
    char TrackbarName6[50] = "t3max_arm";

    char TrackbarName7[50] = "t1min_ball";
    char TrackbarName8[50] = "t1max_ball";
    char TrackbarName9[50] = "t2min_ball";
    char TrackbarName10[50] = "t2max_ball";
    char TrackbarName11[50] = "t3min_ball";
    char TrackbarName12[50] = "t3max_ball";

    char TrackbarName13[50] = "min_arm_radius";
    char TrackbarName14[50] = "max_arm_radius";

    char TrackbarName15[50] = "min_ball_radius";
    char TrackbarName16[50] = "max_ball_radius";


    char TrackbarName17[50] = "arm_precision";
    char TrackbarName18[50] = "ball_precision";

    char TrackbarName19[50] = "arm_min_dist";
    char TrackbarName20[50] = "ball_min_dist";

    cvCreateTrackbar(TrackbarName1, "F1_ARM", &t1min_arm, 260, NULL);
    cvCreateTrackbar(TrackbarName2, "F1_ARM", &t1max_arm, 260, NULL);

    cvCreateTrackbar(TrackbarName3, "F2_ARM", &t2min_arm, 260, NULL);
    cvCreateTrackbar(TrackbarName4, "F2_ARM", &t2max_arm, 260, NULL);

    cvCreateTrackbar(TrackbarName5, "F3_ARM", &t3min_arm, 260, NULL);
    cvCreateTrackbar(TrackbarName6, "F3_ARM", &t3max_arm, 260, NULL);

    cvCreateTrackbar(TrackbarName7, "F1_BALL", &t1min_ball, 260, NULL);
    cvCreateTrackbar(TrackbarName8, "F1_BALL", &t1max_ball, 260, NULL);

    cvCreateTrackbar(TrackbarName9, "F2_BALL", &t2min_ball, 260, NULL);
    cvCreateTrackbar(TrackbarName10, "F2_BALL", &t2max_ball, 260, NULL);

    cvCreateTrackbar(TrackbarName11, "F3_BALL", &t3min_ball, 260, NULL);
    cvCreateTrackbar(TrackbarName12, "F3_BALL", &t3max_ball, 260, NULL);

    cvCreateTrackbar(TrackbarName13, "Camera", &min_arm_radius, MAX_RADIUS, NULL);
    cvCreateTrackbar(TrackbarName14, "Camera", &max_arm_radius, MAX_RADIUS, NULL);

    cvCreateTrackbar(TrackbarName15, "Camera", &min_ball_radius, MAX_RADIUS, NULL);
    cvCreateTrackbar(TrackbarName16, "Camera", &max_ball_radius, MAX_RADIUS, NULL);

    cvCreateTrackbar(TrackbarName17, "Camera", &arm_precision, 150, NULL);
    cvCreateTrackbar(TrackbarName18, "Camera", &ball_precision, 150, NULL);

    cvCreateTrackbar(TrackbarName19, "Camera", &arm_min_dist, width, NULL);
    cvCreateTrackbar(TrackbarName20, "Camera", &ball_min_dist, width, NULL);

// Load threshold from the slider bars in these 2 parameters
    Scalar hsv_min_arm = cvScalar(t1min_arm, t2min_arm, t3min_arm);
    Scalar hsv_max_arm = cvScalar(t1max_arm, t2max_arm, t3max_arm);
    Scalar hsv_min_ball = cvScalar(t1min_ball, t2min_ball, t3min_ball);
    Scalar hsv_max_ball = cvScalar(t1max_ball, t2max_ball, t3max_ball);

// Initialize different images that are going to be used in the program
    Mat hsv_frame(size, CV_8UC3); // image converted to HSV plane
    Mat thresholded_arm(size, CV_8UC1); // final thresholded image
    Mat thresholded_ball(size, CV_8UC1); // final thresholded image
    Mat thresholded1_arm(size, CV_8UC1); // Component image threshold
    Mat thresholded2_arm(size, CV_8UC1);
    Mat thresholded3_arm(size, CV_8UC1);

    Mat thresholded1_ball(size, CV_8UC1); // Component image threshold
    Mat thresholded2_ball(size, CV_8UC1);
    Mat thresholded3_ball(size, CV_8UC1);

    Mat thresArr_arm[] = { thresholded1_arm, thresholded2_arm, thresholded3_arm };
    Mat thresArr_ball[] = { thresholded1_ball, thresholded2_ball, thresholded3_ball };
    Mat filtered(size, CV_8UC1);  //smoothed image

//calibration
//get the manipulator
//get the ball

    int numberOfBallsFound = 0;
//manual calibration

    myCoords armCoords;
    myCoords ballCoords;

    bool calibrationDone = false;

    struct timeval stop, start;

    float armLostTime = 0., ballLostTime = 0.;

    float speed = SPEED;

    bool quit = false;
    bool firstTime = true;
    gettimeofday(&start, NULL);

    int idSock = Connetti();
    if (idSock < 0) {
        printf("\nFailed connection to robot!!\n");
    }
    while (!quit) {

        rc = g_context.WaitAnyUpdateAll();

        g_image.GetMapOutputMode(imageMapMode);
        g_depth.GetMapOutputMode(depthMapMode);

        const XnDepthPixel* pDepthMap = g_depth.GetDepthMap();
        const XnRGB24Pixel* pImageMap = g_image.GetRGB24ImageMap();

        convert_pixel_map_d(pDepthMap, depth, depthMapMode.nYRes, depthMapMode.nXRes);
        convert_pixel_map_i(pImageMap, frame, imageMapMode.nYRes, imageMapMode.nXRes);

        //ok i have rgb and depth so call the ball detection algorithm

        // Load threshold from the slider bars in these 2 parameters
        hsv_min_arm = Scalar(t1min_arm, t2min_arm, t3min_arm);
        hsv_max_arm = Scalar(t1max_arm, t2max_arm, t3max_arm);

        // Covert color space to HSV as it is much easier to filter colors in the HSV color-space.
        cvtColor(frame, hsv_frame, CV_BGR2HSV);

        // Filter out colors which are out of range.
        inRange(hsv_frame, hsv_min_arm, hsv_max_arm, thresholded_arm);

        // the below lines of code is for visual purpose only remove after calibration
        //--------------FROM HERE-----------------------------------
        //Split image into its 3 one dimensional images
        split(hsv_frame, thresArr_arm);

        // Filter out colors which are out of range.
        inRange(thresholded1_arm, Scalar(t1min_arm, 0, 0), Scalar(t1max_arm, 0, 0), thresholded1_arm);
        inRange(thresholded2_arm, Scalar(t2min_arm, 0, 0), Scalar(t2max_arm, 0, 0), thresholded2_arm);
        inRange(thresholded3_arm, Scalar(t3min_arm, 0, 0), Scalar(t3max_arm, 0, 0), thresholded3_arm);

        //-------------REMOVE OR COMMENT AFTER CALIBRATION TILL HERE ------------------

        // hough detector works better with some smoothing of the image
        //cvSmooth(thresholded, thresholded, CV_GAUSSIAN, 9, 9);2
        GaussianBlur(thresholded_arm, thresholded_arm, Size(9, 9), 0);

        //hough transform to detect circle
        vector < Vec3f > circles_arm;
        HoughCircles(thresholded_arm, circles_arm, CV_HOUGH_GRADIENT, 2, arm_min_dist, 100, arm_precision, min_arm_radius, max_arm_radius);

        float dt = 0;
        bool done = false;
        gettimeofday(&stop, NULL);
        dt = (stop.tv_sec - start.tv_sec) + ((stop.tv_usec - start.tv_usec)) * 1e-6;

        // printf("\ndt_aaa=%f\n",dt );

        armLostTime += dt;
        ballLostTime += dt;

        //printf("\narmLostTime=%f\n", armLostTime);
        //printf("\nballLostTime=%f\n", ballLostTime);

        gettimeofday(&start, NULL);

        //find arm
        for (int i = 0; (i < circles_arm.size()) && (!done); i++) {   //get the parameters of circles detected

            // just take the good ones
            armCoords.x = circles_arm[i][0];
            armCoords.y = circles_arm[i][1];
            armCoords.z = depth.at<short int>(cvRound(circles_arm[i][1]), cvRound(circles_arm[i][0]));
            armCoords.radius = circles_arm[i][2];

            if (firstTime) {
                armCoords.x_1 = armCoords.x;
                armCoords.y_1 = armCoords.y;
                armCoords.z_1 = armCoords.z;
                armCoords.radius_1 = armCoords.radius;
            }

            if (calibrationDone) {
                if (IsGood(armCoords, armLostTime, speed, frame.rows, frame.cols)) {
                    printf("Arm! x=%d y=%d z=%d r=%d\n\r", cvRound(armCoords.x), cvRound(armCoords.y), cvRound(armCoords.z), cvRound(armCoords.radius));
                    // draw a circle with the centre and the radius obtained from the hough transform
                    circle(frame, Point(cvRound(armCoords.x), cvRound(armCoords.y)), 2, Scalar(255, 255, 255), -1, 8, 0);   //plot centre
                    circle(frame, Point(cvRound(armCoords.x), cvRound(armCoords.y)), cvRound(armCoords.radius), Scalar(0, 255, 0), 3, 8, 0);   //plot circle
                    putText(frame, "Scorbot Arm", Point(cvRound(armCoords.x), cvRound(armCoords.y)), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 0, 0), 2.0);
                    //the arm has been found... break

                    armCoords.x_1 = armCoords.x;
                    //#endif
                    armCoords.y_1 = armCoords.y;
                    armCoords.z_1 = armCoords.z;
                    armCoords.radius_1 = armCoords.radius;
                    armLostTime = 0.;
                    done = true;

                }
            }
            else {
                circle(frame, Point(cvRound(armCoords.x), cvRound(armCoords.y)), 2, Scalar(255, 255, 255), -1, 8, 0);   //plot centre
                circle(frame, Point(cvRound(armCoords.x), cvRound(armCoords.y)), cvRound(armCoords.radius), Scalar(0, 255, 0), 3, 8, 0);   //plot circle

                armCoords.x_1 = armCoords.x;
                armCoords.y_1 = armCoords.y;
                armCoords.z_1 = armCoords.z;
                armCoords.radius_1 = armCoords.radius;
            }

        }

// Load threshold from the slider bars in these 2 parameters
        hsv_min_ball = Scalar(t1min_ball, t2min_ball, t3min_ball);
        hsv_max_ball = Scalar(t1max_ball, t2max_ball, t3max_ball);

// Filter out colors which are out of range.
        inRange(hsv_frame, hsv_min_ball, hsv_max_ball, thresholded_ball);

// the below lines of code is for visual purpose only remove after calibration
//--------------FROM HERE-----------------------------------
//Split image into its 3 one dimensional images
        split(hsv_frame, thresArr_ball);

// Filter out colors which are out of range.
        inRange(thresholded1_ball, Scalar(t1min_ball, 0, 0), Scalar(t1max_ball, 0, 0), thresholded1_ball);
        inRange(thresholded2_ball, Scalar(t2min_ball, 0, 0), Scalar(t2max_ball, 0, 0), thresholded2_ball);
        inRange(thresholded3_ball, Scalar(t3min_ball, 0, 0), Scalar(t3max_ball, 0, 0), thresholded3_ball);

//-------------REMOVE OR COMMENT AFTER CALIBRATION TILL HERE ------------------

// hough detector works better with some smoothing of the image
//cvSmooth(thresholded, thresholded, CV_GAUSSIAN, 9, 9);2
        GaussianBlur(thresholded_ball, thresholded_ball, Size(9, 9), 0);

//hough transform to detect circle
        vector < Vec3f > circles_ball;
        //#endif
        HoughCircles(thresholded_ball, circles_ball, CV_HOUGH_GRADIENT, 2, ball_min_dist, 100, ball_precision, min_ball_radius, max_ball_radius);

        done = false;
//find ball
        for (int i = 0; (i < circles_ball.size()) && (!done); i++) {   //get the parameters of circles detected

            // just take the good ones
            ballCoords.x = circles_ball[i][0];
            ballCoords.y = circles_ball[i][1];
            ballCoords.z = depth.at<short int>(cvRound(circles_ball[i][1]), cvRound(circles_ball[i][0]));
            ballCoords.radius = circles_ball[i][2];

            if (firstTime) {
                ballCoords.x_1 = ballCoords.x;
                ballCoords.y_1 = ballCoords.y;
                ballCoords.z_1 = ballCoords.z;
                ballCoords.radius_1 = ballCoords.radius;
                firstTime = false;
            }
            if (calibrationDone) {
                if (IsGood(ballCoords, ballLostTime, speed, frame.rows, frame.cols)) {
                    printf("Ball! x=%d y=%d z=%d r=%d\n\r", cvRound(ballCoords.x), cvRound(ballCoords.y), cvRound(ballCoords.z), cvRound(ballCoords.radius));
                    // draw a circle with the centre and the radius obtained from the hough transform
                    circle(frame, Point(cvRound(ballCoords.x), cvRound(ballCoords.y)), 2, Scalar(255, 255, 255), -1, 8, 0);   //plot centre
                    circle(frame, Point(cvRound(ballCoords.x), cvRound(ballCoords.y)), cvRound(ballCoords.radius), Scalar(0, 255, 0), 3, 8, 0);   //plot circle
                    putText(frame, "Ball", Point(cvRound(ballCoords.x), cvRound(ballCoords.y)), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 0, 0), 2.0);
                    //the arm has been found... break

                    ballCoords.x_1 = ballCoords.x;
                    ballCoords.y_1 = ballCoords.y;
                    ballCoords.z_1 = ballCoords.z;
                    ballCoords.radius_1 = ballCoords.radius;
                    ballLostTime = 0.;
                    done = true;
                }
            }
            else {
                circle(frame, Point(cvRound(ballCoords.x), cvRound(ballCoords.y)), 2, Scalar(255, 255, 255), -1, 8, 0);   //plot centre
                circle(frame, Point(cvRound(ballCoords.x), cvRound(ballCoords.y)), cvRound(ballCoords.radius), Scalar(0, 255, 0), 3, 8, 0);   //plot circle

                ballCoords.x_1 = ballCoords.x;
                ballCoords.y_1 = ballCoords.y;
                ballCoords.z_1 = ballCoords.z;
                ballCoords.radius_1 = ballCoords.radius;
            }
        }

        //CONTROL
        if (calibrationDone) {
#if 0
            float angles[4]= {0, 0, 0, 0};
            for(int cnt=0; cnt<100; cnt++) {
                for (int l=0; l<3; l++) {
                    angles[l]=cnt*(360/100.);
                }
                myCoords proof;
                printf("\n input %f %f %f %f", angles[0], angles[1], angles[2], angles[3]);
                CinematicaDiretta(angles, proof);
                printf("\nCD: x=%f, y=%f, z=%f", proof.x, proof.y, proof.z );
                float out[4];
                CinematicaInversa(proof, out);
                printf("\n CI %f %f %f %f", out[0], out[1], out[2], out[3]);
                CinematicaDiretta(out, proof);
                printf("\nCD: x=%f, y=%f, z=%f\n", proof.x, proof.y, proof.z );

            }
#endif
            //read angles from remote
            myCoords gripperPos;
            float angoli[NUMBER_OF_ANGLES];
            memset(angoli, 0, sizeof(angoli));
            if (idSock > 0) {
                if (read(idSock, angoli, sizeof(angoli)) > 0) {

                    myCoords cameraError;
                    cameraError.x = (ballCoords.x - armCoords.x);
                    cameraError.y = (ballCoords.y - armCoords.y);
                    cameraError.z = (ballCoords.z - armCoords.z);

                    if (absVal(cameraError.x) < (X_CAM_DEADZONE + armCoords.radius + ballCoords.radius)) {
                        cameraError.x = 0;
                    }
                    if (absVal(cameraError.y) < (Y_CAM_DEADZONE + armCoords.radius + ballCoords.radius)) {
                        cameraError.y = 0;
                    }
                    if (absVal(cameraError.z) < Z_CAM_DEADZONE) {
                        cameraError.z = 0;
                    }
                    PIDController pid(KP);
                    //compute the current scorbot gripper position
                    CinematicaDiretta(angoli, gripperPos);

                    //mem copy
                    myCoords newGripperRef = gripperPos;

                    //add an error to the current position proportional to the camera error
                    if ((ballLostTime <= MAX_BALL_LOST_TIME) && (armLostTime <= MAX_ARM_LOST_TIME)) {
                        newGripperRef.x += pid.Execute(cameraError.x, dt);
                        newGripperRef.z += pid.Execute(-cameraError.y, dt);
                        // don't use pid or use a different one here because the z is in millimeters from cam!
                        newGripperRef.y += -cameraError.z;   //pid.Execute(cameraError.z, dt);
                    }
                    /*
                     newGripperRef.x = 200;
                     newGripperRef.y = 200;
                     newGripperRef.z = 400;
                     */

                    //avoid vibrations that might be caused from computation errors
                    ComputeDeadzone(gripperPos, newGripperRef, DEADZONE_RADIUS);
                    //compute the inverse kinematic
                    if (newGripperRef != gripperPos) {
                        CinematicaInversa(newGripperRef, angoli);
                    }
                    /*for(int cnt=0; cnt<4; cnt++){
                     printf("\nangolo[%d]=%f", cnt, angoli[cnt]);
                     }*/
                    write(idSock, angoli, sizeof(angoli));
                }
            }
        }

        /* for testing purpose you can show all the images but when done with calibration
         only show frame to keep the screen clean  */

        imshow("Camera", frame); // Original stream with detected ball overlay
        imshow("HSV", hsv_frame); // Original stream in the HSV color space
        imshow("After Color Filtering Arm", thresholded_arm); // The stream after color filtering
        imshow("After Color Filtering Ball", thresholded_ball); // The stream after color filtering
        imshow("F1_ARM", thresholded1_arm); // individual filters
        imshow("F2_ARM", thresholded2_arm);
        imshow("F3_ARM", thresholded3_arm);
        imshow("F1_BALL", thresholded1_ball); // individual filters
        imshow("F2_BALL", thresholded2_ball);
        imshow("F3_BALL", thresholded3_ball);

//cvShowImage( "filtered", thresholded );

        imshow("Display Depth", depth);

        switch ((cvWaitKey(10) & 255)) {
//press space to finish calibration
        case 32: {
            printf("\nCALIBRATION DONE!!\n");
            calibrationDone = true;
        }
            break;
        case 27: {
            quit = true;
        }
            break;
        }

//exit if ESC

    }
// Use Mats...
    *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 0, 0)) = t1min_arm;
    *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 0, 1)) = t2min_arm;
    *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 0, 2)) = t3min_arm;
    *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 1, 0)) = t1max_arm;
    *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 1, 1)) = t2max_arm;
    *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 1, 2)) = t3max_arm;
    *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 2, 0)) = t1min_ball;
    *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 2, 1)) = t2min_ball;
    *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 2, 2)) = t3min_ball;
    *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 3, 0)) = t1max_ball;
    *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 3, 1)) = t2max_ball;
    *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 3, 2)) = t3max_ball;

    *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 4, 0)) = min_arm_radius;
    *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 4, 1)) = max_arm_radius;
    *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 4, 2)) = arm_min_dist;


    *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 5, 0)) = min_ball_radius;
    *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 5, 1)) = max_ball_radius;
    *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 5, 2)) = ball_min_dist;

    *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 6, 0)) = arm_precision;
    *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 6, 1)) = ball_precision;

    cvSave(STORAGE_FILE, threshold_matrix);

// Important, since we initialized from remote pointers.
    depth.release();
    frame.release();

    return 0;
}
