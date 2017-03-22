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
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include "SceneDrawer.h"
#include <XnPropNames.h>
#include <math.h>
#include <iostream>
#include </usr/include/sys/types.h>
#include </usr/include/sys/socket.h>
#include </usr/include/sys/signal.h>
#include </usr/include/netinet/in.h>
#include </usr/include/netdb.h>
#include </usr/include/fcntl.h>
#include </usr/include/arpa/inet.h>

#include <cv.h>
#include <highgui.h>
#include <cxcore.h>
#include <opencv2/core/core.hpp>
//#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;
using namespace xn;

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
CvMat* threshold_matrix;
xn::Context g_Context;
xn::ScriptNode g_scriptNode;
xn::DepthGenerator g_DepthGenerator;
xn::ImageGenerator g_image;
xn::UserGenerator g_UserGenerator;
XnMapOutputMode imageMapMode;

xn::Player g_Player;

XnBool g_bNeedPose = FALSE;
XnChar g_strPose[20] = "";
XnBool g_bDrawBackground = TRUE;
XnBool g_bDrawPixels = TRUE;
XnBool g_bDrawSkeleton = TRUE;
XnBool g_bPrintID = TRUE;
XnBool g_bPrintState = TRUE;

XnBool g_bPrintFrameID = FALSE;
XnBool g_bMarkJoints = FALSE;

Mat frame;
Mat hsv_frame; // image converted to HSV plane
Mat thresholded; // final thresholded image

int t1min = 0, t1max = 0, t2min = 0, t2max = 0, t3min = 0, t3max = 0; // other variables used
XnStatus nRetVal;

#ifndef USE_GLES
#if (XN_PLATFORM == XN_PLATFORM_MACOSX)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#else
#include "opengles.h"
#endif

#ifdef USE_GLES
static EGLDisplay display = EGL_NO_DISPLAY;
static EGLSurface surface = EGL_NO_SURFACE;
static EGLContext context = EGL_NO_CONTEXT;
#endif

#define GL_WIN_SIZE_X 720
#define GL_WIN_SIZE_Y 480

#define HAND_OPEN_LENGTH 20
#define HAND_CLOSE_LENGTH 20

#define ARM_OPEN_LENGTH 70
#define ARM_CLOSE_LENGTH 100

#define CYCLES_GRIPPER 50

#define L1 220
#define L2 220
#define L3 140

#define GRIPPER 0x1
#define CIN_INV 0x2

#define MINIMUM_Z -100
#define NUMBER_OF_ANGLES 6

#define RAD_TO_GRADE(x) ((x)/M_PI)*180
#define GRADE_TO_RAD(x) ((x)/180)*M_PI

#define GOMITO -1

#define PITCH -90

#define IP "160.80.86.195"
#define PORT 4444

#define STORAGE_FILE "/home/pc/PHD/ComputerVision/ComputerVision/ComputerVision/Projects/KinectScorbot/threshold_matrix.xml"
#define SAMPLE_XML_PATH "/home/pc/Kinect/kinect/SensorKinect/OpenNI/Data/SamplesConfig.xml"

//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------

int idsock = -1;
float initialArmLength = 0.;
float initialShoulderLength = 0.;

XnBool g_bPause = false;
XnBool g_bRecord = false;

XnBool g_bQuit = false;
bool skeletonTracked = false;
bool primavolta = true;
int flag = 0;

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

void CleanupExit() {
    g_scriptNode.Release();
    g_DepthGenerator.Release();
    g_UserGenerator.Release();
    g_Player.Release();
    g_Context.Release();

    exit(1);
}

// Callback: New user was detected
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& /*generator*/, XnUserID nId, void* /*pCookie*/) {
    XnUInt32 epochTime = 0;
    xnOSGetEpochTime(&epochTime);
    printf("%d New User %d\n", epochTime, nId);
    // New user found
    if (g_bNeedPose)
    {
        g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
    }
    else
    {
        g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
    }
}
// Callback: An existing user was lost
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& /*generator*/, XnUserID nId, void* /*pCookie*/) {
    XnUInt32 epochTime = 0;
    xnOSGetEpochTime(&epochTime);
    printf("%d Lost user %d\n", epochTime, nId);
    //skeletonTracked=false;
}
// Callback: Detected a pose
void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& /*capability*/, const XnChar* strPose, XnUserID nId, void* /*pCookie*/)
{
    XnUInt32 epochTime = 0;
    xnOSGetEpochTime(&epochTime);
    printf("%d Pose %s detected for user %d\n", epochTime, strPose, nId);
    g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}
// Callback: Started calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& /*capability*/, XnUserID nId, void* /*pCookie*/)
{
    XnUInt32 epochTime = 0;
    xnOSGetEpochTime(&epochTime);
    printf("%d Calibration started for user %d\n", epochTime, nId);
}
// Callback: Finished calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationComplete(xn::SkeletonCapability& /*capability*/, XnUserID nId, XnCalibrationStatus eStatus, void* /*pCookie*/)
{
    XnUInt32 epochTime = 0;
    xnOSGetEpochTime(&epochTime);
    if (eStatus == XN_CALIBRATION_STATUS_OK)
    {   skeletonTracked=true;
        // Calibration succeeded
        printf("%d Calibration complete, start tracking user %d\n", epochTime, nId);
        g_UserGenerator.GetSkeletonCap().StartTracking(nId);
    }
    else
    {
        // Calibration failed
        printf("%d Calibration failed for user %d\n", epochTime, nId);
        if(eStatus==XN_CALIBRATION_STATUS_MANUAL_ABORT)
        {
            printf("Manual abort occured, stop attempting to calibrate!");
            return;
        }
        if (g_bNeedPose)
        {
            g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
        }
        else
        {
            g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
        }
    }
}

#define XN_CALIBRATION_FILE_NAME "UserCalibration.bin"

// Save calibration to file
void SaveCalibration() {
    XnUserID aUserIDs[20] = { 0 };
    XnUInt16 nUsers = 20;
    g_UserGenerator.GetUsers(aUserIDs, nUsers);
    for (int i = 0; i < nUsers; ++i) {
        // Find a user who is already calibrated
        if (g_UserGenerator.GetSkeletonCap().IsCalibrated(aUserIDs[i])) {
            // Save user's calibration to file
            g_UserGenerator.GetSkeletonCap().SaveCalibrationDataToFile(aUserIDs[i], XN_CALIBRATION_FILE_NAME);
            break;
        }
    }
}

// Load calibration from file
void LoadCalibration() {
    XnUserID aUserIDs[20] = { 0 };
    XnUInt16 nUsers = 20;
    g_UserGenerator.GetUsers(aUserIDs, nUsers);
    for (int i = 0; i < nUsers; ++i) {
        // Find a user who isn't calibrated or currently in pose
        if (g_UserGenerator.GetSkeletonCap().IsCalibrated(aUserIDs[i]))
            continue;
        if (g_UserGenerator.GetSkeletonCap().IsCalibrating(aUserIDs[i]))
            continue;

        // Load user's calibration from file
        XnStatus rc = g_UserGenerator.GetSkeletonCap().LoadCalibrationDataFromFile(aUserIDs[i], XN_CALIBRATION_FILE_NAME);
        if (rc == XN_STATUS_OK) {
            // Make sure state is coherent
            g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(aUserIDs[i]);
            g_UserGenerator.GetSkeletonCap().StartTracking(aUserIDs[i]);
        }
        break;
    }
}

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

int sign(XnFloat a) {
    if (a < 0)
        return -1;
    else
        return 1;
}

#if 0
void CinematicaInversa(XnSkeletonJointPosition mano,
        XnSkeletonJointPosition spalla,
        float* angoli) {
    float xx, yy, zz, di, c2, s2, c1, s1, a, b, L1 = 220, L2 = 220;
    xx = -(mano.position.X - spalla.position.X);
    yy = -(mano.position.Z - spalla.position.Z);
    zz = mano.position.Y - spalla.position.Y;
    di = sqrt(xx * xx + yy * yy);
    c2 = (di * di + zz * zz - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    s2 = sqrt(1 - c2 * c2);
    angoli[2] = tograde(atan2(s2, c2));
    a = c2 * L2 + L1;
    b = s2 * L2;
    c1 = (zz * b + di * a) / (a * a + b * b);
    s1 = (c1 * a - di) / b;
    angoli[1] = tograde(atan2(s1, c1));
    angoli[0] = tograde(atan2(yy, xx));
}
#endif

void CinematicaInversa(XnSkeletonJointPosition mano,
                       XnSkeletonJointPosition spalla,
                       float* angoli) {
    float xx, yy, zz, di, c2, s2, c1, s1, a, b;

    float diWrist = L3 * cos(GRADE_TO_RAD(PITCH));
    float xx_grip = ((mano.position.X - spalla.position.X));
    float yy_grip = -((mano.position.Z - spalla.position.Z));
    float zz_grip = ((mano.position.Y - spalla.position.Y));

    float yaw = atan2(yy_grip, xx_grip);

    xx = xx_grip - diWrist * cos(yaw);
    yy = yy_grip - diWrist * sin(yaw);
    zz = zz_grip - L3 * sin(GRADE_TO_RAD(PITCH));

    if (zz < MINIMUM_Z) {
        zz = MINIMUM_Z;
    }

    angoli[0] = RAD_TO_GRADE(yaw);
    printf("\nzz=%f\n", zz);
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

void CalcolaAngoli(XnSkeletonJointPosition mano,
                   XnSkeletonJointPosition gomito,
                   XnSkeletonJointPosition spalla,
                   float* angoli) {
    angoli[0] = atan2(-(mano.position.Z - spalla.position.Z), mano.position.X - spalla.position.X);
    angoli[1] = atan2(
            gomito.position.Y - spalla.position.Y,
            sign(gomito.position.X - spalla.position.X)
                    * sqrt((gomito.position.X - spalla.position.X) * (gomito.position.X - spalla.position.X)
                            + (gomito.position.Z - spalla.position.Z) * (gomito.position.Z - spalla.position.Z)));
    angoli[2] = atan2(
            mano.position.Y - gomito.position.Y,
            sign(mano.position.X - gomito.position.X)
                    * sqrt((mano.position.X - gomito.position.X) * (mano.position.X - gomito.position.X)
                            + (mano.position.Z - gomito.position.Z) * (mano.position.Z - gomito.position.Z)));
    angoli[0] = (angoli[0] / 3.14) * 180;
    angoli[1] = (angoli[1] / 3.14) * 180;
    angoli[2] = (angoli[2] / 3.14) * 180;
    angoli[3] = angoli[2];
}

float distanza(XnSkeletonJointPosition joint1,
               XnSkeletonJointPosition joint2) {
    return sqrt(
            (joint1.position.X - joint2.position.X) * (joint1.position.X - joint2.position.X)
                    + (joint1.position.Y - joint2.position.Y) * (joint1.position.Y - joint2.position.Y)
                    + (joint1.position.Z - joint2.position.Z) * (joint1.position.Z - joint2.position.Z));
}

void vabs(float* val) {
    if ((*val) < 0)
        (*val) *= -1;
}

float AbsVal(float val) {
    return ((val) > 0) ? (val) : (-val);
}

float memo = 1;

// this function is called each frame
void glutDisplay() {
    XnUserID aUserIDs[1] = { 0 };
    XnUInt16 nUsers = 1;
    //XnUserID aUserIDs[1]={0};
    //XnUInt16 nUsers = 1;
    float angoli[NUMBER_OF_ANGLES];
    memset(angoli, 0, sizeof(angoli));
    static int cycles = 0;

    static bool gripper_open = false;
    vector < Vec3f > circles;

    if (flag & GRIPPER != 0) {
        nRetVal = g_Context.WaitAnyUpdateAll();

        g_image.GetMapOutputMode(imageMapMode);

        const XnRGB24Pixel* pImageMap = g_image.GetRGB24ImageMap();

        convert_pixel_map_i(pImageMap, frame, imageMapMode.nYRes, imageMapMode.nXRes);

        // Load threshold from the slider bars in these 2 parameters
        Scalar hsv_min = Scalar(t1min, t2min, t3min);
        Scalar hsv_max = Scalar(t1max, t2max, t3max);

        // Covert color space to HSV as it is much easier to filter colors in the HSV color-space.
        cvtColor(frame, hsv_frame, CV_BGR2HSV);

        // Filter out colors which are out of range.
        inRange(hsv_frame, hsv_min, hsv_max, thresholded);
        Size size = Size(frame.cols, frame.rows);

        // the below lines of code is for visual purpose only remove after calibration
        //--------------FROM HERE-----------------------------------
        //Split image into its 3 one dimensional images
        Mat thresholded1(size, CV_8UC1); // Component image threshold
        Mat thresholded2(size, CV_8UC1);
        Mat thresholded3(size, CV_8UC1);

        Mat thresArr[] = { thresholded1, thresholded2, thresholded3 };

        split(hsv_frame, thresArr);
        // Filter out colors which are out of range.
        inRange(thresholded1, Scalar(t1min, 0, 0), Scalar(t1max, 0, 0), thresholded1);
        inRange(thresholded2, Scalar(t2min, 0, 0), Scalar(t2max, 0, 0), thresholded2);
        inRange(thresholded3, Scalar(t3min, 0, 0), Scalar(t3max, 0, 0), thresholded3);

        //-------------REMOVE OR COMMENT AFTER CALIBRATION TILL HERE ------------------

        // hough detector works better with some smoothing of the image
        //cvSmooth(thresholded, thresholded, CV_GAUSSIAN, 9, 9);2
        GaussianBlur(thresholded, thresholded, Size(9, 9), 0);

        //hough transform to detect circle
        HoughCircles(thresholded, circles, CV_HOUGH_GRADIENT, 2, frame.rows / 4, 100, 50, 0, 0);

        imshow("Camera", frame); // Original stream with detected ball overlay
//    imshow("HSV", hsv_frame); // Original stream in the HSV color space
        imshow("After Color Filtering", thresholded); // The stream after color filtering
        imshow("F1", thresholded1); // individual filters
        imshow("F2", thresholded2);
        imshow("F3", thresholded3);

        if(circles.size() > 0 && gripper_open){
            cycles=0;
        }
        if ((cvWaitKey(10) & 255) == 27) {

            *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 0, 0)) = t1min;
            *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 0, 1)) = t2min;
            *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 0, 2)) = t3min;
            *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 1, 0)) = t1max;
            *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 1, 1)) = t2max;
            *((float*) CV_MAT_ELEM_PTR(*threshold_matrix, 1, 2)) = t3max;
            return;
        }
    }
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Setup the OpenGL viewpoint
    glMatrixMode (GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    xn::SceneMetaData sceneMD;
    xn::DepthMetaData depthMD;
    g_DepthGenerator.GetMetaData(depthMD);
#ifndef USE_GLES
    glOrtho(0, depthMD.XRes(), depthMD.YRes(), 0, -1.0, 1.0);
#else
    glOrthof(0, depthMD.XRes(), depthMD.YRes(), 0, -1.0, 1.0);
#endif

    glDisable (GL_TEXTURE_2D);

    if (!g_bPause) {
        // Read next available data
        g_Context.WaitOneUpdateAll(g_UserGenerator);
    }

    // Process the data
    g_DepthGenerator.GetMetaData(depthMD);
    g_UserGenerator.GetUserPixels(0, sceneMD);
    DrawDepthMap(depthMD, sceneMD);
    g_UserGenerator.GetUsers(aUserIDs, nUsers);
    if (/*(g_UserGenerator.GetUsers(aUserIDs, nUsers) == XN_STATUS_OK) &&*/(skeletonTracked)) {
        XnSkeletonJointPosition mano, gomito, spalla;
        g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUserIDs[0], XN_SKEL_RIGHT_HAND, mano);
        g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUserIDs[0], XN_SKEL_RIGHT_ELBOW, gomito);
        g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUserIDs[0], XN_SKEL_RIGHT_SHOULDER, spalla);

        //printf("skeletonTracked=%d primavolta = %d\n", skeletonTracked, primavolta);
        if ((primavolta) && (skeletonTracked)) {
            printf("\nINITIALIZED!!\n");
            initialArmLength = distanza(mano, gomito);
            initialShoulderLength = distanza(spalla, gomito);
            primavolta = false;
        }
        if ((flag & CIN_INV) == 0) {
            CalcolaAngoli(mano, gomito, spalla, angoli);
        }
        else {
            CinematicaInversa(mano, spalla, angoli);
            for (int n = 0; n < NUMBER_OF_ANGLES; n++) {
                printf("angolo[%d]=%f\n", n, angoli[n]);

            }
            printf("\n");
        }

        float dArmLength, dShoulderLength;
        if (skeletonTracked) {
            dArmLength = (distanza(mano, gomito) - initialArmLength);
            dShoulderLength = (distanza(spalla, gomito) - initialShoulderLength);
            vabs(&dArmLength);
            vabs(&dShoulderLength);
            //printf("\narm=%f, sh=%f", dArmLength, dShoulderLength);
            if ((flag & GRIPPER) != 0) {

                cycles++;
                if (cycles >= CYCLES_GRIPPER) {
                    //printf("\ndist = %f, init=%f\n", dArmLength, initialArmLength);
                    if ((circles.size() > 0) && (!gripper_open)) {

                        gripper_open = true;
                        cycles = 0;
                        printf("mano aperta\n");

                    }
                    else {
                        if ((circles.size() == 0) && (gripper_open)) {
                            gripper_open = false;
                            cycles = 0;
                            printf("mano chiusa\n");
                        }
                    }
                }

                if (gripper_open) {
                    angoli[5] = 1.;
                }
                else {
                    angoli[5] = 0.;
                }
            }

#if 0
            if (cc <= 40)
            angoli[3] = memo;

            if (cc > 40) {
                angoli[3] = xx * 7.5 + 1.2;
                memo = angoli[3];
                cc = 0;
            }
#endif
            /*
             printf("\n");
             for (int i = 0; i < 4; i++) {
             printf("angolo[%d]=%f", i, angoli[i]);
             }
             printf("\n%d", sizeof(angoli));*/
        }
    }
    if (!primavolta) {
        write(idsock, angoli, sizeof(angoli));
    }

    //cc++;
#ifndef USE_GLES
    glutSwapBuffers();
#endif
}

#ifndef USE_GLES
void glutIdle(void) {
    if (g_bQuit) {
        CleanupExit();
    }

    // Display the frame
    glutPostRedisplay();
}

void glutKeyboard(unsigned char key,
                  int /*x*/,
                  int /*y*/) {
    switch (key) {
    case 27:
        CleanupExit();
    case 'b':
        // Draw background?
        g_bDrawBackground = !g_bDrawBackground;
        break;
    case 'x':
        // Draw pixels at all?
        g_bDrawPixels = !g_bDrawPixels;
        break;
    case 's':
        // Draw Skeleton?a
        g_bDrawSkeleton = !g_bDrawSkeleton;
        break;
    case 'i':
        // Print label?
        g_bPrintID = !g_bPrintID;
        break;
    case 'l':
        // Print ID & state as label, or only ID?
        g_bPrintState = !g_bPrintState;
        break;
    case 'f':
        // Print FrameID
        g_bPrintFrameID = !g_bPrintFrameID;
        break;
    case 'j':
        // Mark joints
        g_bMarkJoints = !g_bMarkJoints;
        break;
    case 'p':
        g_bPause = !g_bPause;
        break;
    case 'S':
        SaveCalibration();
        break;
    case 'L':
        LoadCalibration();
        break;
    }
}
void glInit(int * pargc,
            char ** argv) {
    glutInit(pargc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
    glutCreateWindow("User Tracker Viewer");
    //glutFullScreen();
    glutSetCursor (GLUT_CURSOR_NONE);

    glutKeyboardFunc(glutKeyboard);
    glutDisplayFunc(glutDisplay);
    glutIdleFunc(glutIdle);

    glDisable (GL_DEPTH_TEST);
    glEnable (GL_TEXTURE_2D);

    glEnableClientState (GL_VERTEX_ARRAY);
    glDisableClientState (GL_COLOR_ARRAY);
}
#endif // USE_GLES

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));\
		return nRetVal;												\
	}

int connetti() {
    struct sockaddr_in client;
    int lunghezza = sizeof(client);
    idsock = socket(AF_INET, SOCK_STREAM, 0);
    client.sin_family = AF_INET;
    client.sin_port = htons(PORT);
    inet_aton(IP, &(client.sin_addr));
    if (connect(idsock, (struct sockaddr*) &client, sizeof(client)) == -1)
        return -1;
    else
        return idsock;
}

int main(int argc,
         char **argv) {
    nRetVal = XN_STATUS_OK;

    if ((idsock = connetti()) == -1)
        cout << "\n" << "ERRORE DI CONNESSIONE AL ROBOT\n";

    for (int n = 1; n < argc; n++) {
        if (strcmp(argv[n], "gripper") == 0) {
            flag |= GRIPPER;
        }
        else if (strcmp(argv[n], "cin_inv") == 0) {
            flag |= CIN_INV;
        }
    }

    //else
    //{

    xn::EnumerationErrors errors;
    nRetVal = g_Context.InitFromXmlFile(SAMPLE_XML_PATH, g_scriptNode, &errors);
    if (nRetVal == XN_STATUS_NO_NODE_PRESENT) {
        XnChar strError[1024];
        errors.ToString(strError, 1024);
        printf("%s\n", strError);
        return (nRetVal);
    }
    else if (nRetVal != XN_STATUS_OK) {
        printf("Open failed: %s\n", xnGetStatusString(nRetVal));
        return (nRetVal);
    }
    //}

    if (flag & GRIPPER != 0) {
        cvNamedWindow("Camera", CV_WINDOW_AUTOSIZE);
        //  cvNamedWindow("HSV", CV_WINDOW_AUTOSIZE);
        cvNamedWindow("After Color Filtering", CV_WINDOW_AUTOSIZE);
        cvNamedWindow("F1", CV_WINDOW_AUTOSIZE);
        cvNamedWindow("F2", CV_WINDOW_AUTOSIZE);
        cvNamedWindow("F3", CV_WINDOW_AUTOSIZE);
    }

    threshold_matrix = cvCreateMat(2, 3, CV_32FC1);

    // Load the previous values of the threshold if they exist
    if (cvOpenFileStorage(STORAGE_FILE, NULL, CV_STORAGE_READ)) {
        threshold_matrix = (CvMat*) cvLoad(STORAGE_FILE);
//#if 0
        t1min=(int) CV_MAT_ELEM(*threshold_matrix,float,0,0);
        t2min=(int) CV_MAT_ELEM(*threshold_matrix,float,0,1);
        t3min=(int) CV_MAT_ELEM(*threshold_matrix,float,0,2);
        t1max=(int) CV_MAT_ELEM(*threshold_matrix,float,1,0);
        t2max=(int) CV_MAT_ELEM(*threshold_matrix,float,1,1);
        t3max=(int) CV_MAT_ELEM(*threshold_matrix,float,1,2);
//#endif
    }

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

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_image);
    if (nRetVal != XN_STATUS_OK) {
        printf("No image node exists! Check your XML.");
        return -1;
    }

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
    if (nRetVal != XN_STATUS_OK) {
        printf("No depth generator found. Using a default one...");
        xn::MockDepthGenerator mockDepth;
        nRetVal = mockDepth.Create(g_Context);
        CHECK_RC(nRetVal, "Create mock depth");

        // set some defaults
        XnMapOutputMode defaultMode;
        defaultMode.nXRes = 320;
        defaultMode.nYRes = 240;
        defaultMode.nFPS = 30;
        nRetVal = mockDepth.SetMapOutputMode(defaultMode);
        CHECK_RC(nRetVal, "set default mode");

        // set FOV
        XnFieldOfView fov;
        fov.fHFOV = 1.0225999419141749;
        fov.fVFOV = 0.79661567681716894;
        nRetVal = mockDepth.SetGeneralProperty(XN_PROP_FIELD_OF_VIEW, sizeof(fov), &fov);
        CHECK_RC(nRetVal, "set FOV");

        XnUInt32 nDataSize = defaultMode.nXRes * defaultMode.nYRes * sizeof(XnDepthPixel);
        XnDepthPixel* pData = (XnDepthPixel*) xnOSCallocAligned(nDataSize, 1, XN_DEFAULT_MEM_ALIGN);

        nRetVal = mockDepth.SetData(1, 0, nDataSize, pData);
        CHECK_RC(nRetVal, "set empty depth map");

        g_DepthGenerator = mockDepth;
    }

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
    if (nRetVal != XN_STATUS_OK) {
        nRetVal = g_UserGenerator.Create(g_Context);
        CHECK_RC(nRetVal, "Find user generator");
    }

    XnCallbackHandle hUserCallbacks, hCalibrationStart, hCalibrationComplete, hPoseDetected, hCalibrationInProgress, hPoseInProgress;
    if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
        printf("Supplied user generator doesn't support skeleton\n");
        return 1;
    }
    nRetVal = g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
    CHECK_RC(nRetVal, "Register to user callbacks");
    nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationStart(UserCalibration_CalibrationStart, NULL, hCalibrationStart);
    CHECK_RC(nRetVal, "Register to calibration start");
    nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationComplete(UserCalibration_CalibrationComplete, NULL, hCalibrationComplete);
    CHECK_RC(nRetVal, "Register to calibration complete");

    if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
        g_bNeedPose = TRUE;
        if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
            printf("Pose required, but not supported\n");
            return 1;
        }
        nRetVal = g_UserGenerator.GetPoseDetectionCap().RegisterToPoseDetected(UserPose_PoseDetected, NULL, hPoseDetected);
        CHECK_RC(nRetVal, "Register to Pose Detected");
        g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);

        nRetVal = g_UserGenerator.GetPoseDetectionCap().RegisterToPoseInProgress(MyPoseInProgress, NULL, hPoseInProgress);
        CHECK_RC(nRetVal, "Register to pose in progress");
    }

    g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

    nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationInProgress(MyCalibrationInProgress, NULL, hCalibrationInProgress);
    CHECK_RC(nRetVal, "Register to calibration in progress");

    nRetVal = g_Context.StartGeneratingAll();
    CHECK_RC(nRetVal, "StartGenerating");

#ifndef USE_GLES
    glInit(&argc, argv);
    glutMainLoop();
#else
    if (!opengles_init(GL_WIN_SIZE_X, GL_WIN_SIZE_Y, &display, &surface, &context))
    {
        printf("Error initializing opengles\n");
        CleanupExit();
    }

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);

    while (!g_bQuit)
    {
        glutDisplay(idsock);
        eglSwapBuffers(display, surface);

    }
    if(idsock!=-1)
    close(idsock);
    opengles_shutdown(display, surface, context);

    CleanupExit();
#endif

}
