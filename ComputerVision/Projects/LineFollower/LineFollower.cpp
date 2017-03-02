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

static stati status = FOLLOW_RIGHT;
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
    int threshold_value = LINE_THRES;

    int usbFD = USBConfig();

    printf("\nPress any key\n");
    char anyKey;
    scanf("%c", &anyKey);

// Open capture device. 0 is /dev/video0, 1 is /dev/video1, etc.
    VideoCapture capture(CAMERA);
    //CvCapture* capture = cvCaptureFromCAM(CAMERA);
    //capture->set(CV_CAP_PROP_FPS, 60);
    /*
     if (!capture) {
     fprintf(stderr, "ERROR: capture is NULL \n");
     getchar();
     return -1;
     }
     */
    // Create a window in which the captured images will be presented
#ifdef SHOW_IMAGES

    namedWindow("Camera", WINDOW_AUTOSIZE);
    namedWindow("HSV", WINDOW_AUTOSIZE);
    namedWindow("lineBandGreyThres", WINDOW_AUTOSIZE);

    // Create trackbars to threshold the line for the line follower
    char* trackbar_value = "Value";
    int const max_value = 255;
    int threshold_type = 1;

    createTrackbar(trackbar_value, "lineBandGreyThres", &threshold_value, max_value, NULL);
#endif

// grab an image from the capture

    Mat frameMat;
    for (int i = 0; i < 10; i++) {
        capture >> frameMat;
    }

    if (frameMat.empty()) {
        printf("ERROR: capture is NULL \n");
        return -1;
    }
    //IplImage* frame = cvQueryFrame(capture);
// capture size -

// Initialize different images that are going to be used in the program
    /*   IplImage* hsv_frame = cvCreateImage(size, IPL_DEPTH_8U, 3); // image converted to HSV plane
     IplImage* thresholded = cvCreateImage(size, IPL_DEPTH_8U, 1); // final thresholded image
     */

    //cvCanny(thresholded, thresholded, 0,255);
    //create the image band
    Rect rect(LINE_BAND_X(frameMat.rows, frameMat.cols), LINE_BAND_Y(frameMat.rows, frameMat.cols), LINE_BAND_WIDTH(frameMat.rows, frameMat.cols),
              LINE_BAND_HEIGHT(frameMat.rows, frameMat.cols));
    cv::Mat lineBand(frameMat, rect);

    Mat hsv_frame;
    Mat thresholded;

    cvtColor(lineBand, hsv_frame, CV_BGR2HSV);

    //create images for gryscale and thresholded grayscale
    Mat lineBandGrey;
    Mat lineBandGreyThres;

    // Covert color space to HSV as it is much easier to filter colors in the HSV color-space.
    cvtColor(lineBand, hsv_frame, CV_BGR2HSV);

    //convert RGB to grayscale
    cvtColor(lineBand, lineBandGrey, CV_BGR2GRAY);

    //threshold the grayscale to detect the line
    threshold(lineBandGrey, lineBandGreyThres, threshold_value, 255, LINE_THRES_TYPE);

    //calibrate 
    int refLeft = 0;
    int refRight = 0;
    int lineWidth = 0;
    int lineHeight = 0;
#ifdef CALIBRATE
    FollowLine(lineBandGreyThres, lineBandGreyThres.cols + 1, 0, LINE_BAND_Y(frameMat.rows, frameMat.cols), 0, status, refLeft, refRight, lineWidth, lineHeight,
               true);
    printf("Calibration Results: \nrefLeft=%d \nrefRight=%d \nlineWidth=%d \nlineHeight=%d", refLeft, refRight, lineWidth, lineHeight);
#endif

    if (usbFD > 0) {
        int speedControl = PwmRemapping(SPEED_ZERO_CONTROL, SPEED_CONTROL_MIN, SPEED_CONTROL_MAX, SPEED_PWM_MIN, SPEED_PWM_MAX);
        short int controls = (speedControl << 8);
        int driveControl = PwmRemapping(DRIVE_ZERO_CONTROL, DRIVE_CONTROL_MIN, DRIVE_CONTROL_MAX, DRIVE_PWM_MIN, DRIVE_PWM_MAX);
        controls |= driveControl;
        if (write(usbFD, &controls, sizeof(controls)) < 0) {
            printf("\nUSB write ERROR!\n");
        }
    }
    sleep(10);

//program loop
    while (1) {

        //WIP

        int speedControl = PwmRemapping(SPEED_STANDARD_CONTROL, SPEED_CONTROL_MIN, SPEED_CONTROL_MAX, SPEED_PWM_MIN, SPEED_PWM_MAX);

        int driveControl = PwmRemapping(DRIVE_ZERO_CONTROL, DRIVE_CONTROL_MIN, DRIVE_CONTROL_MAX, DRIVE_PWM_MIN, DRIVE_PWM_MAX);

        // Get one frame
        capture >> frameMat;

        // Covert color space to HSV as it is much easier to filter colors in the HSV color-space.
        cvtColor(lineBand, hsv_frame, CV_BGR2HSV);

        /*        if (!frame) {
         fprintf(stderr, "ERROR: frame is null...\n");
         getchar();
         break;
         }
         */
#ifdef DOPO
        int i = 0;
        while (signalAction[i] != NULL) {
            // query the color range
            ColorRange *signal = GetStreetSignalRanges(signalAction[i], signalColorRange);
            if (signal != NULL) {
                // filter the hsv image
                // detect the circle
                // depending on the signal detected do something
                speedControl = DetectSignal(hsv_frame, thresholded, *signal, status);
                //assume one signal per time
                break;
            }
            i++;
        }
#endif
        //convert RGB to grayscale
        cvtColor(lineBand, lineBandGrey, CV_BGR2GRAY);

        //threshold the grayscale to detect the line
        threshold(lineBandGrey, lineBandGreyThres, threshold_value, 255, LINE_THRES_TYPE);

#ifndef CALIBRATE
        refLeft=X_REF_LEFT(lineBandGreyThres.rows, lineBandGreyThres.cols);
        refRight=X_REF_RIGHT(lineBandGreyThres.rows,lineBandGreyThres.cols);
        lineWidth=WIDTH_LINE;
        lineHeight=HEIGHT_LINE;
#endif

        driveControl = FollowLine(lineBandGreyThres, lineBandGreyThres.cols + 1, 0, LINE_BAND_Y(frameMat.rows, frameMat.cols), 0, status, refLeft, refRight,
                                  lineWidth, lineHeight, false);

        short int controls = (speedControl << 8);
        controls |= driveControl;

        printf("controls %d %d\n", speedControl, driveControl);
        if (write(usbFD, &controls, sizeof(controls)) < 0) {
            printf("\nUSB write ERROR!\n");
        }
#ifdef SHOW_IMAGES
        imshow("Camera", frameMat); // Original stream with detected ball overlay
        imshow("HSV", hsv_frame); // Original stream in the HSV color space
        imshow("lineBandGreyThres", lineBandGreyThres);
#endif
        //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
        //remove higher bits using AND operator

        if ((cvWaitKey(5) & 255) == 27) {
            speedControl = PwmRemapping(SPEED_ZERO_CONTROL, SPEED_CONTROL_MIN, SPEED_CONTROL_MAX, SPEED_PWM_MIN, SPEED_PWM_MAX);
            controls = (speedControl << 8);
            driveControl = PwmRemapping(DRIVE_ZERO_CONTROL, DRIVE_CONTROL_MIN, DRIVE_CONTROL_MAX, DRIVE_PWM_MIN, DRIVE_PWM_MAX);
            controls |= driveControl;
            if (write(usbFD, &controls, sizeof(controls)) < 0) {
                printf("\nUSB write ERROR!\n");
            }
            break;
        }

    }
#ifdef SHOW_IMAGES

#endif
// Release the capture device housekeeping

    return 0;
}

