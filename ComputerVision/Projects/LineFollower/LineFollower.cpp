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

#define SHOW_IMAGES
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
    int threshold_value = 100;


    int usbFD=USBConfig();

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
#ifdef SHOW_IMAGES

    cvNamedWindow("Camera", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("HSV", CV_WINDOW_AUTOSIZE);
    namedWindow("lineBandGreyThres", 1);



// Create trackbars to threshold the line for the line follower
    char* trackbar_value = "Value";
    int const max_value = 255;
    int threshold_type = 1;

    createTrackbar(trackbar_value, "lineBandGreyThres", &threshold_value, max_value, NULL);
#endif
// get the image data
    height = frame->height;
    width = frame->width;
    step = frame->widthStep;

// capture size -
    CvSize size = cvSize(width, height);

// Initialize different images that are going to be used in the program
    IplImage* hsv_frame = cvCreateImage(size, IPL_DEPTH_8U, 3); // image converted to HSV plane
    IplImage* thresholded = cvCreateImage(size, IPL_DEPTH_8U, 1); // final thresholded image
    IplImage* filtered = cvCreateImage(size, IPL_DEPTH_8U, 1);  //smoothed image

    //calibrate the motor
    if(usbFD>0){
        short int controls = (15<<8);
        controls|=12;
        if(write(usbFD, &controls, sizeof(controls))<0){
            printf("\nUSB write ERROR!\n");
        }
    }
    sleep(3);

//program loop
    while (1) {

        //WIP

        // Get one frame
        frame = cvQueryFrame(capture);


        // Covert color space to HSV as it is much easier to filter colors in the HSV color-space.
        cvCvtColor(frame, hsv_frame, CV_BGR2HSV);

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
            ColorRange *signal=GetStreetSignalRanges(signalAction[i],signalColorRange);
            if(signal!=NULL){
                // filter the hsv image
                // detect the circle
                // depending on the signal detected do something
                speedControl=DetectSignal(hsv_frame, thresholded, *signal, status);
                //assume one signal per time
                break;
            }
            i++;
        }

//        status=FOLLOW_RIGHT;

        //cvCanny(thresholded, thresholded, 0,255);
        cv::Mat frameMat = cv::cvarrToMat(frame);

        //create the image band
        Rect rect(LINE_BAND_X(frameMat.rows,frameMat.cols), LINE_BAND_Y(frameMat.rows,frameMat.cols), LINE_BAND_WIDTH(frameMat.rows,frameMat.cols), LINE_BAND_HEIGHT(frameMat.rows,frameMat.cols));
        cv::Mat lineBand(frameMat, rect);

        //create images for gryscale and thresholded grayscale
        Mat lineBandGrey;
        Mat lineBandGreyThres;

        //convert RGB to grayscale
        cvtColor(lineBand, lineBandGrey, CV_BGR2GRAY);

        //threshold the grayscale to detect the line
        threshold(lineBandGrey, lineBandGreyThres, threshold_value, 255, 1);


        dirControl=FollowLine(lineBandGreyThres, frame->width + 1, 0, LINE_BAND_Y(frameMat.rows,frameMat.cols), 0, status);

        short int controls=(speedControl<<8);
        controls|=dirControl;

        printf("controls %d %d\n",  speedControl,dirControl);
        if(write(usbFD, &controls, sizeof(controls))<0){
            printf("\nUSB write ERROR!\n");
        }
#ifdef SHOW_IMAGES
        cvShowImage("Camera", frame); // Original stream with detected ball overlay
        cvShowImage("HSV", hsv_frame); // Original stream in the HSV color space
        imshow("lineBandGreyThres", lineBandGreyThres);
#endif
        //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
        //remove higher bits using AND operator
        if ((cvWaitKey(10) & 255) == 27)
            break;
    }
#ifdef SHOW_IMAGES

#endif
// Release the capture device housekeeping
    cvReleaseCapture(&capture);

    return 0;
}


