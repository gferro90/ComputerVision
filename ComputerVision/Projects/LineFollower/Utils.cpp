/**
 * @file Utils.cpp
 * @brief Source file for class Utils
 * @date 23/feb/2017
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
 * the class Utils (public, protected, and private). Be aware that some 
 * methods, such as those inline could be defined on the header file, instead.
 */

/*---------------------------------------------------------------------------*/
/*                         Standard header includes                          */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                         Project header includes                           */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                           Static definitions                              */
/*---------------------------------------------------------------------------*/
#include "Utils.h"
#include "PIDController.h"

static bool InRange(float input,
                    float min,
                    float max) {
    return (input >= min) && (input <= max);
}

/*---------------------------------------------------------------------------*/
/*                           Method definitions                              */
/*---------------------------------------------------------------------------*/

ColorRange *GetStreetSignalRanges(const char* input,
                                  ColorRange *signalColorRange) {

    int i = 0;
    while (signalColorRange[i].signalName != NULL) {
        if (strcmp(signalColorRange[i].signalName, input) == 0) {
            return &signalColorRange[i];
        }
        i++;
    }
    return NULL;
}

int USBConfig() {
    int usbFD = -1;
    int k = 0;
    while (usbFD < 0) {
        usbFD = open(USB_PORT, O_RDWR | O_NOCTTY | O_SYNC);
        if ((k++) > 100) {
            printf("USB:: Failed open %s\n", USB_PORT);
            return -1;
        }
        usleep(100000);
    }

    int speed = USB_BAUDRATE;

    int parity = 0;

    struct termios ttyUSB;
    memset(&ttyUSB, 0, sizeof ttyUSB);
    if (tcgetattr(usbFD, &ttyUSB) != 0) {
        close(usbFD);
        return -1;
    }

    cfsetospeed(&ttyUSB, speed);
    cfsetispeed(&ttyUSB, speed);

    ttyUSB.c_cflag = (ttyUSB.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    ttyUSB.c_iflag &= ~IGNBRK;         // disable break processing
    ttyUSB.c_lflag = 0;                // no signaling chars, no echo,
                                       // no canonical processing
    ttyUSB.c_oflag = 0;                // no remapping, no delays
    ttyUSB.c_cc[VMIN] = 1;            // read  block
    ttyUSB.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
    ttyUSB.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    ttyUSB.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
                                        // enable reading
    ttyUSB.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    ttyUSB.c_cflag |= parity;
    ttyUSB.c_cflag &= ~CSTOPB;
    ttyUSB.c_cflag &= ~CRTSCTS;

    if (tcsetattr(usbFD, TCSANOW, &ttyUSB) != 0) {
        close(usbFD);
        return -1;
    }

    tcflush(usbFD, TCIOFLUSH);
    return usbFD;
//       write(usbFD, inputsFromHttp, (2 * sizeof(int)));

}

int PwmRemapping(float control,
                 float minIn,
                 float maxIn,
                 int minOut,
                 int maxOut) {
    float rangeIn = maxIn - minIn;
    int rangeOut = maxOut - minOut;
    return (int) (((control - minIn) * rangeOut) / rangeIn) + minOut;
}

int DetectSignal(IplImage *hsv_frame,
                 IplImage *thresholded,
                 ColorRange &colorRange,
                 stati& status) {

// Load threshold from the slider bars in these 2 parameters
    CvScalar hsv_min = colorRange.range1;
    CvScalar hsv_max = colorRange.range2;

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

    float speedControl = 0.;
    for (int i = 0; i < circles->total; i++) {   //get the parameters of circles detected
        float* p = (float*) cvGetSeqElem(circles, i);
        //printf("Ball! x=%f y=%f r=%f\n\r",p[0],p[1],p[2] );

        //radius condition on the ball
        if (p[2] <= SIGNAL_MAX_RADIUS && p[2] >= SIGNAL_MIN_RADIUS) {

            // draw a circle with the centre and the radius obtained from the hough transform
            cvCircle(hsv_frame, cvPoint(cvRound(p[0]), cvRound(p[1])),  //plot centre
                     2, CV_RGB(255, 255, 255), -1, 8, 0);
            cvCircle(hsv_frame, cvPoint(cvRound(p[0]), cvRound(p[1])),  //plot circle
                     cvRound(p[2]), CV_RGB(0, 255, 0), 2, 8, 0);

            // execute the signal action changing the status
            speedControl = colorRange.SignalAction((int&) status);
            printf("\ndetected signal for %s\n", colorRange.signalName);
            break;
        }
    }

    /* for testing purpose you can show all the images but when done with calibration
     only show frame to keep the screen clean  */

    cvReleaseMemStorage(&storage);
    return PwmRemapping(speedControl, SPEED_CONTROL_MIN, SPEED_CONTROL_MAX, SPEED_PWM_MIN, SPEED_PWM_MAX);
}

int FollowLine(Mat &lineBandGreyThres,
               double initialMinX,
               double initialMaxX,
               double initialMinY,
               double initialMaxY,
               stati status) {
//get the contours
    int dirControl = 0;

    vector < vector<Point> > contours;
    vector < Vec4i > hierarchy;
    RNG rng(12345);
    Mat temp = lineBandGreyThres.clone();
    findContours(temp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    double maxX = initialMaxX;
    double minX = initialMinX;
//search the minimum and maximum x points on all contours

    for (int i = 0; i < contours.size(); i++) {
        if (InRange(contours[i].size(), MIN_CONTOURS_SIZE, MAX_CONTOURS_SIZE)) {
            printf("\nContours size%d\n", contours[i].size());
            double minX_temp = initialMinX;
            double maxX_temp = initialMaxX;
            double minY_temp = initialMinY;
            double maxY_temp = initialMaxY;

            for (int j = 0; j < contours[i].size(); j++) {

                if ((contours[i])[j].x < minX_temp) {
                    minX_temp = (contours[i])[j].x;
                }
                if ((contours[i])[j].x > maxX_temp) {
                    maxX_temp = (contours[i])[j].x;
                }
                if ((contours[i])[j].y < minY_temp) {
                    minY_temp = (contours[i])[j].y;
                }
                if ((contours[i])[j].y > maxY_temp) {
                    maxY_temp = (contours[i])[j].y;
                }
            }
            //width and height of the contours are compliants with the line
            if (InRange(maxX_temp - minX_temp, MIN_WIDTH_LINE(lineBandGreyThres.rows, lineBandGreyThres.cols),
                        MAX_WIDTH_LINE(lineBandGreyThres.rows, lineBandGreyThres.cols))) {
                if (InRange(maxY_temp - minY_temp, MIN_HEIGHT_LINE(lineBandGreyThres.rows, lineBandGreyThres.cols),
                            MAX_HEIGHT_LINE(lineBandGreyThres.rows+1, lineBandGreyThres.cols))) {
                    printf("\nLINE DETECTED w=%f h=%f!!!\n", maxX_temp - minX_temp, maxY_temp - minY_temp);
                    if (minX_temp < minX) {
                        minX = minX_temp;
                    }
                    if (maxX_temp > maxX) {
                        maxX = maxX_temp;
                    }
                }
            }
        }
    }
    //draw contours on the band
    for (int i = 0; i < contours.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        drawContours(lineBandGreyThres, contours, i, color, 2, 8, hierarchy, 0, Point());
    }

    // something has been detected
    float control = 0.;
    PIDController pid(KP);

    if (minX <= maxX) {
        switch (status) {
        case (FOLLOW_LEFT): {
            //assume left line detected
            //if status is follow left
            float error = X_REF_LEFT(lineBandGreyThres.rows,lineBandGreyThres.cols) - minX;

            //get the drive control on this error
            control = pid.Execute(error);
        }
            break;
        case (FOLLOW_RIGHT): {
            //assume right line detected
            //if status is follow right
            float error = X_REF_RIGHT(lineBandGreyThres.rows,lineBandGreyThres.cols) - maxX;
            printf("\nerror=%f\n", error);
            //get the drive control on this error
            control = pid.Execute(error);
        }
            break;
        default: {
            control = 0;                        //??
        }
        }
    }

    printf("Min=%f Max=%f\n", minX, maxX);
    return PwmRemapping(control, DRIVE_CONTROL_MIN, DRIVE_CONTROL_MAX, DRIVE_PWM_MIN, DRIVE_PWM_MAX);
}
