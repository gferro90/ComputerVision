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

#define STOP 0
#define ZERO_LEFT 1
#define ZERO_RIGHT 2
#define LEFT_LINE 3
#define RIGHT_LINE 4
#define TWO_LINES 5

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
    if (control > maxIn) {
        control = maxIn;
    }
    if (control < minIn) {
        control = minIn;
    }

    float rangeIn = maxIn - minIn;
    int rangeOut = maxOut - minOut;
    return (int) (((control - minIn) * rangeOut) / rangeIn) + minOut;
}

int DetectSignal(Mat &hsv_frame,
                 Mat &thresholded,
                 ColorRange &colorRange,
                 stati& status) {

    float speedControl = SPEED_STANDARD_CONTROL;

// Load threshold from the slider bars in these 2 parameters
    Scalar hsv_min = colorRange.range1;
    Scalar hsv_max = colorRange.range2;

    // Filter out colors which are out of range.
    inRange(hsv_frame, hsv_min, hsv_max, thresholded);
    /*
     vector < vector<Point> > contours;
     vector < Vec4i > hierarchy;
     */
    int middleY = (int) (thresholded.rows / 2.);
    int consecutiveWhiteCnt = 0;
    int maxConsecutiveWhite = 0;
    //sarch white points in the middle line
    for (unsigned int i = 0; i < thresholded.cols; i++) {

        if (thresholded.at < uint8_t > (middleY, i) == 255) {
            consecutiveWhiteCnt++;
        }
        else {
            if (consecutiveWhiteCnt > maxConsecutiveWhite) {
                maxConsecutiveWhite = consecutiveWhiteCnt;
            }
            consecutiveWhiteCnt = 0;
        }
    }
    if (maxConsecutiveWhite > SIGNAL_MIN_WIDTH) {
        speedControl = colorRange.SignalAction((int&) status);
        printf("\ndetected signal for %s\n", colorRange.signalName);
    }

    return PWM_SPEED_REMAP(speedControl);
}

static void StateMachine(int minX,
                         int maxX,
                         int &minX_used,
                         int &maxX_used,
                         int minX_1,
                         int maxX_1,
                         float curvature,
                         int linesCounter,
                         int imgMin,
                         int imgMax,
                         int initialMinX,
                         int initialMaxX,
                         int &speedControl) {
    static int lineStatus = TWO_LINES;
    static int numberCyclesInStatus = 0;

    // printf("\nstatus is %d", lineStatus);
    switch (lineStatus) {

    case (TWO_LINES): {
        if (linesCounter >= 2) {
            maxX_used = maxX;
            minX_used = minX;
            numberCyclesInStatus++;
        }
        else if (linesCounter == 1) {
            bool turnRight = true;
           // printf("\ncurvature = %f!!!\n", curvature);

            if (ABS_VAL(curvature) > CURVATURE_THRES) {
                turnRight = (curvature < 0.);
             /*   if (turnRight) {
                    printf("\nTURN RIGHT!!!\n");
                }
                else {
                    printf("\nTURN LEFT!!!\n");
                }*/
            }
            else {
                //work on derivate
                int gapLeft = ABS_VAL(minX - minX_1);
                int gapRight = ABS_VAL(maxX - maxX_1);
                turnRight = (gapLeft < gapRight);
            }
            if (turnRight) {
                //printf("\nOnly Left Line!! %d %d %d %d %d %d\n", minX, maxX, minX_1, maxX_1, gapRight, gapLeft);

                minX_used = minX;
                //assume the line as left line
                maxX_used = imgMax;
                lineStatus = LEFT_LINE;
            }
            else {
                //printf("\nOnly Right Line!! %d %d %d %d %d %d\n", minX, maxX, minX_1, maxX_1, gapRight, gapLeft);

                //assume the line as right line
                minX_used = imgMin;
                maxX_used = maxX;
                lineStatus = RIGHT_LINE;
            }
            numberCyclesInStatus = 0;
        }
        else if (linesCounter == 0) {
            numberCyclesInStatus++;
            if (numberCyclesInStatus >= ZERO_LINE_CYCLES) {
                lineStatus = STOP;
                numberCyclesInStatus = 0;
            }
        }
    }
        break;

    case (LEFT_LINE): {
        if (linesCounter >= 2) {
            maxX_used = maxX;
            minX_used = minX;
            lineStatus = TWO_LINES;
            numberCyclesInStatus = 0;
        }
        else if (linesCounter == 1) {
            minX_used = minX;
            //assume the line as left line
            maxX_used = imgMax;
            numberCyclesInStatus++;
        }
        else if (linesCounter == 0) {
            minX_used = imgMax;
            maxX_used = imgMax;
            lineStatus = ZERO_LEFT;
            numberCyclesInStatus = 0;
        }
    }
        break;

    case (RIGHT_LINE): {
        if (linesCounter >= 2) {
            maxX_used = maxX;
            minX_used = minX;
            lineStatus = TWO_LINES;
            numberCyclesInStatus = 0;
        }
        else if (linesCounter == 1) {
            //assume the line as right line
            minX_used = imgMin;
            maxX_used = maxX;
            numberCyclesInStatus++;
        }
        else if (linesCounter == 0) {
            minX_used = imgMin;
            maxX_used = imgMin;
            lineStatus = ZERO_RIGHT;
            numberCyclesInStatus = 0;
        }
    }
        break;

    case (ZERO_LEFT): {
        if (linesCounter >= 2) {
            maxX_used = maxX;
            minX_used = minX;
            lineStatus = TWO_LINES;
            numberCyclesInStatus = 0;
        }
        else if (linesCounter == 1) {
            minX_used = minX;
            //assume the line as left line
            maxX_used = imgMax;
            lineStatus = LEFT_LINE;
            numberCyclesInStatus = 0;
        }
        else if (linesCounter == 0) {
            minX_used = imgMax;
            maxX_used = imgMax;
            numberCyclesInStatus++;
            if (numberCyclesInStatus >= ZERO_LINE_CYCLES) {
                lineStatus = STOP;
                numberCyclesInStatus = 0;
            }
        }
    }
        break;

    case (ZERO_RIGHT): {
        if (linesCounter >= 2) {
            maxX_used = maxX;
            minX_used = minX;
            lineStatus = TWO_LINES;
            numberCyclesInStatus = 0;
        }
        else if (linesCounter == 1) {
            //assume the line as right line
            minX_used = imgMin;
            maxX_used = maxX;
            lineStatus = RIGHT_LINE;
            numberCyclesInStatus = 0;
        }
        else if (linesCounter == 0) {
            minX_used = imgMin;
            maxX_used = imgMin;
            numberCyclesInStatus++;
            if (numberCyclesInStatus >= ZERO_LINE_CYCLES) {
                lineStatus = STOP;
                numberCyclesInStatus = 0;
            }
        }
    }
        break;
    case (STOP): {
        if (linesCounter >= 2) {
            maxX_used = maxX;
            minX_used = minX;
            lineStatus = TWO_LINES;
            numberCyclesInStatus = 0;
        }
        else {
            minX_used = initialMinX;
            maxX_used = initialMaxX;
            numberCyclesInStatus++;
            speedControl = PWM_SPEED_REMAP(SPEED_ZERO_CONTROL);
        }
    }

    }

}

int FollowLine(Mat &lineBandGreyThres,
               int initialMinX,
               int initialMaxX,
               int initialMinY,
               int initialMaxY,
               stati status,
               int &refLeft,
               int &refRight,
               int &width,
               int &height,
               int &speedControl,
               bool calibrate) {

    //store the old value for tracking
    static int minX_1 = X_REF_LEFT(lineBandGreyThres.rows, lineBandGreyThres.cols);
    static int maxX_1 = X_REF_RIGHT(lineBandGreyThres.rows, lineBandGreyThres.cols);

//get the contours

    vector < vector<Point> > contours;
    vector < Vec4i > hierarchy;

#ifdef SHOW_IMAGES
    RNG rng(12345);
    Mat temp = lineBandGreyThres.clone();
    findContours(temp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
#else
    findContours(lineBandGreyThres, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
#endif

    int maxX = initialMaxX;
    int minX = initialMinX;
    int maxX_used = initialMaxX;
    int minX_used = initialMinX;

    float curvature = 0.;
    //search the minimum and maximum x points on all contours

    int linesCounter = 0;

    for (int i = 0; i < contours.size() && (linesCounter < 2); i++) {
        if (InRange(contours[i].size(), MIN_CONTOURS_SIZE, MAX_CONTOURS_SIZE)) {
            int minX_temp = initialMinX;
            int maxX_temp = initialMaxX;
            int minY_temp = initialMinY;
            int maxY_temp = initialMaxY;
            int yinMaxX = initialMaxY;
            int yinMinX = initialMinY;

            for (int j = 0; j < contours[i].size(); j++) {
                //take max and min on the middle line

                if ((contours[i])[j].x < minX_temp) {
                    minX_temp = (contours[i])[j].x;
                    yinMinX = (contours[i])[j].y;
                }
                if ((contours[i])[j].x > maxX_temp) {
                    maxX_temp = (contours[i])[j].x;
                    yinMaxX = (contours[i])[j].y;
                }
                if ((contours[i])[j].y < minY_temp) {
                    minY_temp = (contours[i])[j].y;
                }
                if ((contours[i])[j].y > maxY_temp) {
                    maxY_temp = (contours[i])[j].y;
                }
            }

            //width and height of the contours are compliants with the line
            if (InRange(maxY_temp - minY_temp, height + MIN_HEIGHT_LINE(lineBandGreyThres.rows, lineBandGreyThres.cols),
                        height + MAX_HEIGHT_LINE(lineBandGreyThres.rows+1, lineBandGreyThres.cols))) {

                if (calibrate) {
                    width = maxX_temp - minX_temp;
                }
                if (InRange(maxX_temp - minX_temp, width + MIN_WIDTH_LINE(lineBandGreyThres.rows, lineBandGreyThres.cols),
                            width + MAX_WIDTH_LINE(lineBandGreyThres.rows, lineBandGreyThres.cols))) {

                    //printf("\nLINE DETECTED w=%d h=%d rows=%d!!!\n", maxX_temp - minX_temp, maxY_temp - minY_temp, lineBandGreyThres.rows);
                    int average = (maxX_temp + minX_temp) / 2;

                    //
                    curvature += ((maxX_temp - minX_temp) > width) ? (SIGN_VAL(yinMaxX - yinMinX) * ((maxX_temp - minX_temp) / ((float)width) - 1.)) : (0.);

                    if (average < minX) {
                        minX = average;
                    }
                    if (average > maxX) {
                        maxX = average;
                    }
                    linesCounter++;
                }
            }
        }
    }
    curvature /= linesCounter;

    if (calibrate) {
        refLeft = minX;
        refRight = maxX;
        if ((linesCounter >= 2) && (width > 0)) {
            return DRIVE_ZERO_CONTROL;
        }
        else {
            return -1;
        }
    }
#ifdef SHOW_IMAGES
    //draw contours on the band
    for (int i = 0; i < contours.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        drawContours(lineBandGreyThres, contours, i, color, 2, 8, hierarchy, 0, Point());
    }
#endif

    StateMachine(minX, maxX, minX_used, maxX_used, minX_1, maxX_1, curvature, linesCounter, 0, lineBandGreyThres.cols, initialMinX, initialMaxX, speedControl);

    // something has been detected
    float control = DRIVE_ZERO_CONTROL;
    PIDController pid(KP);

    if (minX_used <= maxX_used) {
        switch (status) {
        case (FOLLOW_LEFT): {
            //assume left line detected
            //if status is follow left
            float error = refLeft - minX_used;

            //get the drive control on this error
            control = pid.Execute(error);
        }
            break;
        case (FOLLOW_RIGHT): {
            //assume right line detected
            //if status is follow right
            float error = refRight - maxX_used;
            //printf("\nerror=%f\n", error);
            //get the drive control on this error
            control = pid.Execute(error);
        }
            break;
        default: {
            control = DRIVE_ZERO_CONTROL;                        //??
        }
        }
    }

   // if (linesCounter >= 2) {
        minX_1 = minX;
        maxX_1 = maxX;
    //}

    // printf("\nline status=%d", lineStatus);
    //printf("Min=%d Max=%d\n", minX, maxX);
    return PWM_DRIVE_REMAP(control);
}
