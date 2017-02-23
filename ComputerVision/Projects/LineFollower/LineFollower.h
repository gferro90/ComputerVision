/**
 * @file LineFollower.h
 * @brief Header file for class LineFollower
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

 * @details This header file contains the declaration of the class LineFollower
 * with all of its public, protected and private members. It may also include
 * definitions for inline methods which need to be visible to the compiler.
 */

#ifndef COMPUTERVISION_PROJECTS_LINEFOLLOWER_LINEFOLLOWER_H_
#define COMPUTERVISION_PROJECTS_LINEFOLLOWER_LINEFOLLOWER_H_

/*---------------------------------------------------------------------------*/
/*                        Standard header includes                           */
/*---------------------------------------------------------------------------*/
#include <cv.h>
#include <highgui.h>
#include <cxcore.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
/*---------------------------------------------------------------------------*/
/*                        Project header includes                            */
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/*                           Class declaration                               */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                        Inline method definitions                          */
/*---------------------------------------------------------------------------*/



#define CAMERA 0

//definitions of HSV color ranges for signal recognition
#define BLUE1 cvScalar(0, 0, 0, 0) //TODO
#define BLUE2 cvScalar(0, 0, 0, 0) //TODO

#define GREEN1 cvScalar(0, 0, 0, 0) //TODO
#define GREEN2 cvScalar(0, 0, 0, 0) //TODO

#define RED1 cvScalar(0, 0, 0, 0)//TODO
#define RED2 cvScalar(0, 0, 0, 0)//TODO

//definition of the line follower band
#define LINE_BAND_X(rows, cols) 10
#define LINE_BAND_Y(rows, cols) ((2 * rows) / 3)
#define LINE_BAND_WIDTH(rows, cols) (cols - 2*(LINE_BAND_X(rows,cols)))
#define LINE_BAND_HEIGHT(rows, cols) (rows / 12)

//definition of the signal radius ranges
#define SIGNAL_MIN_RADIUS 0 //TODO
#define SIGNAL_MAX_RADIUS 20000 //TODO

//definition of left and right lines ranges
#define MAX_X_LINE_RIGHT(rows, cols) 0//TODO
#define MIN_X_LINE_RIGHT(rows, cols) 0//TODO
#define MAX_X_LINE_LEFT(rows, cols) 0//TODO
#define MIN_X_LINE_LEFT(rows, cols) 0//TODO

// definition of the references of left and right boundaries
#define X_REF_LEFT(rows, cols) 0 //TODO
#define X_REF_RIGHT(rows, cols) 0 //TODO

// define the controller's parameters
#define KP 1

// define the control remapping parameters
#define CONTROL_MIN 0
#define CONTROL_MAX 0
#define PWM_MIN 0
#define PWM_MAX 0

//define usb port
#define USB_PORT "/dev/ttyACM0"
// define usb baud rate
#define USB_BAUDRATE 115200

enum stati{FOLLOW_RIGHT=0, FOLLOW_LEFT};

#define ASSIGN_SIGNAL_NAME(name, color, function) {name, color##1, color##2, function}

struct ControlOutput {
    int pwmSpeed;
    int pwmDrive;
};

struct ColorRange {
    const char * signalName;
    CvScalar range1;
    CvScalar range2;
    //get the status and returns speed control
    int (*SignalAction)(int&);
};

#include "SignalActions.h"
static ColorRange signalColorRange[] = { ASSIGN_SIGNAL_NAME("TurnRight",GREEN,TurnRightAction), ASSIGN_SIGNAL_NAME("TurnLeft",RED,TurnLeftAction), 0 };



#endif /* COMPUTERVISION_PROJECTS_LINEFOLLOWER_LINEFOLLOWER_H_ */

