/**
 * @file Utils.h
 * @brief Header file for class Utils
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

 * @details This header file contains the declaration of the class Utils
 * with all of its public, protected and private members. It may also include
 * definitions for inline methods which need to be visible to the compiler.
 */

#ifndef COMPUTERVISION_PROJECTS_LINEFOLLOWER_UTILS_H_
#define COMPUTERVISION_PROJECTS_LINEFOLLOWER_UTILS_H_

/*---------------------------------------------------------------------------*/
/*                        Standard header includes                           */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                        Project header includes                            */
/*---------------------------------------------------------------------------*/
#include "LineFollower.h"
/*---------------------------------------------------------------------------*/
/*                           Class declaration                               */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                        Inline method definitions                          */
/*---------------------------------------------------------------------------*/
using namespace cv;
using namespace std;


ColorRange *GetStreetSignalRanges(const char* input,
                                  ColorRange *signalColorRange);
int USBConfig();

int PwmRemapping(float control,
                 float minIn,
                 float maxIn,
                 int minOut,
                 int maxOut);

int DetectSignal(IplImage *hsv_frame,
                 IplImage *thresholded,
                 ColorRange &colorRange,
                 stati& status);

int FollowLine(Mat &lineBandGreyThres,
               double initialMinX,
               double initialMaxX,
               double initialMinY,
               double initialMaxY,
               stati status);

#endif /* COMPUTERVISION_PROJECTS_LINEFOLLOWER_UTILS_H_ */

