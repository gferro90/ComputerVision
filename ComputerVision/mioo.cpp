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
using namespace xn;
using namespace cv;
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

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------


// Let's try a more efficient visualization.
void convert_pixel_map_i(const XnRGB24Pixel* pImageMap, Mat& cv_image, int rows, int cols)
{
  int sizes[2] = {rows, cols};
  cv_image = Mat(2, sizes, CV_8UC3, (void*) pImageMap);
}
 
// Efficient!
void convert_pixel_map_d(const XnDepthPixel* pDepthMap, Mat& cv_depth, int rows, int cols)
{
  int sizes[2] = {rows, cols};
  cv_depth = Mat(2, sizes, CV_16UC1, (void*) pDepthMap);
}






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



int main(int argc, char* argv[])
{

	XnMapOutputMode imageMapMode;
	XnMapOutputMode depthMapMode;

	XnStatus rc;

	EnumerationErrors errors;
	rc = g_context.InitFromXmlFile(SAMPLE_XML_PATH, g_scriptNode, &errors);
	if (rc == XN_STATUS_NO_NODE_PRESENT)
	{
		XnChar strError[1024];
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		return (rc);
	}
	else if (rc != XN_STATUS_OK)
	{
		printf("Open failed: %s\n", xnGetStatusString(rc));
		return (rc);
	}

	rc = g_context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_depth);
	if (rc != XN_STATUS_OK)
	{
		printf("No depth node exists! Check your XML.");
		return 1;
	}

	rc = g_context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_image);
	if (rc != XN_STATUS_OK)
	{
		printf("No image node exists! Check your XML.");
		return 1;
	}
	
	Mat rgb;
	Mat depth;

	namedWindow("Display Image", CV_WINDOW_AUTOSIZE );
	namedWindow("Display Depth", CV_WINDOW_AUTOSIZE );
	printf("Before while\n");
	while(1){
	        rc = g_context.WaitAnyUpdateAll();
 		g_image.GetMapOutputMode(imageMapMode);
 		g_depth.GetMapOutputMode(depthMapMode);


		const XnDepthPixel* pDepthMap = g_depth.GetDepthMap();
		const XnRGB24Pixel* pImageMap = g_image.GetRGB24ImageMap();

		

		convert_pixel_map_d(pDepthMap, depth, depthMapMode.nYRes, depthMapMode.nXRes);
		convert_pixel_map_i(pImageMap, rgb,   imageMapMode.nYRes, imageMapMode.nXRes);


		printf("dimensions= %d %d\n",depthMapMode.nYRes, depthMapMode.nXRes);
		imshow("Display Image",rgb);
		imshow("Display Depth",depth);
		waitKey(0);
		printf("After imshow\n");
	 }
	// Use Mats...
 
	// Important, since we initialized from remote pointers.
	depth.release();
	rgb.release();

	return 0;
}
