
#source code to be compiled here
OBJSX= LineFollower.x PIDController.x Utils.x SignalActions.x

#paths of other projects makefiles
SPB = 

#include specific platform definition (the flag TARGET must specify the target architecture)
-include PlatformDefinitions

ROOT_DIR=../..

#pathes with the default flags and rules
MAKEDEFAULTDIR=$(ROOT_DIR)/MakeDefaults

#include the common flags definitions
include $(MAKEDEFAULTDIR)/MakeStdLibDefs.$(TARGET)

#build dir
#BUILD_DIR=Build

#linker file
#LINKER_LD_FILE=

#add other CFLAGS if needed
#CFLAGS +=

#add other includes if needed 
INCLUDES = -I/home/pc/OpenCV/opencv-3.0.0/include/opencv2
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/include/opencv 
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/core
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/cudaarithm
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/cudabgsegm
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/cudacodec
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/cudafeatures2d
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/cudafilters
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/cudaimgproc
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/cudalegacy
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/cudaobjdetect
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/cudaoptflow
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/cudastereo
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/cudawarping
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/cudev
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/features2d
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/flann
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/hal
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/highgui
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/imgcodecs
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/imgproc
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/java
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/ml
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/objdetect
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/photo
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/python
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/shape
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/stitching
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/superres
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/ts
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/video
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/videoio
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/videostab
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/viz
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/modules/world
INCLUDES += -I/home/pc/Kinect/kinect/OpenNI-OpenNI-e263e59/Include
# Static project libraries
LIBRARIES_STATIC = 

#targets
all: $(SUBPROJ) $(OBJS) \
	$(BUILD_DIR)/LineFollower$(EXEEXT)

#include rules
include $(MAKEDEFAULTDIR)/MakeStdLibRules.$(TARGET)

