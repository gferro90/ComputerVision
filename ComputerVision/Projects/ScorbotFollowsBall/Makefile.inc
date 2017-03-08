
#source code to be compiled here
OBJSX= main.x ../Generic/PIDController.x

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
INCLUDES = -I../Generic
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/include/opencv2
INCLUDES += -I/home/pc/OpenCV/opencv-3.0.0/include/opencv 
INCLUDES += -I/home/pc/Kinect/kinect/OpenNI-OpenNI-e263e59/Include

# Static project libraries
LIBRARIES_STATIC = 

#targets
all: $(SUBPROJ) $(OBJS) \
	$(BUILD_DIR)/ScorbotFollowsBall$(EXEEXT)

#include rules
include $(MAKEDEFAULTDIR)/MakeStdLibRules.$(TARGET)

