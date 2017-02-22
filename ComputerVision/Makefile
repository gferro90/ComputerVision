
BUILD=${CURDIR}/Build

CC = gcc
CXX = g++
LD = g++

INCL_DIRS = -I/home/pc/OpenCV/opencv/opencv/release/opencv2
INCL_DIRS += -I/home/pc/Kinect/kinect/OpenNI-OpenNI-e263e59/Include
#usr/include/ni
LIB_DIRS = -L/home/pc/OpenCV/opencv/opencv/release/lib
#-L/home/pc/Desktop/opencv-2.4.10/build/lib 


CFLAGS =
CPPFLAGS = 
#-DSAMPLE_XML_PATH="/home/pc/Kinect/kinect/SensorKinect/OpenNI/Data/SamplesConfig.xml"

OPTIM = -W -Wall -O2 -g


LFLAGS = -g
LIBS = -lopencv_shape -lopencv_stitching -lopencv_objdetect -lopencv_superres -lopencv_videostab -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_video -lopencv_photo -lopencv_ml -lopencv_imgproc -lopencv_flann -lopencv_viz -lopencv_core  -lOpenNI -lglut -lGL
#-lopencv_core -lopencv_calib3d -lopencv_highgui -lOpenNI -lglut -lGL

COBJS =
#CPPOBJS = mioo.cpp
CPPOBJS= main.cpp SceneDrawer.cpp

OBJS = $(COBJS:%.c=$(BUILD)/%.o)
OBJS += $(CPPOBJS:%.cpp=$(BUILD)/%.o)



TARGET = $(BUILD)/output




%all: $(TARGET) $(OBJS)


$(BUILD)/%.o: %.c
	$(CC) -c $(CFLAGS) $(INCL_DIRS) $(OPTIM) $< -o $@ 

$(BUILD)/%.o: %.cpp
	$(CXX) -c $(CPPFLAGS) $(INCL_DIRS) $(OPTIM) $< -o $@ 

$(TARGET): $(OBJS)
	$(LD) $(LFLAGS) $(OBJS) $(LIB_DIRS) $(LIBS) -o $@

clean:
	rm $(OBJS) $(TARGET)
