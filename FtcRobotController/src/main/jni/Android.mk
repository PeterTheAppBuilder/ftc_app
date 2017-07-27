    LOCAL_PATH := $(call my-dir)

	include $(CLEAR_VARS)

	#opencv
	OPENCVROOT:= C:\OpenCV-android-sdk
	OPENCV_CAMERA_MODULES:=on
	OPENCV_INSTALL_MODULES:=on
	OPENCV_LIB_TYPE:=SHARED
	include ${OPENCVROOT}/sdk/native/jni/OpenCV.mk

	LOCAL_SRC_FILES := ftc_vision_OpencvNativeClass.cpp p_Block.cpp grid.cpp stepPoint.cpp pathFinder_I.cpp

	LOCAL_LDLIBS += -llog
	LOCAL_MODULE := MyOpencvLibs


	include $(BUILD_SHARED_LIBRARY)