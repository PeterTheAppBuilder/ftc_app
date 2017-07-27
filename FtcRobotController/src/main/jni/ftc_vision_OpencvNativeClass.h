#include <jni.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

#ifndef _Included_ftc_vision_OpencvNativeClass
#define _Included_ftc_vision_OpencvNativeClass
#ifdef __cplusplus
extern "C" {
#endif

KeyPoint findBalls(Mat img, Mat& gray,bool blueside);
void calibrate();
void findLines(Mat rawImage, Mat& drawingImage);


static int bestBallX = 1;
static int bestBallY = 1;

static int bestBallXAbs = NULL;
static int bestBallYAbs = NULL;

static int bestBallRadius = 1;
static int minH = 93;
static int minS = 100;
static int minV = 10;
static int maxH = 105;
static int maxS = 255;
static int maxV = 255;



static int avH =108;
static int avS = 221;
static int avV = 95;

static Mat currentImage;


JNIEXPORT void JNICALL Java_ftc_vision_OpencvNativeClass_findBalls
  (JNIEnv * , jclass, jlong, jlong,jboolean);

JNIEXPORT void JNICALL Java_ftc_vision_OpencvNativeClass_calibrate
(JNIEnv * , jclass);



#ifdef __cplusplus
}
#endif
#endif
