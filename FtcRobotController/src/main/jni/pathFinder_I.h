//
// Created by Administrator on 7/20/2017.
//
#include <jni.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "grid.h"


using namespace cv;
using namespace std;

#ifndef _Included_ftc_pathfinder_pathFinderNative
#define _Included_ftc_pathfinder_pathFinderNative
#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT jfloatArray JNICALL Java_ftc_pathfinder_pathFinderNative_calcPath
    (JNIEnv * , jclass, jdouble, jdouble,jdouble,jdouble,jdouble,jlong,jboolean);

JNIEXPORT void JNICALL Java_ftc_pathfinder_pathFinderNative_drawPath
        (JNIEnv * , jclass, jlong, jint);



std::vector<cv::Point2f> m_filteredSteps;
grid m_grid = grid(0,0);
cv::Mat* fieldMap;

Scalar color_obstacle = Scalar(255,255,255);
//these color values specify if the robot can only move in one direction while over them
//that way it will not approach wall balls from angles that the collector can't handle
Scalar color_allowsOnlyVertical = Scalar(255,0,0);
Scalar color_allowsOnlyHorizontal = Scalar(0,0,255);
Scalar color_penalty1 = Scalar(0,255,0);


//this variable makes sure that we always choose one way to go around obstacles
//by drawing a line from start to finish, we know if we are maneuvering around an
//invalid patch. We want to go with our first decision around this patch to commit
//to a decision. Therefore, if we are about to take a path that is obstructed by
//obstacles, remember that the last calculation already made a decision, and don't
//re-calculate the path next time. Once we are free of the obstacle, we can update
//the path.
bool obstructedObstaclesLast = false;


void createGrid();
vector<double> posToBlock(double x, double y, double fieldSize);
bool scalar_equals(Scalar a, Scalar b);
bool calcPath(double my_X, double my_Y, double target_X, double target_Y, double fieldSize);
int countObsticals(Mat* map);



#ifdef __cplusplus
}
#endif
#endif