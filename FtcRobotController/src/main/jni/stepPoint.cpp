//
// Created by Administrator on 7/20/2017.
//
#include "stepPoint.h"
#include "opencv2/opencv.hpp"



stepPoint::stepPoint(int X, int Y, bool mandatory) {
    x = X;
    y = Y;
    p = cv::Point(X,Y);
    this->mandatory = mandatory;
}
