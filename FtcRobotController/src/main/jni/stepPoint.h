//
// Created by Administrator on 7/20/2017.
//
#include "opencv2/opencv.hpp"

#ifndef FTC_2017_2018_STEPPOINT_H
#define FTC_2017_2018_STEPPOINT_H


class stepPoint {
public:
    int x;
    int y;
    cv::Point p;
    bool mandatory = false;
    stepPoint(int X, int Y, bool mandatory);
};


#endif //FTC_2017_2018_STEPPOINT_H
