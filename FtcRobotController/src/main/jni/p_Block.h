//
// Created by Administrator on 7/20/2017.
//
#include <jni.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>

#ifndef FTC_2017_2018_P_BLOCK_H
#define FTC_2017_2018_P_BLOCK_H

//#ifdef __cplusplus
//extern "C" {
//#endif
class p_Block {
public:
    int x;
    int y;
    bool open = false;
    bool closed = false;
    bool invalid = false;
    bool horizontalOnly=false;
    bool verticalOnly=false;
    //the h_score represents a score given to each block that increases based on how
    //many steps away it is from the target. It is the sum of the difference in x values
    //and the difference in y. This way of calculating distance is known as the Manhattan method.
    //All values in this variable are positive.
    int h_score = 0;
    //the g_score represents a score given to parented blocks that represents how
    //much it will cost to move there. For example, blocks that are diagonal from
    //a parent block cost more than ones that are straight up or down.
    double g_score = 0.0;
    //this is a predefined movement penalty set by the field map.
    double pre_g_score = 0;
    //the sum of our g and h score
    double f_score = 0;
    //initialize ourselves by setting our x and y pos
    p_Block* m_parent = NULL;



    p_Block(int x, int y);
    void setTarget(int target_x, int target_y);
    void setParent(p_Block* parent);
    static double calc_g_score(p_Block* parent, p_Block* block2);
    bool isHorizontal(p_Block* otherBlock);
    bool isVertical(p_Block* otherBlock);
    void setObstical(bool o);
};
//#ifdef __cplusplus
//}
//#endif
#endif

