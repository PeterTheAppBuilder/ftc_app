#include "p_Block.h"
#include <math.h>

p_Block::p_Block(int x, int y) {
    this->x = x;
    this->y = y;
}
void p_Block::setTarget(int target_x, int target_y){
    //calculate the h_score using the Manhattan method where scores increase based on their
    //sum of the delta x and y positions
    h_score = abs(target_x-x)+abs(target_y-y);
}
void p_Block::setParent(p_Block* parent){
    m_parent = parent;
    g_score = calc_g_score(m_parent,this);
    open = true;
    if(h_score != NULL){//if our h score has been set, we can calculate our f_score
        f_score = h_score+g_score;
    }
}

double p_Block::calc_g_score(p_Block* parentBlock, p_Block* block2) {
    //first calculate the cost of moving from the parent to child
    double g_score_relative = sqrt(pow(block2->x-parentBlock->x,2)
                                        +pow(block2->y-parentBlock->y,2));
    //add the parents current g_score
    double g_score_sum = parentBlock->g_score + g_score_relative;

    //the parent of the parent could be null if the parent of this block is the starting block
    if(parentBlock->m_parent != NULL && parentBlock != NULL){
        double slopeBefore;
        if(parentBlock->m_parent->x-parentBlock->x!=0){
            slopeBefore = (parentBlock->m_parent->y-parentBlock->y)/
                          (parentBlock->m_parent->x-parentBlock->x);
        }else{
            //if there is a divide by zero for the slope, set it to a really big number, but
            //it doesn't really matter
            slopeBefore = 999;
        }
        double slopeAfter;
        if(parentBlock->x-block2->x!=0){
            slopeAfter = (parentBlock->y - block2->y)/(parentBlock->x-block2->x);
        }else{
            slopeAfter = 999;
        }

        if(slopeAfter!=slopeBefore){
            g_score_sum += 1.9;
        }
    }
    g_score_sum += block2->pre_g_score;


    return g_score_sum;
}

bool p_Block::isHorizontal(p_Block* otherBlock){
    if(otherBlock->y == this->y && abs(this->x-otherBlock->x) == 1){
        return true;
    }
    return false;
}

bool p_Block::isVertical(p_Block *otherBlock) {
    if(otherBlock->x == this->x && abs(this->y-otherBlock->y) == 1){
        return true;
    }
    return false;
}

void p_Block::setObstical(bool o){
    this->invalid = o;
}