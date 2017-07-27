//
// Created by Administrator on 7/20/2017.
//
#include <vector>
#include "p_Block.h"
#include "stepPoint.h"

#ifndef FTC_2017_2018_GRID_H
#define FTC_2017_2018_GRID_H


class grid {
public:
    int m_rows,m_cols;
    int m_targetX, m_targetY;

    //the open and closed list of blocks
    std::vector<p_Block> allBlocks;
    std::vector<stepPoint> m_steps;

    grid(int rows, int cols);

    void setTarget(int x, int y);

    void setObstacle(int row, int col);

    p_Block* getBlock(int row, int col);

    void setOnlyHorizontal(int row, int col);

    void setOnlyVertical(int row, int col);

    void findPath(int myPosX, int myPosY);

    void calcPathStep();
private:
    p_Block* currentBlock;

};


#endif //FTC_2017_2018_GRID_H
