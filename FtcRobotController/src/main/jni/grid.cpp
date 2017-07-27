
#include "grid.h"
#include "p_Block.h"
#include "stepPoint.h"
#include <math.h>
#include <android/log.h>

//a grid will be initialized with the number of rows and columns it has
grid::grid(int rows, int cols) {
    //set our rows and columns
    m_rows = rows;
    m_cols = cols;
    //reset everything from the previous path-finding
    allBlocks = std::vector<p_Block>();
    m_steps = std::vector<stepPoint>();
    //create all the blocks, but assign them a blank target
    for(int X = 0; X<m_cols;X++){
        for(int Y = 0; Y<m_rows;Y++){
            allBlocks.push_back(p_Block(X,Y));
        }
    }

}

void grid::setTarget(int x, int y) {
    m_targetX = x;
    m_targetY = y;
    //loop through all the blocks and set their target.
    //this will also calculate their h scores.
    for(int X = 0; X<m_cols;X++){
        for(int Y = 0; Y<m_rows;Y++){
            getBlock(X,Y)->setTarget(m_targetX,m_targetY);
        }
    }

}
//if the used decides that this block should be avoided, mark it
void grid::setObstacle(int row, int col) {
    //mark that block as an obstacle
    getBlock(row,col)->invalid = true;
}
//since we know how our data is organized, we can efficiently access blocks
p_Block* grid::getBlock(int row, int col){
    int index = (row*m_cols)+col;
    return &allBlocks.at(index);
}

void grid::setOnlyHorizontal(int row, int col) {
    getBlock(row,col)->horizontalOnly = true;
}

void grid::setOnlyVertical(int row, int col) {
    getBlock(row,col)->verticalOnly = true;
}

void grid::findPath(int myPosX, int myPosY) {
    //add ourselves to the closed list
    p_Block* ourself = getBlock(myPosX,myPosY);
    ourself->g_score = 0; // our movement cost is 0 since it is the starting point

    //__android_log_print(ANDROID_LOG_DEBUG, "LOG_TAG", "target x: %d, y: %d, start x: %d, y: %d" ,m_targetX,m_targetY, myPosX,myPosY);

    currentBlock = ourself;
    for(;;){

        calcPathStep();
        if(currentBlock->x==m_targetX && currentBlock->y == m_targetY){
            break;
        }
    }


    //trace the path via parents. The step ArrayList will be in reverse order
    std::vector<stepPoint> steps = std::vector<stepPoint>();
    p_Block* stepBlock = currentBlock;

    //remember if the previous block was constrained, because we need to figure out
    //which blocks we absolutely must go through to preserve the constraints
    bool lastBlockConstrained = false;
    for(;;){
        if(stepBlock->x == ourself->x and stepBlock->y == ourself->y){
            break;
        }

        bool needsToGoThrough = false;
        //if this block is not constrained however the last block was, add it as a
        //point that needs to be gone through in order to preserve it's constraint
        if(!stepBlock->horizontalOnly && !stepBlock->verticalOnly && lastBlockConstrained){
            needsToGoThrough = true;
        }

        steps.push_back(stepPoint(stepBlock->x,stepBlock->y,needsToGoThrough));


        if(stepBlock->horizontalOnly or stepBlock->verticalOnly){
            lastBlockConstrained = true;
        }else{
            lastBlockConstrained = false;
        }
        stepBlock = stepBlock->m_parent;
    }

    m_steps = steps;
    std::reverse(m_steps.begin(), m_steps.end());

}

void grid::calcPathStep() {
    currentBlock->closed = true;
    currentBlock->open = false;



    //start comparing blocks at the bottom left going around us
    for(int adjX = currentBlock->x - 1; adjX <= currentBlock->x + 1; adjX ++){
        for(int adjY = currentBlock->y -1; adjY <= currentBlock->y +1; adjY++){

            //if the adjacent spot we are comparing is currently a closed block
            bool unusable = false;

            ////////Check if this block is closed, invalid, or outside the grid.
            ///If so, then don't do anything


            //start by seeing if the block is out of bounds to make sure we are not accessing
            //a non-existent ArrayList member.
            if(adjX < 0 or adjY < 0 or adjX >= m_rows or adjY >= m_cols){
                unusable = true;
            }else{
                if(getBlock(adjX,adjY)->closed or getBlock(adjX,adjY)->invalid){
                    unusable = true;
                }
            }
            //don't allow diagonal movements because they screw us when trying to determine the
            //path with the least possible turns
            if(sqrt(pow(adjX-currentBlock->x,2)+pow(adjY-currentBlock->y,2)) > 1){
                unusable = true;
            }

            if(!unusable){//only process this is the block is not unusable

                bool okayToParent=true;
                //we can not create a new block with us as the parent if it violates any of
                //the constraints like horizontal and vertical only constraints.
                if(currentBlock->verticalOnly){
                    if(!currentBlock->isVertical(getBlock(adjX,adjY))){
                        okayToParent = false;
                    }
                }
                if(currentBlock->horizontalOnly){
                    if(!currentBlock->isHorizontal(getBlock(adjX,adjY))){
                        okayToParent = false;
                    }
                }

                double newMovementCost = p_Block::calc_g_score(currentBlock,getBlock(adjX,adjY));
                if((newMovementCost < getBlock(adjX,adjY)->g_score || getBlock(adjX,adjY)->m_parent == NULL) and okayToParent){

                    //setting the parent will also calculate the g_cost and f_cost
                    //it will also mark this block as open
                    getBlock(adjX,adjY)->setParent(currentBlock);
                }
            }
        }
    }
    //choose the block with the lowest f_score to be the current block
    double lowestFScore = 100000000000000.0;
    p_Block* lowestFBlock = NULL;
    for(int i = 0; i< allBlocks.size();i++){
        if(allBlocks.at(i).open){
            if(allBlocks.at(i).f_score < lowestFScore){
                lowestFScore = allBlocks.at(i).f_score;
                lowestFBlock = &allBlocks.at(i);
            }
        }
    }
    currentBlock=lowestFBlock;
}