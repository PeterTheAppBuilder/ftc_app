package ftc.pathfinder;

import org.opencv.core.Point;

import java.util.ArrayList;

public class grid{
    public int m_rows,m_cols;
    public int m_targetX, m_targetY;

    //the open and closed lists of blocks
    //there are also onstraint lists that prohibit movements in certain directions
    public ArrayList<p_Block> allBlocks;

    //a grid will be initialized with the number of rows and columns it has
    public grid(int rows, int cols){
        allBlocks = new ArrayList<>();
        //set our rows and columns
        m_rows = rows;
        m_cols = cols;
    }

    public void setTarget(int x,int y){
        m_targetX = x;
        m_targetY = y;


        //now that we know the target, we can create all of the blocks
        for(int X = 0; X<m_rows;X++){
            for(int Y = 0; Y<m_cols;Y++){
                allBlocks.add(new p_Block(X,Y,m_targetX,m_targetY));
            }
        }
    }
    //if the user decides that this block should be avoided, mark it
    public void setObstacle(int row, int col){
        //mark that block as an obstacle
        getBlock(row,col).invalid = true;
    }

    public p_Block getBlock(int row, int col){
        int index = (row*m_cols)+col;
        return allBlocks.get(index);
    }

    public void setOnlyDown(int row, int col){
        getBlock(row,col).downOnly = true;
    }
    public void setOnlyLeft(int row, int col){
        getBlock(row,col).leftOnly = true;
    }
    public void setOnlyUp(int row, int col){
        getBlock(row,col).upOnly = true;
    }
    public void setOnlyRight(int row, int col){
        getBlock(row,col).rightOnly = true;
    }


    public void findPath(int myPosX, int myPosY){
        //add ourselves to the closed list
        p_Block ourself = new p_Block(myPosX,myPosY,m_targetX,m_targetY);
        ourself.g_score = 0.0f; // our movement cost is 0 since it is the starting point



        p_Block currentBlock = ourself;
        for(;;){
            currentBlock.closed = true;
            currentBlock.open = false;

            //start comparing blocks at the bottom left going around us
            for(int adjX = currentBlock.x - 1; adjX <= currentBlock.x + 1; adjX ++){
                for(int adjY = currentBlock.y -1; adjY <= currentBlock.y +1; adjY++){
                    //if the adjacent spot we are comparing is currently a closed block
                    boolean closedBlockHere = false;

                    ////////Scan to see if there is a closed block or invalid block already here
                    for(p_Block iterBlock: allBlocks){
                        if(iterBlock.closed || iterBlock.invalid){
                            if(iterBlock.x == adjX && iterBlock.y == adjY){
                                closedBlockHere = true;
                            }
                        }
                    }
                    //Can't scan negative blocks outside the grid
                    if(adjX < 0||adjY < 0 || adjX > m_rows || adjY > m_cols){
                        closedBlockHere = true;
                    }


                    if(!closedBlockHere){

                        //if there is already a block in the adjacent space that has a parent,
                        //remember it to see if we should override it's parent
                        boolean competingParent = false;

                        if(getBlock(adjX,adjY).m_parent != null){
                            //remember that there was a competing parent
                            competingParent = true;
                        }

                        if(!competingParent){
                            //setting the parent will also calculate the g_score
                            getBlock(adjX,adjY).setParent(currentBlock);

                            //add it to the open list since all is well
                            getBlock(adjX,adjY).open = true;
                            getBlock(adjX,adjY).closed = false;
                        }else{//if there is a competing parent
                            //See if it's g value is improved
                            //by going through us. If so, parent it to us.
                            //this will set the block's g score and f score since it has all it needs
                            double g_score_through_us = p_Block.calc_g_score(currentBlock,getBlock(adjX,adjY));

                            boolean isOkayToOverride = true;
                            for(p_Block iterBlock: allBlocks){
                                if(iterBlock.invalid){
                                    if(currentBlock.isCutCorner(getBlock(adjX,adjY),iterBlock)){
                                        isOkayToOverride = false;
                                    }
                                }
                            }
                            if(g_score_through_us < getBlock(adjX,adjY).g_score && isOkayToOverride){
                                getBlock(adjX,adjY).setParent(currentBlock);
                            }
                        }
                    }
                }
            }
            //choose the block with the lowest f_score to be the current block
            double lowestFScore = 100000000000000.0;
            p_Block lowestFBlock = null;
            for(p_Block thisBlock:allBlocks){
                if(thisBlock.open){
                    p_Block thisBlocksParent = thisBlock.m_parent;

                    boolean isOkay = true;
                    //check if moving to this block would cut a corner. If so, don't move there
                    for(p_Block invalidBlock: allBlocks){
                        if(invalidBlock.invalid && thisBlocksParent.isCutCorner(thisBlock,invalidBlock)){
                            isOkay = false;
                        }
                    }
                    //check if moving to this block would violate a down, left,
                    // up, or right only constraint. If so, don't move there
                    if(thisBlocksParent.downOnly){
                        //if the other block is not to the left of us and it is marked as left only, it's not okay
                        if(!thisBlocksParent.isDown(thisBlock)){
                            isOkay = false;
                        }
                    }
                    if(thisBlocksParent.leftOnly){
                        //if the other block is not to the left of us and it is marked as left only, it's not okay
                        if(!thisBlocksParent.isLeft(thisBlock)){
                            isOkay = false;
                        }
                    }
                    if(thisBlocksParent.upOnly){
                        //if the other block is not to the left of us and it is marked as left only, it's not okay
                        if(!thisBlocksParent.isUp(thisBlock)){
                            isOkay = false;
                        }
                    }
                    if(thisBlocksParent.rightOnly){
                        //if the other block is not to the left of us and it is marked as left only, it's not okay
                        if(!thisBlocksParent.isRight(thisBlock)){
                            isOkay = false;
                        }
                    }




                    if(thisBlock.f_score < lowestFScore && isOkay){
                        lowestFScore = thisBlock.f_score;
                        lowestFBlock = thisBlock;
                    }
                }

            }
            currentBlock = lowestFBlock;
            if(currentBlock.x == m_targetX && currentBlock.y == m_targetY){
                break;
            }
        }
        //trace the path via parents. The step ArrayList will be in reverse order
        ArrayList<Point> steps = new ArrayList<Point>();
        p_Block stepBlock = currentBlock;
        for(;;){
            steps.add(new Point(stepBlock.x,stepBlock.y));
            if(stepBlock == ourself){
                break;
            }
            stepBlock = stepBlock.m_parent;
        }
        m_steps = steps;
    }
    public ArrayList<Point> m_steps;
}
