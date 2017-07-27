package ftc.pathfinder;

import android.os.SystemClock;

import com.qualcomm.hardware.ams.AMSColorSensor;

import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Collections;

public class grid {
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

        //reset everything from the previous pathfinding
        allBlocks = new ArrayList<>();
        m_steps = new ArrayList<>();

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

    public void setOnlyHorizontal(int row, int col){
        getBlock(row,col).horizontalOnly = true;
    }
    public void setOnlyVertical(int row, int col){
        getBlock(row,col).verticalOnly = true;
    }


    private p_Block currentBlock;

    public void findPath(int myPosX, int myPosY){
        //add ourselves to the closed list
        p_Block ourself = new p_Block(myPosX,myPosY,m_targetX,m_targetY);
        ourself.g_score = 0.0f; // our movement cost is 0 since it is the starting point

        currentBlock = ourself;
        for(;;){
            calcPathStep();
            if(currentBlock.x==m_targetX && currentBlock.y == m_targetY){
                break;
            }
        }

        //trace the path via parents. The step ArrayList will be in reverse order
        ArrayList<stepPoint> steps = new ArrayList<stepPoint>();
        p_Block stepBlock = currentBlock;

        //remember if the previous block was constrained, because we need to figure out
        //which blocks we absolutely must go through to preserve the constraints
        boolean lastBlockConstrained = false;
        for(;;){
            if(stepBlock == ourself){
                break;
            }

            boolean needsToGoThrough = false;
            //if this block is not constrained however the last block was, add it as a
            //point that needs to be gone through in order to preserve it's constraint
            if(!stepBlock.horizontalOnly && !stepBlock.verticalOnly && lastBlockConstrained){
                needsToGoThrough = true;
            }

            steps.add(new stepPoint(stepBlock.x,stepBlock.y,needsToGoThrough));


            if(stepBlock.horizontalOnly || stepBlock.verticalOnly){
                lastBlockConstrained = true;
            }else{
                lastBlockConstrained = false;
            }
            stepBlock = stepBlock.m_parent;
        }
        m_steps = steps;
        Collections.reverse(m_steps);

    }
    public ArrayList<stepPoint> m_steps;
    public void calcPathStep(){
        currentBlock.closed = true;
        currentBlock.open = false;

        //start comparing blocks at the bottom left going around us
        for(int adjX = currentBlock.x - 1; adjX <= currentBlock.x + 1; adjX ++){
            for(int adjY = currentBlock.y -1; adjY <= currentBlock.y +1; adjY++){

                //if the adjacent spot we are comparing is currently a closed block
                boolean unusable = false;

                ////////Check if this block is closed, invalid, or outside the grid.
                ///If so, then don't do anything


                //start by seeing if the block is out of bounds to make sure we are not accessing
                //a non-existent ArrayList member.
                if(adjX < 0||adjY < 0 || adjX >= m_rows || adjY >= m_cols){
                    unusable = true;
                }else{
                    if(getBlock(adjX,adjY).closed || getBlock(adjX,adjY).invalid){
                        unusable = true;
                    }
                }
                //don't allow diagonal movements because they screw us when trying to determine the
                //path with the least possible turns
                if(Math.sqrt(Math.pow(adjX-currentBlock.x,2)+Math.pow(adjY-currentBlock.y,2)) > 1){
                    unusable = true;
                }

                if(!unusable){//only process this is the block is not unusable
                    boolean okayToParent=true;
                    //we can not create a new block with us as the parent if it violates any of
                    //the constraints like horizontal and vertical only constraints.
                    if(currentBlock.verticalOnly){
                        if(!currentBlock.isVertical(getBlock(adjX,adjY))){
                            okayToParent = false;
                        }
                    }
                    if(currentBlock.horizontalOnly){
                        if(!currentBlock.isHorizontal(getBlock(adjX,adjY))){
                            okayToParent = false;
                        }
                    }
                    double newMovementCost = p_Block.calc_g_score(currentBlock,getBlock(adjX,adjY));
                    if((newMovementCost < getBlock(adjX,adjY).g_score || getBlock(adjX,adjY).m_parent == null) && okayToParent){
                        //setting the parent will also calculate the g_cost and f_cost
                        //it will also mark this block as open
                        getBlock(adjX,adjY).setParent(currentBlock);
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


                if(thisBlock.f_score < lowestFScore && isOkay){
                    lowestFScore = thisBlock.f_score;
                    lowestFBlock = thisBlock;
                }

            }

        }
        currentBlock = lowestFBlock;
    }
}
