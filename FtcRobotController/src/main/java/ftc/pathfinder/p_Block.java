package ftc.pathfinder;

/**
 * Created by Administrator on 6/30/2017.
 */

public class p_Block {
    public int x;
    public int y;
    boolean isObstical = false;


    public boolean open = false;
    public boolean closed = false;
    public boolean invalid = false;

    public boolean horizontalOnly=false;
    public boolean verticalOnly=false;





    //the h_score represents a score given to each block that increases based on how
    //many steps away it is from the target. It is the sum of the difference in x values
    //and the difference in y. This way of calculating distance is known as the Manhattan method.
    //All values in this variable are positive.
    public int h_score = -1;

    //the g_score represents a score given to parented blocks that represents how
    //much it will cost to move there. For example, blocks that are diagonal from
    //a parent block cost more than ones that are straight up or down.
    public double g_score = -1.0;

    //this is a predefined movement penalty set by the field map.
    public double pre_g_score = 0;


    public double f_score = -1.0;//the sum of our g and h score

    //initialize ourselves by setting our x and y pos
    public p_Block(int x, int y, int target_x, int target_y){
        this.x = x;
        this.y = y;

        //calculate the h_score using the Manhattan method where scores increase based on their
        //sum of the delta x and y positions
        h_score = Math.abs(target_x-x)+Math.abs(target_y-y);
    }
    public p_Block m_parent = null;

    public void setParent(p_Block parent){
        m_parent = parent;
        g_score = calc_g_score(parent,this);
        open = true;
        if(h_score != -1){//if our h score has been set, we can calculate our f_score
            f_score = h_score+g_score;
        }
    }

    public static double calc_g_score(p_Block parentBlock, p_Block block2){
        //first calculate the cost of moving from the parent to child
        double g_score_relative = Math.sqrt(Math.pow(block2.x-parentBlock.x,2)
                +Math.pow(block2.y-parentBlock.y,2));
        //add the parents current g_score
        double g_score_sum = parentBlock.g_score + g_score_relative;

        //the parent of the parent could be null if the parent of this block is the starting block
        if(parentBlock.m_parent != null && parentBlock != null){
            double slopeBefore;
            if(parentBlock.m_parent.x-parentBlock.x!=0){
                slopeBefore = (parentBlock.m_parent.y-parentBlock.y)/
                        (parentBlock.m_parent.x-parentBlock.x);
            }else{
                //if there is a divide by zero for the slope, set it to a really big number, but
                //it doesn't really matter
                slopeBefore = 999;
            }
            double slopeAfter;
            if(parentBlock.x-block2.x!=0){
                slopeAfter = (parentBlock.y - block2.y)/(parentBlock.x-block2.x);
            }else{
                slopeAfter = 999;
            }

            if(slopeAfter!=slopeBefore){
                g_score_sum += 1.9;
            }
        }
        g_score_sum += block2.pre_g_score;


        return g_score_sum;
    }

    public boolean isCutCorner(p_Block otherBlock,p_Block cornerBlock){
        if(otherBlock.x==this.x+1 && otherBlock.y==this.y+1 && cornerBlock.x==this.x+1 && cornerBlock.y == this.y){
            return true;
        }
        if(otherBlock.x==this.x+1 && otherBlock.y==this.y-1 && cornerBlock.x==this.x+1 && cornerBlock.y == this.y){
            return true;
        }
        if(otherBlock.x==this.x-1 && otherBlock.y==this.y+1 && cornerBlock.x==this.x && cornerBlock.y == this.y+1){
            return true;
        }
        if(otherBlock.x==this.x+1 && otherBlock.y==this.y+1 && cornerBlock.x==this.x && cornerBlock.y == this.y+1){
            return true;
        }
        if(otherBlock.x==this.x-1 && otherBlock.y==this.y-1 && cornerBlock.x==this.x-1 && cornerBlock.y == this.y){
            return true;
        }
        if(otherBlock.x==this.x-1 && otherBlock.y==this.y+1 && cornerBlock.x==this.x-1 && cornerBlock.y == this.y){
            return true;
        }
        if(otherBlock.x==this.x+1 && otherBlock.y==this.y-1 && cornerBlock.x==this.x && cornerBlock.y == this.y-1){
            return true;
        }
        if(otherBlock.x==this.x-1 && otherBlock.y==this.y-1 && cornerBlock.x==this.x && cornerBlock.y == this.y-1){
            return true;
        }

        return false;
    }



    public boolean isHorizontal(p_Block otherBlock){
        if(otherBlock.y == this.y && Math.abs(this.x-otherBlock.x) == 1){
            return true;
        }
        return false;
    }
    public boolean isVertical(p_Block otherBlock){
        if(otherBlock.x == this.x && Math.abs(this.y-otherBlock.y) == 1){
            return true;
        }
        return false;
    }



    public void setObstical(boolean o){
        isObstical=o;
    }
}
