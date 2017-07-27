package ftc.pathfinder;

import android.os.AsyncTask;
import android.os.SystemClock;
import android.provider.Settings;
import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

import ftc.vision.FrameGrabber;

import static org.opencv.imgproc.Imgproc.line;

/**
 * Created by Administrator on 7/3/2017.
 */

public class pathFinder extends AsyncTask<Double,Double,ArrayList<Point> > {
    public static grid m_grid;
    public static Mat fieldMap;
    public static void createGrid(){
        m_grid = new grid(fieldMap.cols(),fieldMap.rows());
    }

    public static ArrayList<Point> m_filteredSteps = null;

    public static double[] obstacle = {255,255,255};
    //these color values specify if the robot can only move in one direction while over them
    //that way it will not approach wall balls from angles that the collector can't handle
    public static double[] allowsOnlyVertical = {255,0,0};
    public static double[] allowsOnlyHorizontal = {0,0,255};
    public static double[] penalty1 = {0,255,0};



    public static double[] posToBlock(double x, double y, double fieldSize){
        //calculate the positions as percentages of the total field size
        double x_percent = x/fieldSize;
        //the top left of our map is our origin, however, our input assumes
        //that the origin is the bottom left. Therefore, we must invert the y
        double y_percent = 1-(y/fieldSize);
        //scale the percentages to the size of our map
        x_percent *= fieldMap.cols();
        y_percent *= fieldMap.rows();

        //cast the scaled positions to the nearest block because our
        //pathfinder can't compute fractions of blocks
        double[] finalBlockPos = {(int) x_percent, (int) y_percent,x_percent, y_percent};
        return finalBlockPos;

    }

    public static boolean calcPath(double my_X, double my_Y, double target_X, double target_Y,double fieldSize){
        long timeStart = SystemClock.uptimeMillis();

        //convert our pos to path-finding coordinates
        double[] my_pos_block = posToBlock(my_X,my_Y,fieldSize);
        double[] target_pos_block = posToBlock(target_X, target_Y,fieldSize);

        m_grid.setTarget((int) target_pos_block[0],(int) target_pos_block[1]);

        //loop through all of this pixels which are coded with different colors that apply different
        //constraints
        //for example, obstacles are coded by the color white

        for(int r = 0; r < fieldMap.cols();r++){
            for(int c = 0; c < fieldMap.rows();c++){
                double[] thisPixel = fieldMap.get(c,r);
                if(Arrays.equals(thisPixel,obstacle)){
                    m_grid.setObstacle(r,c);
                }
                if(Arrays.equals(thisPixel,allowsOnlyHorizontal)){
                    m_grid.setOnlyHorizontal(r,c);
                }
                if(Arrays.equals(thisPixel,allowsOnlyVertical)){
                    m_grid.setOnlyVertical(r,c);
                }
                if(Arrays.equals(thisPixel,penalty1)){
                    m_grid.getBlock(r,c).g_score = 1.5;
                }
            }
        }

        //if we are seated on an invalid block, abort because there is no way out
        //or if our target is on an invalid block
        if(m_grid.getBlock((int) my_pos_block[0],(int) my_pos_block[1]).invalid ||
                m_grid.getBlock((int) target_pos_block[0],(int) target_pos_block[1]).invalid){
            return false;
        }

        m_grid.findPath((int) my_pos_block[0],(int) my_pos_block[1]);
        ArrayList<stepPoint> pointSteps =  m_grid.m_steps;
        ArrayList<Point> filteredSteps = new ArrayList<>();

        //start at the first point, since it is in reverse order
        stepPoint currentPoint = pointSteps.get(0);
        boolean finished = false;


        int currentMandatoryPointIndex = pointSteps.size()+10;
        for(int i =0;i<pointSteps.size();i++){
            if(pointSteps.get(i).mandatory){
                currentMandatoryPointIndex = i;
                break;
            }
        }

        for(;;){
            //go in backwards order through the ArrayList of points
            for(int i=pointSteps.size()-1; i>=0;i--){
                int obsticalCountBefore = countObsticals(fieldMap);
                Mat collisionLineMat = fieldMap.clone();
                //draw a line from the current point to the finish
                //if it cuts through any obsticals, there will be less
                //obstical pixels after drawing a line from start to finish
                line(collisionLineMat,currentPoint.p,pointSteps.get(i).p,new Scalar(0,0,0));
                int obsticalCountAfter = countObsticals(collisionLineMat);
                collisionLineMat.release();
                if(obsticalCountBefore == obsticalCountAfter && i <= currentMandatoryPointIndex){
                    if(i==currentMandatoryPointIndex){
                        currentMandatoryPointIndex = pointSteps.size()+10;
                        for(int j =i+1;j<pointSteps.size();j++){
                            if(pointSteps.get(j) != null){
                                if(pointSteps.get(j).mandatory){
                                    currentMandatoryPointIndex = j;
                                }
                            }
                        }
                    }


                    //if we can get to this point in a straight line
                    //without going through any obsticals, add it to the step list
                    currentPoint = pointSteps.get(i);
                    filteredSteps.add(pointSteps.get(i).p);
                    //if we have gotten to the finish line
                    if(i == pointSteps.size()-1){
                        finished = true;
                    }
                    break;
                }
            }
            if(finished){break;}
        }



        FrameGrabber.findTime = SystemClock.uptimeMillis()-timeStart;




        m_filteredSteps = filteredSteps;
        return true;
    }
    public static int countObsticals(Mat map){
        int obsticals = 0;
        for(int r = 0; r < map.cols();r++){
            for(int c = 0; c < map.rows();c++){
                double[] thisPixel = map.get(c,r);
                if(Arrays.equals(thisPixel,obstacle)){
                    obsticals++;
                }
            }
        }
        return obsticals;
    }

    public static boolean upToDate = true;
    @Override
    protected ArrayList<Point> doInBackground(Double... params) {
        if(calcPath(params[0],params[1],params[2],params[3],params[4])){
            upToDate = false;
        }
        return null;
    }
    public static void test(){
        long start = SystemClock.uptimeMillis();
        //pathFinderNative.calcPath(6,0,16,24,25,fieldMap.getNativeObjAddr());
        Log.d("LOG_TAG","time: " + Integer.toString((int) (SystemClock.uptimeMillis()-start)));
    }

}
