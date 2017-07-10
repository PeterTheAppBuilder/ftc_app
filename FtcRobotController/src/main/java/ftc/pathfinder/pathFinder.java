package ftc.pathfinder;

import android.os.SystemClock;

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

public class pathFinder {
    public static grid m_grid;
    public static Mat fieldMap;
    public static void createGrid(){
        m_grid = new grid(fieldMap.cols(),fieldMap.rows());
    }

    public static ArrayList<Point> m_filteredSteps;

    public static double[] obstacle = {255,255,255};
    //these color values specify if the robot can only move in one direction while over them
    //that way it will not approach wall balls from angles that the collector can't handle
    public static double[] allowsOnlyVertical = {255,0,0};
    public static double[] allowsOnlyHorizontal = {0,0,255};
    public static ArrayList<Point> calcPath(int my_X, int my_Y, int target_X, int target_Y){


        long timeStart = SystemClock.uptimeMillis();


        target_Y = 24-target_Y;
        my_Y = 24-my_Y;
        m_grid.setTarget(target_X,target_Y);



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

            }
        }
        m_grid.findPath(my_X,my_Y);
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
        return filteredSteps;
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

}
