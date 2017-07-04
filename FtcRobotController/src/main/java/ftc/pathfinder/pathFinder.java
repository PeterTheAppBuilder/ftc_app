package ftc.pathfinder;

import android.os.SystemClock;

import org.opencv.core.Mat;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

import ftc.vision.FrameGrabber;

/**
 * Created by Administrator on 7/3/2017.
 */

public class pathFinder {
    public static grid m_grid;
    public static Mat fieldMap;
    public static void createGrid(){
        m_grid = new grid(fieldMap.cols(),fieldMap.rows());
    }

    public static void calcPathStep(){
        m_grid.calcPathStep();
    }
    public static void calcPathStart(int my_X, int my_Y, int target_X, int target_Y){
        m_grid.setTarget(target_X,target_Y);



        //loop through all of this pixels which are coded with different colors that apply different
        //constraints
        //for example, obstacles are coded by the color white
        double[] obstacle = {255,255,255};

        //these color values specify if the robot can only move in one direction while over them
        //that way it will not approach wall balls from angles that the collector can't handle
        double[] allowsOnlyVertical = {255,0,0};
        double[] allowsOnlyHorizontal = {0,0,255};

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
        m_grid.calcStart(my_X,my_Y);
    }

    public static ArrayList<Point> m_filteredSteps;

    public static ArrayList<Point> calcPath(int my_X, int my_Y, int target_X, int target_Y){

        long timeStart = SystemClock.uptimeMillis();

        m_grid.setTarget(target_X,target_Y);



        //loop through all of this pixels which are coded with different colors that apply different
        //constraints
        //for example, obstacles are coded by the color white
        double[] obstacle = {255,255,255};

        //these color values specify if the robot can only move in one direction while over them
        //that way it will not approach wall balls from angles that the collector can't handle
        double[] allowsOnlyVertical = {255,0,0};
        double[] allowsOnlyHorizontal = {0,0,255};

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
        ArrayList<Point> pointSteps =  m_grid.m_steps;
        ArrayList<Point> filteredSteps = new ArrayList<>();
        //go through all of the points in the 2d point array and filter them into key points
        //that way we only worry about the points that represent changes in direction, and
        //not the ones that are co-linear.
        Point lastPoint = new Point(-1,-1);
        double lastDirection = -0.1;
        double currentDirection;
        for(Point p: pointSteps){
            //compare the current slope between the solved points and see if it has changed,
            //meaning that the direction of the path has changed. If so, mark it. Otherwise,
            //do nothing because the movement is the same as before.

            //this would cause a divide by zero in the slope calculation
            //so set it to a constant high number. Otherwise, calculate
            //the slope normally
            if(p.x-lastPoint.x == 0.0){
                currentDirection = 999;
            }else{
                currentDirection = (p.y-lastPoint.y)/(p.x-lastPoint.x);
            }
            if(currentDirection != lastDirection){
                filteredSteps.add(lastPoint);
            }


            lastPoint = p;
            lastDirection = currentDirection;

        }
        //the step list may not include the final point, so add it to the
        //list of filtered points if it didn't already
        if(!filteredSteps.contains(new Point(target_X,target_Y))){
            filteredSteps.add(new Point(target_X,target_Y));
        }
        m_filteredSteps = filteredSteps;

        FrameGrabber.findTime = SystemClock.uptimeMillis()-timeStart;
        return filteredSteps;
    }

}
