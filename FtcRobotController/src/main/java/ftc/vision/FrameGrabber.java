package ftc.vision;

import android.os.SystemClock;
import android.util.Log;
import android.view.SurfaceView;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import ftc.pathfinder.p_Block;
import ftc.pathfinder.pathFinder;
import ftc.pathfinder.pathFinderNative;

import static org.opencv.core.CvType.CV_8UC4;
import static org.opencv.imgproc.Imgproc.circle;
import static org.opencv.imgproc.Imgproc.cvtColor;
import static org.opencv.imgproc.Imgproc.line;
import static org.opencv.imgproc.Imgproc.putText;
import static org.opencv.imgproc.Imgproc.rectangle;

/**
 * Created by Administrator on 6/3/2017.
 */

public class FrameGrabber implements
        CameraBridgeViewBase.CvCameraViewListener2 {
    @Override
    public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat(height,width, CV_8UC4);
        drawingImage = new Mat(height,width,CV_8UC4);
    }

    @Override
    public void onCameraViewStopped() {

    }

    static cameraSettings mcameraSettings;


    public FrameGrabber(cameraSettings c) {
        c.setVisibility(SurfaceView.VISIBLE);
        c.setCvCameraViewListener(this);
        c.turnOffAutoExposure();
        this.mcameraSettings = c;
    }









    public static boolean usingVision = false;




    public static Mat mRgba, drawingImage;



    //occurs when the calibrate button is pressed
    public static void calibrate(){
        OpencvNativeClass.calibrate();
        mcameraSettings.turnOffAutoExposure();
    }


    public float Square(float x){
        return x*x;
    }


    static{
        System.loadLibrary("MyOpencvLibs");
    }

    Mat blankMat;
    public static boolean blueside = true;

    public static long findTime = 0;


    public static double x = 0;
    public static double y =0;
    public static double bestBallX_abs = 0;
    public static double bestBallY_abs = 0;
    public static double step_x = 0;
    public static double step_y = 0;
    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {

        mRgba.release();
        mRgba = inputFrame.rgba();
        if(drawingImage != null){drawingImage.release();}




        ///////////////////////////////Find the balls in cpp//////////////////////////////////////////
        OpencvNativeClass.findBalls(mRgba.getNativeObjAddr(),drawingImage.getNativeObjAddr(),blueside);
        ////////////////////////////////////////////////////////////////////////////////////////////
        //let the pathfinder draw its path on us
        pathFinderNative.drawPath(drawingImage.getNativeObjAddr(),14);

        bestBallX = OpencvNativeClass.bestBallX;
        bestBallY = OpencvNativeClass.bestBallY;
        bestBallRadius = OpencvNativeClass.bestBallRadius;
        uptoDate = false;






        Point ball = new Point(((bestBallX_abs/fieldLength)*24)*20,((bestBallY_abs/fieldLength)*24)*20);
        rectangle(drawingImage,ball,new Point(ball.x+20,ball.y+20),new Scalar(255,255,0),-1);

        Point us = new Point((x/fieldLength)*24*20,((1-(y/fieldLength))*24)*20);
        circle(drawingImage,us,6,new Scalar(0,255,0),5);

        Point thisStep = new Point((step_x/fieldLength)*24*20,((1-(step_y/fieldLength))*24)*20);

        circle(drawingImage,thisStep,6,new Scalar(255,255,255),5);






        //putText(drawingImage,"X: " + bestBallX + " Y: " + bestBallY,new Point(50,50),1,5,new Scalar(0,255,0),2);



        //rectangle(drawingImage,new Point(bestBallX - (bestBallRadius/2.0f),bestBallY - (bestBallRadius/2.0f)),
        //        new Point(bestBallX + (bestBallRadius/2.0f),bestBallY+(bestBallRadius/2.0f)),
        //        new Scalar(255,0,0,1),5);

        return drawingImage;
    }
    public static boolean uptoDate = false;

    public static double fieldLength = 358.775;
    public static Point calcScreenPosI(Point fieldPos){
        Point percent = new Point(fieldPos.x/fieldLength,1-(fieldPos.y/fieldLength));
        percent = new Point((int) (percent.x * 24),(int) (percent.y * 24));
        return percent;
    }
    public static Point calcScreenPos(Point fieldPos){
        Point percent = new Point(fieldPos.x/fieldLength,1-(fieldPos.y/fieldLength));
        percent = new Point((percent.x * 24), (percent.y * 24));
        return percent;
    }
    public static Point multPoint(Point p, double x){
        return new Point(p.x*x,p.y*x);
    }
    public static Point addPoint(Point p, Point x){
        return new Point(p.x+x.x,p.y+x.y);
    }








    public static int bestBallX=-1;
    public static int bestBallY=-1;
    public static float bestBallRadius = -1.0f;




}
