package ftc.vision;

import android.os.SystemClock;
import android.view.SurfaceView;

import com.qualcomm.ftcrobotcontroller.R;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;

import java.io.IOException;
import java.util.Arrays;

import ftc.pathfinder.grid;
import ftc.pathfinder.p_Block;
import ftc.pathfinder.pathFinder;

import static org.opencv.core.CvType.CV_8UC4;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2RGB;
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
        mGray = new Mat(height,width,CV_8UC4);
        drawingImage = new Mat(height,width, CV_8UC4);
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




    static Mat mRgba, drawingImage, mGray;



    //occurs when the calibrate button is pressed
    public static void calibrate(){
        OpencvNativeClass.calibrate();
        mcameraSettings.turnOffAutoExposure();
    }


    public float Square(float x){
        return x*x;
    }



    public final float blobDistanceThreshold = 200;
    public final float ScanningResolution = 16;

    static{
        System.loadLibrary("MyOpencvLibs");
    }

    public static String cameraReturnString = null;
    Mat blankMat;
    public static boolean blueside = true;

    public static long findTime = 0;


    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {



        mRgba.release();
        mRgba = inputFrame.rgba();




        OpencvNativeClass.convertGray(mRgba.getNativeObjAddr(),mGray.getNativeObjAddr(),blueside);
        bestBallX = OpencvNativeClass.bestBallX;
        bestBallY = OpencvNativeClass.bestBallY;
        bestBallRadius = OpencvNativeClass.bestBallRadius;
        uptoDate = false;






        rectangle(mGray,new Point(bestBallX - (bestBallRadius/2.0f),bestBallY - (bestBallRadius/2.0f)),
                new Point(bestBallX + (bestBallRadius/2.0f),bestBallY+(bestBallRadius/2.0f)),
                new Scalar(255,0,0,1),5);

        if(pathFinder.m_grid != null){
            int scale = 30;
            rectangle(mGray,new Point(0,0),new Point(pathFinder.m_grid.m_rows*scale,pathFinder.m_grid.m_cols*scale),
                    new Scalar(100,100,255),4);
            rectangle(mGray,new Point(pathFinder.m_grid.m_targetX*scale,pathFinder.m_grid.m_targetY*scale),
                    new Point((pathFinder.m_grid.m_targetX *scale)+scale,(pathFinder.m_grid.m_targetY*scale)+scale),
                    new Scalar(255,255,255),-1);
            for(p_Block iterBlock: pathFinder.m_grid.allBlocks){
                if(iterBlock.invalid){
                    rectangle(mGray,new Point(iterBlock.x*scale,iterBlock.y*scale),
                            new Point((iterBlock.x *scale)+scale,(iterBlock.y*scale)+scale),new Scalar(255,0,0),1);
                }
                if(iterBlock.open){
                    rectangle(mGray,new Point(iterBlock.x*scale,iterBlock.y*scale),
                            new Point((iterBlock.x *scale)+scale,(iterBlock.y*scale)+scale),new Scalar(0,255,0),1);
                }
                if(iterBlock.closed){
                    rectangle(mGray,new Point(iterBlock.x*scale,iterBlock.y*scale),
                            new Point((iterBlock.x *scale)+scale,(iterBlock.y*scale)+scale),new Scalar(193,40,236),-1);
                }
                if(iterBlock.m_parent != null && iterBlock.closed){
                    line(mGray,new Point(iterBlock.x*scale,iterBlock.y*scale),
                            new Point(iterBlock.m_parent.x*scale,iterBlock.m_parent.y*scale),new Scalar(255,255,0),3);
                }

                putText(mGray,Integer.toString((int) iterBlock.g_score),new Point(iterBlock.x*scale,iterBlock.y*scale),1,1,new Scalar(160,160,255));
                putText(mGray,Integer.toString((int) iterBlock.f_score),new Point(iterBlock.x*scale,(iterBlock.y*scale)+20),1,1,new Scalar(255,160,160));



            }

            if(pathFinder.m_filteredSteps != null){
                int i = 0;
                for(Point thisStepPoint: pathFinder.m_filteredSteps){
                    putText(mGray,Integer.toString(i),new Point(thisStepPoint.x*scale,thisStepPoint.y*scale),1,5,new Scalar(255,255,255));
                    i++;
                }
                putText(mGray,Long.toString(findTime),new Point(50,50),1,4,new Scalar(255,255,255));
            }

        }




        return mGray;

    }
    public static boolean uptoDate = false;









    public static int bestBallX=-1;
    public static int bestBallY=-1;
    public static float bestBallRadius = -1.0f;




}
