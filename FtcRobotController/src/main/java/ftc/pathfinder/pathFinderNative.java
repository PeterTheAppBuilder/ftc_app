package ftc.pathfinder;

import android.os.AsyncTask;
import android.os.SystemClock;
import android.util.Log;

import org.opencv.core.*;

import java.util.ArrayList;


import static org.opencv.core.CvType.CV_8UC4;
import static org.opencv.imgproc.Imgproc.circle;


public class pathFinderNative extends AsyncTask<Double,Double,Double > {
    public static int blockposx;
    public static int blockposy;



    public static Mat fieldMap;
    public static ArrayList<Point> m_filteredSteps;

    public static boolean upToDate = true;


    public native static float[] calcPath(double my_X, double my_Y, double target_X, double target_Y,
                                       double fieldSize,long fieldMap, boolean newPath);

    public native static void drawPath(long drawingImage, int scale);


    @Override
    protected Double doInBackground(Double... params) {
        //if params[5] is , then it means a new path
        boolean newPath = params[5] == 1;
        long start = SystemClock.uptimeMillis();
        float[] coords = calcPath(params[0],params[1],params[2],params[3],params[4],
                fieldMap.getNativeObjAddr(),newPath);
        Log.d("LOG_TAG","time: " + Integer.toString((int) (SystemClock.uptimeMillis()-start)));
        //if the pathfinder fails, we don't want to update m_filteredSteps
        if(coords.length != 0){
            ArrayList<Point> filteredSteps = new ArrayList<>();
            for(int i = 0; i < coords.length/2; i++){
                filteredSteps.add(new Point(coords[i*2],coords[(i*2)+1]));
            }
            m_filteredSteps = filteredSteps;
            upToDate = false;
        }

        return null;
    }
}
