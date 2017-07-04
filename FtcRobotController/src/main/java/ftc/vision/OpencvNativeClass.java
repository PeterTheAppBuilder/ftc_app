package ftc.vision;

import org.opencv.core.Point;

/**
 * Created by Administrator on 6/4/2017.
 */

public class OpencvNativeClass {
    static int bestBallX = 0;
    static int bestBallY = 0;
    static float bestBallRadius = 0;
    public native static void convertGray(long matAddrRgba, long matAddr,boolean blueside);
    public native static void calibrate();


}
