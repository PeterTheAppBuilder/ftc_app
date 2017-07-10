package ftc.pathfinder;

import org.opencv.core.Point;

/**
 * Created by Administrator on 7/7/2017.
 */

public class stepPoint {
    public int x;
    public int y;
    public Point p;
    public boolean mandatory = false;
    public stepPoint(int X, int Y, boolean mandatory){
        x = X;
        y = Y;
        p = new Point(X,Y);
        this.mandatory = mandatory;
    }
}
