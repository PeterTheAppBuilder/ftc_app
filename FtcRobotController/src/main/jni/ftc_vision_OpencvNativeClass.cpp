#include <ftc_vision_OpencvNativeClass.h>
#include "Blob.h"
#include <vector>
#include <string>
#include <sstream>
#include "opencv2/opencv.hpp"

JNIEXPORT void JNICALL Java_ftc_vision_OpencvNativeClass_convertGray
(JNIEnv * env, jclass jcl, jlong addrRgba, jlong addrDrawingImage,jboolean blueside){
Mat& mRgb = *(Mat*)addrRgba;
Mat& mDrawingImage = *(Mat*)addrDrawingImage;


bool mblueside = (bool) blueside;


KeyPoint bestBall = toGray(mRgb,mDrawingImage,mblueside);

Point2f p = bestBall.pt;

jfieldID fid = env->GetStaticFieldID(jcl,"bestBallX","I");
env->SetStaticIntField(jcl, fid, p.x);

jfieldID fid1 = env->GetStaticFieldID(jcl,"bestBallY","I");
env->SetStaticIntField(jcl, fid1, p.y);



jfieldID fid2 = env->GetStaticFieldID(jcl,"bestBallRadius","F");
env->SetStaticFloatField(jcl, fid2,bestBall.size/2);


}





KeyPoint toGray(Mat img, Mat &drawingImage,bool blueside) {
    //let's not care about the alpha channel
    cvtColor(img,img,COLOR_RGBA2RGB);
    //Save the image in the case that we need to calibrate
    currentImage.release();
    currentImage = img.clone();// for the calibration code's access


    blur(img,img,Size(2,2));
    resize(img, img, Size(), 0.5, 0.5, INTER_CUBIC);
    blur(img,img,Size(2,2));







    Mat booleanImage;
    Mat hsv;
    cv::cvtColor(img,hsv,COLOR_RGB2HSV);

    if(blueside){
        //Convert hsv to a boolean image based on if the color meets the hsv requirements or not
        cv::inRange(hsv,Scalar(avH-20,75,30),Scalar(avH+20,255,255),booleanImage);
    }else{
        Mat hsv1;
        Mat hsv2;

        //Convert hsv to a boolean image based on if the color meets the hsv requirements or not
        cv::inRange(hsv,Scalar(0,150,20),Scalar(5,255,255),hsv1);
        cv::inRange(hsv,Scalar(172,150,20),Scalar(179,255,255),hsv2);
        bitwise_or(hsv1,hsv2,booleanImage);

    }



    //bitwise_and(booleanImageRgb,booleanImage,booleanImage);

    drawingImage.release();
    drawingImage = booleanImage.clone();
    cvtColor(drawingImage,drawingImage,COLOR_GRAY2RGB);

    char str[200];
    sprintf(str,"Hav: %d Sav: %d Vav: %d",avH, avS, avV);

    putText(drawingImage, str   ,
            Point(0,50),1,1,Scalar(0,255,0,1));





    // Setup SimpleBlobDetector parameters.
    SimpleBlobDetector::Params params;

    // Change thresholds
    //params.minThreshold = 10;
    //params.maxThreshold = 200;

    params.filterByColor = true;
    params.blobColor = 255;

    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 5*5;
    params.maxArea =75*75;

    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.6;

    // Filter by Convexity
    params.filterByConvexity = false;

    // Filter by Inertia
    params.filterByInertia = false;


    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

    // SimpleBlobDetector::create creates a smart pointer.
    // So you need to use arrow ( ->) instead of dot ( . )
    std::vector<KeyPoint> keypoints;
    detector->detect( booleanImage, keypoints);

    resize(drawingImage, drawingImage, Size(), 2, 2, INTER_CUBIC);



    //store a variable that keeps track of the ball with the largest y position
    //since y position is inversed (top is 0), we will look for the closest ball
    int lowestBallY = 0;
    KeyPoint bestBallKeyPoint = KeyPoint(0,0,0);


    for(int i = 0; i<keypoints.size();i++){
        keypoints[i].pt.x *=2;
        keypoints[i].pt.y *=2;
        keypoints[i].size *=2;
        float ballYPosAbs = 24.24642f + ((53715410 - 24.24642)/(1 + pow((keypoints[i].pt.y/0.0006110034f),1.019436)));

        float predictedSize = 5.455348 + ((60873630 - 5.455348)/(1 + pow((ballYPosAbs/0.01534042),1.623841)));

        Point2f p = keypoints[i].pt;
        float currentBallRadius = keypoints[i].size;
        //only pass the values of this ball on if it's radius correlates to it's size correctly, otherwise,
        //just display purple
        if(predictedSize/(keypoints[i].size/2) < 1.7 && predictedSize/(keypoints[i].size/2) > 0.3){
            bestBallX = p.x;
            bestBallY = p.y;
            bestBallRadius = keypoints[i].size;

            if(keypoints[i].pt.y > lowestBallY){
                lowestBallY = keypoints[i].pt.y;
                bestBallKeyPoint= keypoints[i];
            }
            rectangle(drawingImage,Point(p.x - (bestBallRadius/2.0f),p.y - (bestBallRadius/2.0f)),
                      Point(p.x + (bestBallRadius/2.0f),p.y+(bestBallRadius/2.0f)),
                      Scalar(0,255,10,1),3);
        }else{


            rectangle(drawingImage,Point(p.x - (currentBallRadius/2.0f),p.y - (currentBallRadius/2.0f)),
                      Point(p.x + (currentBallRadius/2.0f),p.y+(currentBallRadius/2.0f)),
                      Scalar(255,125,0,1),3);
        }

    }



    return bestBallKeyPoint;

}


JNIEXPORT void JNICALL Java_ftc_vision_OpencvNativeClass_calibrate
(JNIEnv * env, jclass jcl){
calibrate();
}

void calibrate(){


    Rect ballROI = Rect(bestBallX - (int) (bestBallRadius/2),
                        bestBallY- (int) (bestBallRadius/2),
                        (int) bestBallRadius,(int) bestBallRadius);
    Mat croppedBall = currentImage(ballROI);
    Mat croppedBallHSV;
    cvtColor(croppedBall,croppedBallHSV,COLOR_RGB2HSV);




    //Create a mask the same size of the ball and draw a black circle in the middle with everything else white
    Mat circleMask;
    //we can only have a 1 channel mask, but copy the image over with the size
    cvtColor(croppedBall,circleMask,COLOR_RGB2GRAY);

    circleMask.setTo(Scalar(0));
    circle(circleMask,Point(circleMask.cols/2,circleMask.rows/2),(int) circleMask.rows/2,Scalar(255),-1);








    long totalH = 0;
    long totalS = 0;
    long totalV = 0;
    long totalPixels = 0;
    for(int x = 0; x < croppedBall.rows;x++){
        for(int y = 0; y< croppedBall.cols; y++){
            if(circleMask.at<Vec3b>(y,x)[0] != 0){//only compare this pixel if it is inside the circle
                double thisPixelH = croppedBallHSV.at<Vec3b>(y,x)[0];
                double thisPixelS = croppedBallHSV.at<Vec3b>(y,x)[1];
                double thisPixelV = croppedBallHSV.at<Vec3b>(y,x)[2];
                totalH+=thisPixelH;
                totalS+= thisPixelS;
                totalV+= thisPixelV;
                totalPixels++;
            }
        }
    }
    //divide the total values by the number of pixels to get the average
    avH = totalH/totalPixels;
    avS = totalS/totalPixels;
    avV = totalV/totalPixels;

}
