#include <ftc_vision_OpencvNativeClass.h>
#include <vector>
#include <string>
#include <sstream>
#include "opencv2/opencv.hpp"
#include "p_Block.h"


JNIEXPORT void JNICALL Java_ftc_vision_OpencvNativeClass_findBalls
(JNIEnv * env, jclass jcl, jlong addrRgba, jlong addrDrawingImage,jboolean blueside){
Mat& mRgb = *(Mat*)addrRgba;
Mat& mDrawingImage = *(Mat*)addrDrawingImage;


bool mblueside = (bool) blueside;

KeyPoint bestBall = findBalls(mRgb,mDrawingImage,mblueside);


//findLines(mRgb,mDrawingImage);


Point2f p = bestBall.pt;

jfieldID fid = env->GetStaticFieldID(jcl,"bestBallX","I");
env->SetStaticIntField(jcl, fid, p.x);

jfieldID fid1 = env->GetStaticFieldID(jcl,"bestBallY","I");
env->SetStaticIntField(jcl, fid1, p.y);

jfieldID fid2 = env->GetStaticFieldID(jcl,"bestBallRadius","F");
env->SetStaticFloatField(jcl, fid2,bestBall.size/2);


//update the best ball absolute positions in the native class//
//jfieldID fid3 = env->GetStaticFieldID(jcl,"bestBallXAbs","F");
//env->SetStaticFloatField(jcl, fid3,bestBallXAbs);
//jfieldID fid4 = env->GetStaticFieldID(jcl,"bestBallYAbs","F");
//env->SetStaticFloatField(jcl, fid4,bestBallYAbs);


}
void findLines(Mat rawImage, Mat &drawingImage){
    //apply an rgb filter to get all yellow pixels
    //Mat colorBooleanImage;
    //229,299,6
    cvtColor(rawImage,rawImage,COLOR_RGBA2RGB);
    cvtColor(rawImage,rawImage,COLOR_RGB2HSV);

    //hue = 26
    cv::inRange(rawImage,Scalar(16,200,200),Scalar(36,255,255),drawingImage);
    cvtColor(drawingImage,drawingImage,COLOR_GRAY2RGB);



    //start the lowest y at 0
    int lowestYellowY = 0;
    int lowestYellowX = 0;
    for(int x = 0; x < rawImage.cols;x++){
        for(int y = 0; y < rawImage.rows; y++){

            //if this pixel is yellow (true) and it is lower than the lowest Yellow
            if(drawingImage.at<Vec3b>(Point(x, y))[0] != 0 && y>lowestYellowY){
                lowestYellowX = x;
                lowestYellowY = y;
            }

        }
    }
    circle(drawingImage,Point(lowestYellowX,lowestYellowY),10,Scalar(255,0,0),-1);


    /*
    vector<Vec4i> lines;
    HoughLinesP(drawingImage, lines, 1, CV_PI/180, 50, 40, 5 );

    //convert the boolean image to RGB so that we can draw in color the lines
    cvtColor(drawingImage,drawingImage,COLOR_GRAY2RGB);

    //loop through all the lines and draw them
    for( int i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        line( drawingImage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
    }
     */
}

KeyPoint findBalls(Mat img, Mat &drawingImage,bool blueside) {

    //let's not care about the alpha channel
    cvtColor(img,img,COLOR_RGBA2RGB);
    Mat originalCameraImage = img.clone();
    //Save the image in the case that we need to calibrate
    currentImage.release();
    currentImage = img.clone();// for the calibration code's access


    resize(img, img, Size(), 0.5, 0.5, INTER_CUBIC);
    blur(img,img,Size(3,3));


    Mat booleanImage;
    Mat hsv;
    cv::cvtColor(img,hsv,COLOR_RGB2HSV);

    if(blueside){
        //Convert hsv to a boolean image based on if the color meets the hsv requirements or not
        cv::inRange(hsv,Scalar(avH-20,100,30),Scalar(avH+20,255,255),booleanImage);
    }else{
        Mat hsv1;
        Mat hsv2;

        //Convert hsv to a boolean image based on if the color meets the hsv requirements or not
        cv::inRange(hsv,Scalar(0,150,20),Scalar(5,255,255),hsv1);
        cv::inRange(hsv,Scalar(172,150,20),Scalar(179,255,255),hsv2);
        //we need to do an or to apply equal margin on both sides
        //because red is 0 on the HSV color wheel
        bitwise_or(hsv1,hsv2,booleanImage);

    }

    drawingImage.release();
    drawingImage = booleanImage.clone();
    cvtColor(drawingImage,drawingImage,COLOR_GRAY2RGB);

    /*/////display the calibrated/uncalibrated average h s and v values///////////*/
    char str[200];
    sprintf(str,"Hav: %d Sav: %d Vav: %d",avH, avS, avV);

    putText(drawingImage, str   ,
            Point(0,50),1,1,Scalar(0,255,0,1));
    /*////////////////////////////////////////////////////////////////////////////*/



    ////////////////////////////////////Detect Blobs//////////////////////////////
    // Setup SimpleBlobDetector parameters.
    SimpleBlobDetector::Params params;
    params.filterByColor = true;
    //this is a binary image, so only look at pixels with a 255 value.
    params.blobColor = 255;
    //Lets define a reasonable min and max area, since balls will never be giant, and will
    //never be only 1 pixel big.
    params.filterByArea = true;
    params.minArea = 3*3;
    params.maxArea =75*75;

    params.filterByCircularity = true;
    params.filterByConvexity = false;
    params.filterByInertia = false;
    //how much the blobs must look like circles -> we can have a high tollerance because
    //we know the position of balls on the floor, so we will use their radius to confirm
    //that they are at the predicted distance.
    params.minCircularity = 0.6;


    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
    // SimpleBlobDetector::create creates a smart pointer.
    // So you need to use arrow ( ->) instead of dot ( . )
    std::vector<KeyPoint> keypoints;
    detector->detect( booleanImage, keypoints);
    //////////////////////////////////////////////////////////////////////////////

    //scale the image back up after doing the blob detection
    resize(drawingImage, drawingImage, Size(), 2, 2, INTER_CUBIC);








    //////////////////Overlay the Original Image for half of the screen///////////////////
    //First, crop out what we want to overlay. It will be on the right half of the screen
    //Top left: cols/2, 0 With: cols/2 Height: rows
    Mat originalCropped = originalCameraImage(
            Rect(originalCameraImage.cols/2 ,0,
                 originalCameraImage.cols/2,originalCameraImage.rows));
    //Now, copy what we have cropped onto the right half of the drawing image.
    originalCropped.copyTo(drawingImage(
            Rect(drawingImage.cols/2, 0,
                 drawingImage.cols/2, drawingImage.rows)));
    //////////////////////////////////////////////////////////////////////////////////////








    ////////////////////////Filter Out Unqualified Blobs and Display////////////////////////////////
    //store a variable that keeps track of the ball with the largest y position
    //since y position is inverted (top is 0), we will look for the closest ball
    int lowestBallY = 0;
    KeyPoint bestBallKeyPoint = KeyPoint(0,0,0);
    for(int i = 0; i<keypoints.size();i++){
        //multiply all the values by 2 since we scaled the image down initially
        keypoints[i].pt.x *=2;
        keypoints[i].pt.y *=2;
        keypoints[i].size *=2;

        //Calculate the ball relative y position in field coords so that we can see if it's radius
        //is reasonable for it's distance. Otherwise, we will discard the blob.
        float ballYPosAbs = 182417400 + ((89.93381 - 182417400)/(1 + pow(((480-keypoints[i].pt.y)/4440.877),5.265578)));
        float predictedSize = 5.455348 + ((60873630 - 5.455348)/(1 + pow((ballYPosAbs/0.01534042),1.623841)));


        Point2f p = keypoints[i].pt;
        float currentBallRadius = keypoints[i].size;
        //only pass the values of this ball on if it's radius correlates to it's size correctly, otherwise,
        //just display purple
        if(predictedSize/(keypoints[i].size/2) < 1.7 && predictedSize/(keypoints[i].size/2) > 0.8){
            //If this keypoint is lower (greater y) than the last, mark it as the lowest
            if(keypoints[i].pt.y > lowestBallY){
                lowestBallY = keypoints[i].pt.y;
                bestBallKeyPoint= keypoints[i];
            }
            //draw a green rectangle around the good ball to let the user know which ball it qualifies
            rectangle(drawingImage,Point(p.x - (keypoints[i].size/2.0f),p.y - (keypoints[i].size/2.0f)),
                      Point(p.x + (keypoints[i].size/2.0f),p.y+(keypoints[i].size/2.0f)),
                      Scalar(0,255,10,1),3);
        }else{
            //if the ball doesn't qualify, only draw an orange box around it to let the user know
            rectangle(drawingImage,Point(p.x - (currentBallRadius/2.0f),p.y - (currentBallRadius/2.0f)),
                      Point(p.x + (currentBallRadius/2.0f),p.y+(currentBallRadius/2.0f)),
                      Scalar(255,125,0,1),3);
        }

        bestBallX = bestBallKeyPoint.pt.x;
        bestBallY = bestBallKeyPoint.pt.y;
        bestBallRadius = bestBallKeyPoint.size;
    }
    //draw a fixed size target rectangle that is smaller than the green one so that it is
    //obvious that this ball is the chosen ball
    double sizeRect = 20;
    //draw a red box around the ball the it chose
    rectangle(drawingImage, Point(bestBallX- sizeRect, bestBallY - sizeRect),
              Point(bestBallX + sizeRect,bestBallY + sizeRect),
              Scalar(255,0,0,1),3);
    ////////////////////////////////////////////////////////////////////////////////////////////////






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




