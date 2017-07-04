#include "Blob.h"
#include <math.h>

void Blob::setSimilarityPercentage(float s) {
    similarityPercentage = s;
}

Blob::Blob(int x, int y){
    X = x;
    Y = y;
    centerX = X;
    centerY = Y;
}

void Blob::setSize(int Width, int Height) {
    width = Width;
    height = Height;
    centerX = X+(width/2);
    centerY = Y+(height/2);
}
float Blob::distanceTo(int x, int y) {
    return (float) sqrt(pow(x- centerX,2) + pow(y-centerY,2));
}
float Blob::getDensity() {
    return this->pixelsInside/((this->width)*(this->height));
}
void Blob::addPoint(int pointx, int pointy){
    if(pointx<X){
        width+=X-pointx;
        X = pointx;
    }
    if(pointy<Y){
        height+=Y-pointy;
        Y = pointy;
    }
    if(pointx>X+width){
        width= pointx-X;
    }
    if(pointy>Y+height){
        height= pointy-Y;
    }
    centerX = X+(width/2);
    centerY = Y+(height/2);
}