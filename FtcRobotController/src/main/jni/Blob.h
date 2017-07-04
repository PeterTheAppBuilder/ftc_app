

#ifndef _Blob_h_
#define _Blob_h_

class Blob{
public:
    int X;
    int Y;
    float width;
    float height;
    int centerX;
    int centerY;
    float pixelsInside = 0;
    float similarityPercentage = 0.0f;

    Blob(int x,int y);

    void setSize(int, int);

    float distanceTo(int, int);

    void setSimilarityPercentage(float);

    float getDensity();

    void addPoint(int,int);
};

#endif