//
//  funcs.cpp
//  OpenCV
//
//  Created by 조현민 on 2020/05/21.
//  Copyright © 2020 Daniel Cho. All rights reserved.
//

#include "funcs.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

int quadratic_eq(vector<float> poly, int x)
{
    float c=poly.at(0);
    float b=poly.at(1);
    float a=poly.at(2);
    return (int)(a*x*x+b*x+c);
}

vector<float> polyfit(vector<Point2f>& pts){
    int size=(int)pts.size();
    Mat X(size,3,CV_32FC1);
    Mat Y(size,1, CV_32FC1);
    for (int i=0;i<size;i++)
    {
        float x=pts.at(i).x;
        X.at<float>(i,0)=1;
        X.at<float>(i,1)=x;
        X.at<float>(i,2)=x*x;
        Y.at<float>(i,0)=pts.at(i).y;
    }
    Mat result=(((X.t()*X).inv())*X.t())*Y;
    vector<float> result_vec(3);
    result_vec.at(0)=result.at<float>(0);
    result_vec.at(1)=result.at<float>(1);
    result_vec.at(2)=result.at<float>(2);
    return result_vec;

}

vector<Point2f> slidingWindow(Mat image,Rect window)
{
    vector<Point2f> points;
    const Size imgSize=image.size();
    bool shouldBreak=false;
    while(true)
    {
        float currentX=window.x+window.width*0.5f;
        Mat roi=image(window);
        vector<Point2f> locations;
        findNonZero(roi,locations);
        float avgX=0.0f;
        for (int i=0;i<locations.size();i++)
        {
            float x = locations[i].x;
            avgX+=x;
        }
        float avgY=0.0f;
        for (int i=0;i<locations.size();i++)
        {
            float y=locations[i].y;
            avgY+=y;
        }
        avgX=locations.empty() ? -1:avgX/locations.size();
        avgY=locations.empty() ? -1:avgY/locations.size();
        Point point;
        if (!(avgX==-1||avgY==-1))
        {
            avgX+=window.x;
            avgY+=window.y;
            point.x=avgX;
            point.y=avgY;
            points.push_back(point);
        }
        window.y -= window.height;
        if (window.y<0)
        {
            window.y=0;
            shouldBreak=true;
        }
        int origin=window.x;
        window.x+=(point.x-currentX);
        if(window.x<0) window.x=origin;
        if(window.x+window.width>=imgSize.width)
            window.x=imgSize.width - window.width-1;
        if(shouldBreak) break;
    }
    return points;
}

Mat laneimage_processing(Mat work_img)
{
    Mat maskYellow, maskWhite;
       
    inRange(work_img, Scalar(160,160,0),Scalar(255,255,50),maskYellow);
    inRange(work_img,Scalar(200,200,200),Scalar(255,255,255),maskWhite);
       
    Mat mask,processed;
    bitwise_or(maskYellow,maskWhite,mask);
    bitwise_and(work_img,mask,processed);
       
    const Size kernelSize=Size(9,9);
    GaussianBlur(processed,processed,kernelSize,0);
       
    Mat kernel=Mat::ones(15,15,CV_8U);
    dilate(processed,processed,kernel);
    erode(processed,processed,kernel);
    morphologyEx(processed,processed,MORPH_CLOSE,kernel);
       
    const int thresholdVal=150;
    threshold(processed,processed,thresholdVal,255,THRESH_BINARY);
    
    return processed;
}
