//
//  Cat_EXP.cpp
//  OpenCV
//
//  Created by 조현민 on 2020/05/19.
//  Copyright © 2020 Daniel Cho. All rights reserved.
//

#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <filesystem>

using namespace cv;
using namespace std;
namespace filesys = std::filesystem;
Size patternsize(9,6);
vector<vector<Point3f>> objectPoints;
vector<vector<Point2f>> imagePoints;
vector<vector<Point3f>>::iterator it2;
vector<vector<Point2f>>::iterator it3;

int quadratic_eq(vector<float> poly, int x)
{
    float c=poly.at(0);
    float b=poly.at(1);
    float a=poly.at(2);
    return (int)(a*x*x+b*x+c);
}

//Point** pts_fillpoly(vector<float> polyl,vector<float> polyr, vector<Point2f>& lpts, vector<Point2f>& rpts,Point (&pts)[][])
//{
//    int lstart = (int) lpts.at(0).x;
//    int lfinish = (int) lpts.back().x;
//    int rstart = (int) rpts.at(0).x;
//    int rfinish = (int) rpts.back().x;
//    int l=0;
//    for (int i=lstart;i<lfinish+1;i++)
//    {
//        pts[0][l]=Point(i,quadratic_eq(polyl,i));
//    }
//    for (int j=rstart;j<rfinish+1;j++)
//    {
//        pts[0][l]=Point(j,quadratic_eq(polyr,j));
//    }
//    return pts;
//}

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
    

int main(){
    filesys::path currentpath=filesys::current_path();
    currentpath+="/data/*.jpg";
    vector<String> filenames;
    glob(currentpath,filenames,true);
    vector<Point2f> corners;
    
    vector<Point3f> objp;
    vector<Point3f>::iterator it;
    it=objp.begin();
    for (int i=0;i<54;i++)
    {
        Point3f pt(i%9,i/9,0);
        it=objp.insert(it,pt);
    }
    
    
    
    for (int i=0;i<filenames.size();i++)
    {
        String filename=filenames[i];
        Mat img=imread(filename);
        Mat gray;
        cvtColor(img,gray,COLOR_BGR2GRAY);
        bool patternfound=findChessboardCorners(gray,patternsize,corners);
        if(patternfound)
        {
            it2=objectPoints.insert(it2,objp);
            it3=imagePoints.insert(it3,corners);
        }
    }
    filesys::path Datapath=filesys::current_path();
    Datapath+="/data/test/straight_lines1.jpg";
    Mat img=imread(Datapath);
    double rms;
    Mat cameraMatrix=Mat::eye(3,3,CV_64F);
    Mat distCoeffs=Mat::zeros(8,1,CV_64F);
    vector<Mat> rvecs, tvecs;
    rms=calibrateCamera(objectPoints,imagePoints,img.size(),cameraMatrix,distCoeffs,rvecs,tvecs);
    Mat dst;
    undistort(img,dst,cameraMatrix,distCoeffs);
    Datapath=filesys::current_path();
    Datapath +="/output.jpg";
    
    filesys::path Videopath=filesys::current_path();
    Videopath+="/video.mp4";
    VideoCapture cap(Videopath);
    if(!cap.isOpened())
    {
        cout << "Fail to open video"<<endl;
        return 1;
    }

    Point2f srcVertices[4];
    srcVertices[0]=Point(566,433);
    srcVertices[1]=Point(758,433);
    srcVertices[2]=Point(1078,630);
    srcVertices[3]=Point(300,630);
    
    Point2f dstVertices[4];
    dstVertices[0]=Point(0,0);
    dstVertices[1]=Point(640,0);
    dstVertices[2]=Point(640,480);
    dstVertices[3]=Point(0,480);
    Mat perspectiveMatrix=getPerspectiveTransform(srcVertices,dstVertices);
    Mat dest(480,640,CV_8UC3);
    
    Mat invertedPerspectiveMatrix;
    invert(perspectiveMatrix,invertedPerspectiveMatrix);
    
    Mat org;
    Mat work_img;
    
    while(true)
    {
        cap.read(org);
        if (org.empty()) break;
    
        warpPerspective(org,dest,perspectiveMatrix,dest.size(),INTER_LINEAR,BORDER_CONSTANT);
        cvtColor(dest,work_img,COLOR_RGB2GRAY);
        
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
        //Checked
        
        
        vector<Point2f> lpts = slidingWindow(processed,Rect(0,420,120,60));
        vector<Point2f> rpts = slidingWindow(processed,Rect(520,420,120,60));
        imshow("Hello",processed);
        waitKey();
        
        vector<Point2f> loutpts,routpts;
        perspectiveTransform(lpts,loutpts,invertedPerspectiveMatrix);
        perspectiveTransform(rpts,routpts,invertedPerspectiveMatrix);

        vector<float> polyl_const= polyfit(loutpts);
        vector<float> polyr_const= polyfit(routpts);
//        Point** pts_fill = pts_fillpoly(polyl_const,polyr_const,lpts,rpts,pts);
        int lstart = (int) loutpts.at(0).x;
        int lfinish = (int) loutpts.back().x;
        if (lstart>lfinish)
        {
            int memory=lstart;
            lstart=lfinish;
            lfinish=memory;
        }
        int rstart = (int) routpts.at(0).x;
        int rfinish = (int) routpts.back().x;
        if(rstart>rfinish)
        {
            int memory=rstart;
            rstart=rfinish;
            rfinish=memory;
        }
        int nptnum=lfinish-lstart+rfinish-rstart+4;
        Point pts[1][nptnum];
        int l=0;
        pts[0][l]=Point(330,670);
        l++;
        for (int i=lstart;i<lfinish+1;i++)
        {
            pts[0][l]=Point(i,quadratic_eq(polyl_const,i));
            l++;

        }
        for (int j=rstart;j<rfinish+1;j++)
        {
            pts[0][l]=Point(j,quadratic_eq(polyr_const,j));
            l++;

        }
        pts[0][l]=Point(1080,670);
        const Point* ppt[1]={pts[0]};
        int npt[1]={nptnum};
        Mat copyimg;
        org.copyTo(copyimg);
        fillPoly(org,ppt,npt,1,Scalar(255,0,0));
        addWeighted(org,0.5,copyimg,0.5,0,copyimg);
        
        imshow("Image",copyimg);
        waitKey();
        
    }
    return 0;
}
