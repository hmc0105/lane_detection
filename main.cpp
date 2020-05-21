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
#include "funcs.hpp"

using namespace cv;
using namespace std;
namespace filesys = std::filesystem;
Size patternsize(9,6);
vector<vector<Point3f>> objectPoints;
vector<vector<Point2f>> imagePoints;
vector<vector<Point3f>>::iterator it2;
vector<vector<Point2f>>::iterator it3;

int main(){
    filesys::path currentpath=filesys::current_path();
    currentpath+="/data/*.jpg";
    vector<String> filenames;
    glob(currentpath,filenames,true);
    vector<Point2f> corners;
    vector<Point3f> objp;
    
    //Making Object Vector(3D location of board)
    vector<Point3f>::iterator it;
    it=objp.begin();
    for (int i=0;i<54;i++)
    {
        Point3f pt(i%9,i/9,0);
        it=objp.insert(it,pt);
    }
    
    
    //Save All board informations
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
    Mat init_img;
    Mat org;
    Mat work_img;
    
    while(true)
    {
        cap.read(init_img);
        if (init_img.empty()) break;
        
        undistort(init_img,org,cameraMatrix,distCoeffs);
        
        warpPerspective(org,dest,perspectiveMatrix,dest.size(),INTER_LINEAR,BORDER_CONSTANT);
        
        cvtColor(dest,work_img,COLOR_RGB2GRAY);
        
        Mat processed=laneimage_processing(work_img);
        
        
        vector<Point2f> lpts = slidingWindow(processed,Rect(0,420,120,60));
        vector<Point2f> rpts = slidingWindow(processed,Rect(520,420,120,60));
    
        
        vector<Point2f> loutpts,routpts;
        perspectiveTransform(lpts,loutpts,invertedPerspectiveMatrix);
        perspectiveTransform(rpts,routpts,invertedPerspectiveMatrix);

        vector<float> polyl_const= polyfit(loutpts);
        vector<float> polyr_const= polyfit(routpts);

        
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
        fillPoly(org,ppt,npt,1,Scalar(255,0,255));
        addWeighted(org,0.5,copyimg,0.5,0,copyimg);
        
        imshow("Image",copyimg);
        waitKey();
        
    }
    return 0;
}
