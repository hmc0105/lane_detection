//
//  funcs.hpp
//  OpenCV
//
//  Created by 조현민 on 2020/05/21.
//  Copyright © 2020 Daniel Cho. All rights reserved.
//

#ifndef funcs_hpp
#define funcs_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;


int quadratic_eq(vector<float> poly, int x);

vector<float> polyfit(vector<Point2f>& pts);

vector<Point2f> slidingWindow(Mat image,Rect window);

Mat laneimage_processing(Mat img);


#endif /* funcs_hpp */
