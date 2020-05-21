# lane_detection with C++

# reference source https://janhalozan.com/2019/06/01/lane-detector/
# Calibration Image & Video Source https://github.com/hmc0105/advanced_lane_detection

# Needed OpenCV

Play
1. clone this github repository
2. play main.cpp

Processes in main.cpp
1. Do image Calibration using data_cal/*.jpg
2. Read Video and get a frame (continuously)
3. get a bird view from each frame
4. change it into Gray image
5. Do Window Seeaching Algorithm and find line Points
6. Using line Points, it calculate quadratic regressions for each line
7. Draw it into original frame
