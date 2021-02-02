#ifndef findBox
#define findBox
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <iostream>
#include <algorithm>
#include <iterator> 

using namespace cv;
using namespace std;

class FindBox{
    private:
    // Mat img;
    // vector<Point2f> pointBuf;

    vector<Point> getCorners(Mat& grey_mat, Mat& mat);
    cv::Point getAngle(const cv::Point& p1, const cv::Point& p2, const cv::Point& p3);
    double getAnglehelper(const cv::Point& center, const cv::Point& point, const cv::Point& base);
    vector<vector<int>> getcomb(int N, int K);
    vector<float> getSlope(float x1, float y1, float x2, float y2);
    vector<Point2f> detectTrape(const vector<Point2f>& arr);
    float checkSquare(const Mat& mat, int mask_sz, vector<Point> corners);

    public:

        // FindBox(Mat& arg_img, vector<Point2f>&arg_pointBuf){
        //     img = arg_img;
        //     pointBuf = arg_pointBuf;
        // }

        FindBox(){
        }

        bool getBox (Mat& img, vector<Point2f>& pointBuf);
    
};

#endif