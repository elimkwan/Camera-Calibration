#ifndef findBox
#define findBox
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include <algorithm>
#include <iterator> 

using namespace cv;
using namespace std;

class FindBox{
    private:
    // Mat img;
    // vector<Point2f> pointBuf;
    Mat mask;
    int SQ_AREA_THRES = 10; //5, if sony
    int SQ_PARAL_THRES = 1; //0.5, if sony
    int SQ_HALF_AREA = 1000; //350000, if sony
    int SQ_PROXIMITY = 5; //5, if sony
    bool SONY = false;

    vector<Point> getCorners(Mat& grey_mat, Mat& mat, vector<Point>& poly_mask);
    cv::Point getAngle(const cv::Point& p1, const cv::Point& p2, const cv::Point& p3);
    double getAnglehelper(const cv::Point& center, const cv::Point& point, const cv::Point& base);
    vector<vector<int>> getcomb(int N, int K);
    float getSlope(float x1, float y1, float x2, float y2);
    bool detectparallels(const vector<Point>& arr, vector<Point>& output);
    double checkSquare(const Mat& mat, int mask_sz, vector<Point> corners);
    float getDistance(Point p1, Point p2);
    vector<Point> SquareSanityCheck(vector<vector<Point>> pot_sq, int mask_sz, vector<double> white_space, const Mat& ref_mat);

    public:

        FindBox(){
        }

        bool getBox (Mat& img, vector<Point2f>& pointBuf);
    
};

#endif