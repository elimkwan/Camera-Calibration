#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include "findBox.hpp"

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

using namespace cv;
using namespace std;


// double findBoxAngle(const cv::Point& center, const cv::Point& point, const cv::Point& base)
// {
//     /* Compute the angle between the center, a point and it's base (starting point for the angle computation)
//      *
//      *      %
//      *    *   *         @ = center
//      *   *  @  #        # = base (origin)
//      *    *   *         % = point
//      *      *           From # to %, there are 90 degrees
//      */

//     double angle = std::atan2(point.y - center.y, point.x - center.x) * 180 / 3.141592;
//     angle = (angle < 0) ? (360 + angle) : angle;
//     return (360 - angle);
// }

// vector<Point> findBoxCorners(Mat& grey_mat, Mat& mat){

//     Mat grad_x, grad_y, abs_grad_x, abs_grad_y, sobel_mat, canny_mat;
//     vector<vector<Point>> contours;
//     vector<Point> poly;

//     int scale = 1;
// 	int delta = 0;
// 	int ddepth = CV_16S;

//     //Blur img
//     GaussianBlur(grey_mat, grey_mat, Size(5,5), 0, 0, BORDER_DEFAULT );

//     //Sobel
//     // Gradient X
// 	Sobel( grey_mat, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
// 	// Gradient Y
// 	Sobel( grey_mat, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
// 	convertScaleAbs( grad_x, abs_grad_x );
// 	convertScaleAbs( grad_y, abs_grad_y );
// 	addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, sobel_mat);

//     //Canny
//     int thresh = 100;
//     Canny(sobel_mat, canny_mat, thresh, thresh*2);

//     vector<Vec4i> hierarchy;
//     findContours(canny_mat, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

//     // int idx = 0;
//     // for( ; idx >= 0; idx = hierarchy[idx][0] )
//     // {
//     //     Scalar color( rand()&255, rand()&255, rand()&255 );
//     //     // drawContours( mat, contours, idx, color, 8, "FILLED", hierarchy );
//     //     drawContours(mat,contours,-1,(0,0,255),2);
//     // }

//     // imshow( "Components", mat);
//     // waitKey(0);

//     approxPolyDP(contours[0], poly, 3, true );

//     for (auto k:poly ){
//         circle(mat, k, 50, (0, 0, 255), 3);
//     }
//     imshow( "Components", mat);
//     waitKey(0);
    
//     return poly;
// }


// bool findBox(Mat& img, vector<Point2f>& pointBuf){

//     Mat dst;

//     //turn to gray scale
//     cvtColor(img, dst, CV_BGR2GRAY);

//     //dilate
//     int dilation_size = 0;
//     Mat element = getStructuringElement( MORPH_RECT,
//                        Size( 2*dilation_size + 1, 2*dilation_size+1 ),
//                        Point( dilation_size, dilation_size ) );
//     cv::dilate(dst, dst, element);

//     vector<Point> corners = findBoxCorners(dst, img);
    
//     vector<Point> result;
//     // float angle;
//     // for (int d=0; d<corners.size() ;d++){
//     //     for 



//     // }

    


//     cv::Point2f pt(10, 20);
//     pointBuf.push_back(pt);
//     return true;
// }

int main(int argc, char* argv[])
{
    vector<cv::String> fn;
    string img_dir =  "/code/box-example/image/*.JPG";
    glob(img_dir, fn, false);
    size_t count_fn = fn.size();
    Mat cur_img;
    Mat cameraMatrix, distCoeffs;
    vector<vector<Point2f> > imagePoints;

    for(size_t i=0; i<count_fn; i++){

        cout << "Image: " << i << endl;

        bool blinkOutput = false; //to flip the image

        //load in an image
        cur_img = imread(fn[i]);

        //find the presence of the box
        vector<Point2f> pointBuf;
        FindBox box; 
        bool found = box.getBox(cur_img, pointBuf);

        if (found){
            imagePoints.push_back(pointBuf);
            blinkOutput = true;
        }

        if(blinkOutput){
            bitwise_not(cur_img, cur_img);
        }

        // for (auto k:pointBuf){
        //     cout << "Point Buffer" << k << endl;
        // }


    }

    // Calibration
    // runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints)

    // Show Undistort Img
    // Mat temp = view.clone();
    // undistort(temp, view, cameraMatrix, distCoeffs); //opencv func
}

// runCalibrationAndSave

// runCalibration