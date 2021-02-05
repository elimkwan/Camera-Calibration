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
VideoCapture inputCapture;
std::ofstream outstream;

Mat nextImage();
bool runCalibration(Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                    vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                    vector<float>& reprojErrs,  double& totalAvgErr);

static double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                         const vector<vector<Point2f> >& imagePoints,
                                         const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                         const Mat& cameraMatrix , const Mat& distCoeffs,
                                         vector<float>& perViewErrors);

bool saveCalibration(std::string name, Mat& cameraMatrix, Mat& distCoeffs);


static double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                         const vector<vector<Point2f> >& imagePoints,
                                         const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                         const Mat& cameraMatrix , const Mat& distCoeffs,
                                         vector<float>& perViewErrors)
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); ++i )
    {
        cv::projectPoints( Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                       distCoeffs, imagePoints2);
        err = cv::norm(Mat(imagePoints[i]), Mat(imagePoints2), 4); //L2 is 4

        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}


bool runCalibration(Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                    vector<vector<Point2f>> imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                    vector<float>& reprojErrs,  double& totalAvgErr){

    cameraMatrix = Mat::eye(3, 3, CV_64F);
    distCoeffs = Mat::zeros(8, 1, CV_64F);

    float squareWidth = 106; //106mm 400.62992126px
    float squareHeight = 105; //105mm 396.8503937px
    vector<vector<Point3f>> objectPoints(1);;
    objectPoints[0].push_back(Point3f(0,0,0));
    objectPoints[0].push_back(Point3f(squareHeight,squareWidth,0));
    objectPoints[0].push_back(Point3f(squareHeight,0,0));
    objectPoints[0].push_back(Point3f(0,squareWidth,0));
    // cout << imagePoints.size() << objectPoints[0].size() << endl;;

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    // //Find intrinsic and extrinsic camera parameters
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                                 distCoeffs, rvecs, tvecs,  cv::CALIB_FIX_K4| cv::CALIB_FIX_K5);

    cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                                            rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;
}

bool saveCalibration(string name, Mat& cameraMatrix, Mat& distCoeffs){

    outstream.open(name, std::ios_base::app);
    if (outstream){

        outstream << "Camera Coefficient"<< endl;
        for (int r=0; r < cameraMatrix.rows; r++){
            for (int c=0; c < cameraMatrix.cols; c++){
                double value = cameraMatrix.at<double>(r,c);
                outstream << value << endl;
            }
        }

        outstream << "Distortion Coefficients "<< endl;
        for (int r=0; r < distCoeffs.rows; r++){
            for (int c=0; c < distCoeffs.cols; c++){
                double value = distCoeffs.at<double>(r,c);
                outstream << value << endl;
            }
        }
        return true;
    }

    return false;

                    
}

Mat nextImage(){
    Mat result;
    if( inputCapture.isOpened() )
    {
        Mat view0;
        inputCapture >> view0;
        view0.copyTo(result);
    }
    return result;
}


int main(int argc, char* argv[])
{
    vector<cv::String> fn;
    string img_dir =  "/code/box-example/image/*";
    glob(img_dir, fn, false);
    size_t count_fn = fn.size();
    Mat cur_img;
    vector<vector<Point2f> > imagePoints;
    // Calibration Param
    cv::Size imageSize;
    Mat cameraMatrix, distCoeffs;
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr;
    int num_valid_photo = 0;

    inputCapture.open(1,cv::CAP_V4L2);
    if( !inputCapture.isOpened() )
    {
        cout << "Cant find camera" << endl;
        return 0;
    }

    for(size_t i=0; i<count_fn; i++){
    // while(num_valid_photo < 20){

        cout << "Image: " << i << endl;

        bool blinkOutput = false; //to flip the image

        //load in an image
        cur_img = imread(fn[i]);

        // cur_img = nextImage();
        // imshow("video stream",cur_img);
        // waitKey(100);

        //find the presence of the box
        vector<Point2f> pointBuf;
        FindBox box; 
        bool found = box.getBox(cur_img, pointBuf);

        if (found){
            num_valid_photo ++;

            std::string name = "./image_v/" + to_string(num_valid_photo) +".jpg";
            cv::imwrite(name, cur_img);

            imagePoints.push_back(pointBuf);
            blinkOutput = true;
        }

        if(blinkOutput){
            bitwise_not(cur_img, cur_img);
        }


    }
    cout << "Number of Valid Photos: " << num_valid_photo << endl;
    cout << "Avg_Reprojection_Error: " << totalAvgErr << endl;;

    if (num_valid_photo > 0){
        imageSize = cur_img.size();
        runCalibration(imageSize, cameraMatrix, distCoeffs,
                    imagePoints, rvecs, tvecs,
                    reprojErrs, totalAvgErr);
        saveCalibration("result", cameraMatrix, distCoeffs);

        Mat view, rview, map1, map2;
        initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
            getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
            imageSize, CV_16SC2, map1, map2);


        vector<cv::String> fn2;
        string img_dir2 =  "/code/box-example/image_v/*";
        glob(img_dir2, fn2, false);
        for(int i = 0; i < (int)fn2.size(); i++ )
        {
            view = imread(fn2[i], 1);

            if(view.empty())
                continue;
            remap(view, rview, map1, map2, INTER_LINEAR);

            std::string name2 = "./image_un/undistorded_" + to_string(i) +".jpg";
            cv::imwrite(name2, rview);

            imshow("Image View", rview);
            waitKey(0);
            // char c = (char)waitKey();
            // if( c  == ESC_KEY || c == 'q' || c == 'Q' )
            //     break;
        }

    } else {
        cout << "Not enough photos" << endl;
    }

    inputCapture.release();
    outstream.close();
    return 0;
}

