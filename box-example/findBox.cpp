#include "findBox.hpp"

bool FindBox::getBox (Mat& img, vector<Point2f>& pointBuf){
    Mat dst;

    //turn to gray scale
    cvtColor(img, dst, COLOR_BGR2GRAY);

    //dilate
    int dilation_size = 0;
    Mat element = getStructuringElement( MORPH_RECT,
                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                       Point( dilation_size, dilation_size ) );
    cv::dilate(dst, dst, element);

    vector<Point> corners = getCorners(dst, img);

    if (corners.size() < 4){
        return false;
    }

    std::rotate(corners.begin(),
                corners.begin()+2, // this will be the new first element
                corners.end());

    Mat mask = cv::Mat::zeros(dst.rows, dst.cols, CV_8U);
    const Point* ppt[1] = { &corners[0] };
    int num = corners.size();
    int npt[] = {num};
    fillPoly(mask,
            ppt,
            npt,
            1,
            Scalar( 255, 255, 255 ),
            8);
    int mask_sz = countNonZero(mask);

    vector<Point> threeRightAngle;
    vector<Point> sq;
    vector<vector<int>> comb =  getcomb(corners.size(), 4);
    for (auto c:comb){
        sq = {corners[c[0]],corners[c[1]],corners[c[2]],corners[c[3]]};

        //check area of bounding box
        cv::RotatedRect rectangle = cv::minAreaRect(sq);
        auto sz = rectangle.size;
        float area = sz.height*sz.width;
        if ( area < 400000){ //full square area = 773865
            continue;
        }
        // cout << "Area: " << area << endl;
        float white_space = checkSquare(mask, mask_sz, sq);
        if (white_space < 5){
            break;
        }

    }

    std::sort(sq.begin(), sq.end(),[](const cv::Point2f &a, const cv::Point2f &b) {
        return (a.x > b.x);
    });
    // for (auto elem: sq){
    //     cout << elem << endl;
    // }
    int g=0;
    for (g; g<4; g++){
        cv::Mat mask2 = cv::Mat::zeros(img.rows, img.cols, CV_8U);
        cv::circle(mask2, sq[g], 60, (255), -1, FILLED, 0);
        cv::Mat res;
        cv::bitwise_and(mask2 , mask, res);
        cout << countNonZero(res) << endl;

        vector<Point> p;
        goodFeaturesToTrack(res, p, 5, 0.5, 5);
        // cout << "No of points: "<< p.size()<< endl;

        if (p.size() != 3){
            break;
        }
    }

    cout << g << endl;
    // vector<Point> final_ans;
    // final_ans.push_back(sq[g]);

    std::rotate(sq.begin(),
                sq.begin()+g, // this will be the new first element
                sq.end());

    // for (auto elem: sq){
    //     cout << elem << endl;
    // }




    cv::Point2f pt(10, 20);
    pointBuf.push_back(pt);
    return true;
}

float FindBox::checkSquare(const Mat& arg_mat, int mask_sz, vector<Point> corners){

    Mat mat = arg_mat.clone();
    const Point* ppt[1] = { &corners[0] };
    int num = corners.size();
    int npt[] = {num};
    fillPoly(mat,
            ppt,
            npt,
            1,
            Scalar( 0,0,0 ),
            8);

    int count = countNonZero(mat);
    double per = count*100/mask_sz;
    // cout << "non zero: " << std::to_string(per) <<endl;
    // cout << countNonZero(mat) << " "<< mask_sz<< endl;
    // imshow( "Components", mat);
    // waitKey(0);
    return per;
}

vector<Point2f> FindBox::detectTrape(const vector<Point2f>& arr){
    //find parallel line
    vector<vector<int>> comb = {{0,1,2,3},{0,2,1,3},{0,3,1,2}};
    vector<Point2f> de;
    for (auto g: comb){
        // cout << g[0] << g[1] << g[2] << g[3] <<endl;
        vector<float> line1, line2, line3;
        line1 = getSlope(arr[g[0]].x,  arr[g[0]].y, arr[g[1]].x,  arr[g[1]].y);
        line2 = getSlope(arr[g[2]].x,  arr[g[2]].y, arr[g[3]].x,  arr[g[3]].y);
        line3 = getSlope(arr[g[0]].x,  arr[g[0]].y, arr[g[2]].x,  arr[g[2]].y);

        if (abs(line2[0]-line1[0]) < 0.5 && abs(line3[0]-line1[0]) > 50){
            cout << line1[0] << line2[0] << line3[0] <<endl;
            cout << "parallel " << endl;
            vector<Point2f> ans = {arr[g[0]],arr[g[1]],arr[g[2]],arr[g[3]]};
            return ans;
        }
    }
    return de;
    

}

vector<float> FindBox::getSlope(float x1, float y1, float x2, float y2){
    vector<float> ans;
    float m = (y2-y1)/(x2-x1);
    ans.push_back(m);
    ans.push_back(y1-m*x1);
    return ans;
}



vector<vector<int>> FindBox::getcomb(int N, int K)
{
    vector<vector<int>> results;
    std::string bitmask(K, 1); // K leading 1's
    bitmask.resize(N, 0); // N-K trailing 0's
 
    // print integers and permute bitmask
    do {
        vector<int> arr;
        for (int i = 0; i < N; ++i) // [0..N-1] integers
        {
            if (bitmask[i]) arr.push_back(i);
        }
        results.push_back(arr);
    } while (std::prev_permutation(bitmask.begin(), bitmask.end()));

    return results;
}


vector<Point> FindBox::getCorners(Mat& grey_mat, Mat& mat){
    Mat grad_x, grad_y, abs_grad_x, abs_grad_y, sobel_mat, canny_mat;
    vector<vector<Point>> contours;
    vector<Point> poly;

    int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;

    //Blur img
    GaussianBlur(grey_mat, grey_mat, Size(5,5), 0, 0, BORDER_DEFAULT );

    //Sobel
    // Gradient X
	Sobel( grey_mat, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
	// Gradient Y
	Sobel( grey_mat, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
	convertScaleAbs( grad_x, abs_grad_x );
	convertScaleAbs( grad_y, abs_grad_y );
	addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, sobel_mat);

    //Canny
    int thresh = 100;
    Canny(sobel_mat, canny_mat, thresh, 150);

    vector<Vec4i> hierarchy;
    findContours(canny_mat, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    approxPolyDP(contours[0], poly, 3, true );

    // for (auto k:poly ){
    //     circle(mat, k, 5, (0, 0, 255), 3);
    // }
    // imshow( "Components", mat);
    // waitKey(0);


    return poly;

}


cv::Point FindBox::getAngle(const cv::Point& p1, const cv::Point& p2, const cv::Point& p3){
    /* Compute the angle between the center, a point and it's base (starting point for the angle computation)
     *
     *      %
     *    *   *         @ = center
     *   *  @  #        # = base (origin)
     *    *   *         % = point
     *      *           From # to %, there are 90 degrees
     */



    double angle = getAnglehelper(p1,p2,p3);    
    if (angle < 110 && angle > 70){
        return p1;
    }

    angle = getAnglehelper(p2,p1,p3);
    if (angle < 110 && angle > 70){
        return p2;
    }

    angle = getAnglehelper(p3,p1,p2);   
    if (angle < 110 && angle > 70){
        return p3;
    }

    return cv::Point(-1,-1); 

}

double FindBox::getAnglehelper(const cv::Point& center, const cv::Point& point, const cv::Point& base){
    double angle = std::atan2(point.y - center.y, point.x - center.x) * 180 / 3.141592;
    angle = (angle < 0) ? (360 + angle) : angle;
    angle = 360 - angle;
    return angle;
}
