#include "findBox.hpp"

bool FindBox::getBox (Mat& img, vector<Point2f>& pointBuf){
    Mat dst, grey_img;

    //turn to gray scale
    cvtColor(img, grey_img, COLOR_BGR2GRAY);

    //dilate
    int dilation_size = 0;
    Mat element = getStructuringElement( MORPH_RECT,
                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                       Point( dilation_size, dilation_size ) );
    cv::dilate(grey_img, dst, element);

    vector<Point> poly;
    vector<Point> corners = getCorners(dst, img, poly);

    if (corners.size() < 4 || corners.size() >45){
        cout << "Failed. No. of detected corners:" << corners.size() << endl;
        return false;
    }

    std::rotate(corners.begin(),
                corners.begin()+2, // this will be the new first element
                corners.end());

    Mat mask = cv::Mat::zeros(dst.rows, dst.cols, CV_8U);
    const Point* ppt[1] = { &poly[0] };
    int num = poly.size();
    int npt[] = {num};
    fillPoly(mask,
            ppt,
            npt,
            1,
            Scalar( 255, 255, 255 ),
            8);
    int mask_sz = countNonZero(mask);
    if (mask_sz < 1000){
        cout << "Failed2. Mask too small." << endl;
        return false;
    }

    vector<Point> sq;
    vector<vector<int>> comb =  getcomb(corners.size(), 4);
    vector<vector<Point>> pot_sq;
    vector<double> ptw;
    bool found =false;
    for (auto c: comb){
        sq = {corners[c[0]],corners[c[1]],corners[c[2]],corners[c[3]]};
        //check area of bounding box
        cv::RotatedRect rectangle = cv::minAreaRect(sq);
        auto sz = rectangle.size;
        float area = sz.height*sz.width;
        if ( area < FindBox::SQ_HALF_AREA){ //full square area = 773865
            continue;
        }
        // cout << "Area: " << area << endl;
        double white_space = checkSquare(mask, mask_sz, sq);
        // cout << "White: " << white_space << endl;
        if (white_space < FindBox::SQ_AREA_THRES){
            found = true;
            cout << "Detected area of white: " << white_space << endl;
            ptw.push_back(white_space);
            pot_sq.push_back(sq);
        }
    }
    if (!found){
        cout << "Cannot find 4 pt for rectangle" << endl;
        return false;
    } 
    // sq = pot_sq[std::distance(ptw.begin(),std::min_element(ptw.begin(), ptw.end()))];
    sq = SquareSanityCheck(pot_sq, mask_sz, ptw, mask);

    // for (auto k:sq ){
    //     circle(img, k, 5, (0, 0, 255), 3);
    // }
    // imshow( "Components", img);
    // waitKey(0);

    //find the main corner point
    vector<Point> ans(4);
    int g=0;
    bool found_main = false;
    vector<float> shapesz_vec;
    for (g; g<4; g++){
        cv::Mat mask2 = cv::Mat::zeros(img.rows, img.cols, CV_8U);
        cv::circle(mask2, sq[g], 50, (255), -1, FILLED, 0);
        cv::Mat res;
        cv::bitwise_and(mask2 , mask, res);
        // cout << countNonZero(res) << endl;
        vector<Point> p;
        // float shapesz = countNonZero(res);
        goodFeaturesToTrack(res, p, 6, 0.5, 5);
        // fillConvexPoly(res,p,Scalar(255,255,255));
        // float shapesz_new = countNonZero(res);
        // shapesz_vec.push_back(shapesz_new-shapesz);
        // cout << "No. of good features: " << p.size() << endl;
        if (p.size() != 3){
            found_main =  true;
            break;
        }
    }
    // g = std::distance(shapesz_vec.begin(),std::max_element(shapesz_vec.begin(), shapesz_vec.end()));
    if (!found_main){
        cout << "cant find main corner" << endl;
        // g = 0;
        return false;
    }
    ans[0] = sq[g];
    sq.erase(sq.begin()+g);

    //find the opposite corner of the main corner
    vector<float> dist;
    for (auto points: sq){
        dist.push_back(getDistance(ans[0], points));
    }
    g = std::distance(dist.begin(),std::max_element(dist.begin(), dist.end()));
    ans[1] = sq[g];
    sq.erase(sq.begin()+g);

    // find the corner that connect with the main point through a the broken line segment
    vector<float> num_nonzero;
    for (auto points: sq){
        Mat test_img = mask.clone();
        cv::line(test_img, ans[0], points, (255),1);
        num_nonzero.push_back(countNonZero(test_img));
    }
    g = (num_nonzero[0] > num_nonzero[1])? 0 : 1;
    ans[2] = sq[g];
    sq.erase(sq.begin()+g);
    ans[3] = sq[0];

    for (auto elem: ans){
        pointBuf.push_back(Point2f(float(elem.x), float(elem.y)));
    }

    cornerSubPix(grey_img, pointBuf, Size(11,11), Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));

    cout << "Acceptable square? " << endl;
    for (int k=0; k<pointBuf.size(); k++){
        putText(grey_img, to_string(k), Point2f(pointBuf[k].x+50, pointBuf[k].y+50),0,2,(0,0,255));
        circle(grey_img,pointBuf[k] , 5, (0, 0, 255), 3);
    }
    imshow("out", grey_img);
    waitKey(50);
    char user_ans;
    cin >> user_ans ;
    if (user_ans == 'y'){
        return true;
    } 
    
    return false;
}

float FindBox::getDistance(Point p1, Point p2){
    return (pow(p1.x - p2.x,2) + pow(p1.y - p2.y,2));
}

vector<Point> FindBox::SquareSanityCheck(vector<vector<Point>> pot_sq, int mask_sz, vector<double> white_space,const Mat& ref_mat){
    vector<float> arr;
    for (auto sq: pot_sq){
        Mat oursq = cv::Mat::zeros(ref_mat.rows, ref_mat.cols, CV_8U);
        fillConvexPoly(oursq,sq,Scalar(255,255,255));

        Mat res;
        cv::bitwise_xor(oursq, ref_mat, res);
        float a = countNonZero(res)*100/mask_sz; 
        arr.push_back(a);
        // cout << "extra space"<< a << endl;
        // imshow("square sanity check",oursq);
        // waitKey(0);
    }

    return pot_sq[std::distance(arr.begin(),std::min_element(arr.begin(), arr.end()))];

}

double FindBox::checkSquare(const Mat& arg_mat, int mask_sz, vector<Point> arg_corners){

    vector<vector<int>> comb = getcomb(4,2);
    for(auto c: comb){
        if (getDistance(arg_corners[c[0]], arg_corners[c[1]]) < SQ_PROXIMITY){
            cout << "Points too close together to be considered as square" << endl;
            return 100;
        }
    }

    // Mat mat = cv::Mat::zeros(arg_mat.rows, arg_mat.cols, CV_8U);
    Mat mat = arg_mat.clone();

    fillConvexPoly(mat,arg_corners,Scalar(0,0,0));
    // Mat mat = cv::Mat::zeros(arg_mat.rows, arg_mat.cols, CV_8U);
    // fillConvexPoly(mat,arg_corners,Scalar(0,0,0));
    int count = countNonZero(mat);
    // double per = count*100/mask_sz;
    double per = count*100/mask_sz;

    cout << "Count: " << count << endl;
    cout << "mask_sz: " << mask_sz << endl; 

    vector<Point> corners;
    if (per < FindBox::SQ_AREA_THRES){
        bool parallel = detectparallels(arg_corners, corners);
        // imshow("square sanity check",mat);
        // waitKey(0);
        if (parallel){
            return per;
        }
    }
    return 100;

    // return 100;
}

bool FindBox::detectparallels(const vector<Point>& arr, vector<Point>& output){
    
    vector<vector<int>> comb = getcomb(4,2);
    vector<float> slopes;
    for (auto c: comb){
        slopes.push_back(getSlope(arr[c[0]].x, arr[c[0]].y, arr[c[1]].x, arr[c[1]].y));
    }

    std::sort(slopes.begin(),slopes.end());
    vector<float> compare(3);

    compare[0] = abs(slopes[1]-slopes[0]);
    compare[1] = abs(slopes[3]-slopes[2]);
    compare[2] = abs(slopes[5]-slopes[4]);

    for (auto elem: slopes){
        cout << "slope: " << elem << endl;
    }

    int count = 0;
    for (auto elem:compare){
        if (elem < FindBox::SQ_PARAL_THRES){
            count ++;
        }
    }

    if (count > 1){
        cout << "Can detect parallel!!!" << endl;
        return true;
    }
    cout << "Cannot detect parallel" << endl;
    return false;

}

float FindBox::getSlope(float x1, float y1, float x2, float y2){
    vector<float> ans;
    float m = (y2-y1)/(x2-x1);
    // ans.push_back(m);
    // ans.push_back(y1-m*x1);
    return m;
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


vector<Point> FindBox::getCorners(Mat& grey_mat, Mat& mat, vector<Point>& poly_mask){
    float masksizeThres,  FastDectorThres;
    bool cam = FindBox::SONY;
    if (cam){
        masksizeThres = 5000;
        FastDectorThres = 10;
    } else {
        masksizeThres = 1000;
        FastDectorThres = 5;
    }

    Mat grad_x, grad_y, abs_grad_x, abs_grad_y, sobel_mat, canny_mat;
    vector<vector<Point>> contours;
    // vector<Point> poly;

    int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;

    //Blur img
    GaussianBlur(grey_mat, grey_mat, Size(5,5), 0, 0, BORDER_DEFAULT );

    // adaptiveThreshold(grey_mat, grey_mat,255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,0,11,2);

    //Sobel
    // Gradient X
	Sobel( grey_mat, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
	// Gradient Y
	Sobel( grey_mat, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
	convertScaleAbs( grad_x, abs_grad_x );
	convertScaleAbs( grad_y, abs_grad_y );
	addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, sobel_mat);

    //Canny
    int thresh = 50;
    Canny(sobel_mat, canny_mat, thresh, 150);


    vector<Vec4i> hierarchy;
    findContours(canny_mat, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    vector<vector<Point>> poly(contours.size());
    for( size_t i = 0; i < contours.size(); i++ )
    {
        approxPolyDP( contours[i], poly[i], 3, true );
    }
    float max_area = 0;
    int max_poly_idx = 0;

    for(size_t i = 0 ; i < poly.size() ; i++){
        if (poly[i].size()>1){
            double a = contourArea(poly[i]);
            // cout << "area: " << a << endl;
            if (a > max_area){
                max_poly_idx = i;
            }
        }
    }
    poly_mask = poly[max_poly_idx];
    // for (auto elem: poly[max_poly_idx]){
    //     poly_mask.push_back(elem);
    // }

    Mat mask = cv::Mat::zeros(mat.rows, mat.cols, CV_8U);
    const Point* ppt[1] = { &poly_mask[0] };
    int num = poly_mask.size();
    int npt[] = {num};
    fillPoly(mask,
            ppt,
            npt,
            1,
            Scalar( 255, 255, 255 ),
            8);
    int mask_sz = countNonZero(mask);
    if (mask_sz < masksizeThres){
        cout << "Failed1. Mask too small." << endl;
        vector<Point> dummy;
        return dummy;
    }

    Ptr<FastFeatureDetector> fastDetector = FastFeatureDetector::create(FastDectorThres, true);
    std::vector<cv::KeyPoint> keypoints;
    fastDetector->detect(grey_mat, keypoints);
    Mat mask2, mask3, mask4;
    int morph_size = 10;
    Mat element_er = getStructuringElement( MORPH_ELLIPSE, cv::Size( 2*morph_size + 10, 2*morph_size +10), cv::Point( morph_size, morph_size ) );
    erode(mask, mask2, element_er);
    dilate(mask, mask3, element_er);

    cv::bitwise_xor(mask2,  mask3, mask4);
    // imshow("ero mask", mask4);
    // waitKey(0);

    vector<Point> cen_final;

    for (auto elem: keypoints){
        uchar val = mask4.at< uchar >(elem.pt);
        // cout << +val << endl;
        if ( val == 255){
            cen_final.push_back(elem.pt);

        }
    }
    return cen_final;

}


cv::Point FindBox::getAngle(const cv::Point& p1, const cv::Point& p2, const cv::Point& p3){

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
