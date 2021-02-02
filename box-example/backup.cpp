

    // Good Feature to Track
    // Mat mask = cv::Mat::zeros(grey_mat.rows, grey_mat.cols, CV_8U);
    // const Point* ppt[1] = { &poly[0] };
    // int num = poly.size();
    // int npt[] = {num};
    // fillPoly(mask,
    //         ppt,
    //         npt,
    //         1,
    //         Scalar( 255, 255, 255 ),
    //         8);

    // vector<Point> corners;
    // goodFeaturesToTrack(mask, corners, 4, 0.5, 100);
    // for (auto k:corners ){
    //     circle(mat, k, 50, (0, 0, 255), 3);
    // }
    // imshow( "Components", mat);
    // waitKey(0);

    // if (corners.size() < 4){
    //     return poly; //to be change
    // }


    // FastFeatureDetector
    // Mat mask = cv::Mat::zeros(grey_mat.rows, grey_mat.cols, CV_8U);
    // const Point* ppt[1] = { &poly[0] };
    // int num = poly.size();
    // int npt[] = {num};
    // fillPoly(mask,
    //         ppt,
    //         npt,
    //         1,
    //         Scalar( 255, 255, 255 ),
    //         8);

    // Ptr<FastFeatureDetector> fastDetector = FastFeatureDetector::create(10, true);
    // std::vector<cv::KeyPoint> keypoints;
    // fastDetector->detect(grey_mat, keypoints);
    // cv::Mat output;
    // cv::drawKeypoints(mask, keypoints, output);
    // cv::imshow("sift_result", output);
    // waitKey(0);

    // int dilation_size = 0;
    // Mat element = getStructuringElement( MORPH_RECT,
    //                 Size( 2*dilation_size + 1, 2*dilation_size+1 ),
    //                 Point( dilation_size, dilation_size ) );
    // cv::erosion(mask, erosion, element);

    // Mat erosion;
    // erode(mask, erosion,5);
    // cv::imshow("sift_result", erosion);
    // waitKey(0);
    // vector<Point> boundary;
    // for (auto elem: keypoints){
    //     uchar val = erosion.at< uchar >(elem.pt);
    //     cout << +val << endl;
    //     if (+val != 255){
    //         boundary.push_back(elem.pt);
    //     }
    // }
    // for (auto k:boundary ){
    //     circle(mat, k, 5, (0, 0, 255), 3);
    // }
    // imshow( "Components", mat);
    // waitKey(0);


    // for (auto elem: keypoints){
    //     cout << elem.pt <<endl;
    // }
    

    // vector<Point> all_ptsi;
    // for (auto elem: keypoints){
    //     all_ptsi.push_back(cv::Point(int(elem.pt.x), int(elem.pt.y)));
    // }
    

    






    // vector<Vec2f> lines; // will hold the results of the detection
    // HoughLines(canny_mat, lines, 1, CV_PI/180, 150, 50, 50 );

    // for( size_t i = 0; i < lines.size(); i++ )
    // {
    //     float rho = lines[i][0], theta = lines[i][1];

    //     float m = - std::cos(theta) / std::sin(theta); 
    //     float c = rho / std::sin(theta);
    //     // cout << "m"<< m << "; c"<< c << endl;


    //     Point pt1, pt2;
    //     double a = cos(theta), b = sin(theta);
    //     double x0 = a*rho, y0 = b*rho;
    //     pt1.x = cvRound(x0 + 1000*(-b));
    //     pt1.y = cvRound(y0 + 1000*(a));
    //     pt2.x = cvRound(x0 - 1000*(-b));
    //     pt2.y = cvRound(y0 - 1000*(a));
    //     cv::line(mat, pt1, pt2, (0,0,255), 3);
    // }
    // imshow("Hough Line Transform", mat);
    // waitKey(0);

    // Mat labels;
    // std::vector<Point2f> centers;
    // double compactness = cv::kmeans(lines, 2, lines[1],
    //                             TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0),
    //                             3, KMEANS_PP_CENTERS, centers);
    // cout << centers << endl;




    // RotatedRect r = minAreaRect(contours[0]);
    // cv::Point2f boxPts[4];
    // r.points(boxPts);

    // vector<Point2f> all_pts;
    // all_pts.push_back(*boxPts);
    // all_pts.push_back(*(boxPts+1));
    // all_pts.push_back(*(boxPts+2));
    // all_pts.push_back(*(boxPts+3));

    // vector<Point> all_ptsi;

    // for (auto pt: all_pts){
    //     all_ptsi.push_back(cv::Point(int(pt.x), int(pt.y)));
    // }

    // Mat mask = cv::Mat::zeros(grey_mat.rows, grey_mat.cols, CV_8U);
    // const Point* ppt[1] = { &all_ptsi[0] };
    // int npt[] = {4};
    // fillPoly(mask,
    //         ppt,
    //         npt,
    //         1,
    //         Scalar( 255, 255, 255 ),
    //         8);
    // imshow( "mask", mask);
    // waitKey(0);
    // vector<Point> corners;
    // goodFeaturesToTrack(mask, corners, 4, 0.5, 100);



    //Harris Corner Detection
    // Mat dst = Mat::zeros( grey_mat.size(), CV_32FC1 );
    // cv::cornerHarris(grey_mat, dst, 5,3,0.02);

    // Mat dst_norm, dst_norm_scaled;
    // normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
    // convertScaleAbs( dst_norm, dst_norm_scaled );
    // int dilation_size = 0;
    // Mat element = getStructuringElement( MORPH_RECT,
    //                 Size( 2*dilation_size + 1, 2*dilation_size+1 ),
    //                 Point( dilation_size, dilation_size ) );
    // cv::dilate(dst_norm_scaled, dst_norm_scaled, element);
    // adaptiveThreshold(dst_norm_scaled, dst_norm_scaled,255,1,0,11,2);

    // Mat labels;
    // Mat stats;
    // Mat centroids;
    // cv::connectedComponentsWithStats(dst_norm_scaled, labels, stats, centroids);

    // // cout <<centroids.size() << " " << stats.size() << endl;

    vector<Point2f> cen;
    // for (int i = 0; i < centroids.rows; i++) {
    //     double x = centroids.at<double>(i,0);
    //     double y = centroids.at<double>(i,1);
    //     cen.push_back(Point2f(x,y)); 
    // }

    // cornerSubPix(dst_norm_scaled,cen, Size(11,11), Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));

