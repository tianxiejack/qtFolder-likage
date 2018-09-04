/* This is sample from the OpenCV book. The copyright notice is below */

/* *************** License:**************************
   Oct. 3, 2008
   Right to use this code in any way you want without warranty, support or any guarantee of it working.

   BOOK: It would be nice if you cited it:
   Learning OpenCV: Computer Vision with the OpenCV Library
     by Gary Bradski and Adrian Kaehler
     Published by O'Reilly Media, October 3, 2008

   AVAILABLE AT:
     http://www.amazon.com/Learning-OpenCV-Computer-Vision-Library/dp/0596516134
     Or: http://oreilly.com/catalog/9780596516130/
     ISBN-10: 0596516134 or: ISBN-13: 978-0596516130

   OPENCV WEBSITES:
     Homepage:      http://opencv.org
     Online docs:   http://docs.opencv.org
     Q&A forum:     http://answers.opencv.org
     Issue tracker: http://code.opencv.org
     GitHub:        https://github.com/opencv/opencv/
   ************************************************** */

#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/stitching/detail/matchers.hpp"

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

using namespace cv;
using namespace std;
using namespace cv::detail;
using namespace cv::xfeatures2d;

const char* liveCaptureHelp =
    "When the live video from camera is used as input, the following hot-keys may be used:\n"
        "  <ESC>, 'q' - quit the program\n"
        "  's' - save gun ball linkage params\n";

static int print_help()
{
    cout <<
            " Given a list of chessboard images, the number of corners (nx, ny)\n"
            " on the chessboards, and a flag: useCalibrated for \n"
            "   calibrated (0) or\n"
            "   uncalibrated \n"
            "     (1: use cvStereoCalibrate(), 2: compute fundamental\n"
            "         matrix separately) stereo. \n"
            " Calibrate the cameras and display the\n"
            " rectified results along with the computed disparity images.   \n" << endl;
    cout << "Usage:\n ./stereo_calib -w=<board_width default=9> -h=<board_height default=6> -s=<square_size default=1.0> <image list XML/YML file default=../data/stereo_calib.xml>\n" << endl;
    cout << liveCaptureHelp << endl;
    return 0;
}

static bool loadCameraParams( const string& filename, int& flags, Mat& cameraMatrix2, Mat& distCoeffs2, Size& imageSize)
{
    FileStorage fs2( filename, FileStorage::READ );

    bool ret = fs2.isOpened();
    if(!ret){
        cout << filename << " can't opened !\n" << endl;
        return ret;
    }

    String date;
    fs2["calibration_time"] >> date;

    int frameCount = (int)fs2["nframes"];
    int imgwidth = (int)fs2["image_width"];
    int imgheight = (int)fs2["image_height"];
    imageSize.width = imgwidth;
    imageSize.height = imgheight;
    double totalAvgErr;

    //Mat cameraMatrix2, distCoeffs2;
    fs2["camera_matrix"] >> cameraMatrix2;
    fs2["distortion_coefficients"] >> distCoeffs2;
    fs2["avg_reprojection_error"] >> totalAvgErr;

    cout << "calibration date: " << date << endl
         << "image width:      " << imgwidth << endl
         << "image height:     " << imgheight << endl
         << "camera matrix:    \n" << cameraMatrix2 << endl
         << "distortion coeffs:\n" << distCoeffs2 << endl
         << "avg error:        " << totalAvgErr << endl;

    /*FileNode features = fs2["features"];
    FileNodeIterator it = features.begin(), it_end = features.end();
    int idx = 0;
    std::vector<uchar> lbpval;

    // iterate through a sequence using FileNodeIterator
    for( ; it != it_end; ++it, idx++ )
    {
        cout << "feature #" << idx << ": ";
        cout << "x=" << (int)(*it)["x"] << ", y=" << (int)(*it)["y"] << ", lbp: (";
        // you can also easily read numerical arrays using FileNode >> std::vector operator.
        (*it)["lbp"] >> lbpval;
        for( int i = 0; i < (int)lbpval.size(); i++ )
            cout << " " << (int)lbpval[i];
        cout << ")" << endl;
    }*/
    fs2.release();

    return ret;
}

static bool saveLinkageParams( const string& filename,
                       Size imageSize, const Mat& cameraMatrix_gun, const Mat& distCoeffs_gun,
                               const Mat& cameraMatrix_ball, const Mat& distCoeffs_ball, const Mat& homography)
{
    FileStorage fs( filename, FileStorage::WRITE );
    bool ret = fs.isOpened();
    if(!ret){
        cout << filename << " can't opened !\n" << endl;
        return ret;
    }

    time_t tt;
    time( &tt );
    struct tm *t2 = localtime( &tt );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_time" << buf;
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "camera_matrix_gun" << cameraMatrix_gun;
    fs << "distortion_coefficients_gun" << distCoeffs_gun;
    fs << "camera_matrix_ball" << cameraMatrix_ball;
    fs << "distortion_coefficients_ball" << distCoeffs_ball;
    fs << "homography" << homography;

    fs.release();

    return ret;
}

static bool linkage_findHomography(const Mat& img_1, const Mat& img_2,
                                   vector<Point2f>& corners_o1, vector<Point2f>& corners_o2,
                                   Mat& homography, double distThreshold = 30.0, bool bDraw = false)
{
    if(img_1.empty() || img_1.empty())
        return false;

    vector<Point2f> corners[2];

    //-- 初始化
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    //Ptr<FeatureDetector> detector = SIFT::create();
    //Ptr<DescriptorExtractor> descriptor = SIFT::create();
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // Ptr<FeatureDetector> detector = FeatureDetector::create(detector_name);
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create(descriptor_name);
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming(2)" );

    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    //Mat outimg1;
    //drawKeypoints( img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    //imshow("特征点",outimg1);

    if(descriptors_1.empty() || descriptors_2.empty())
        return false;

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> matches;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, matches );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = matches[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    // 仅供娱乐的写法
    //min_dist = min_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;
    //max_dist = max_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    std::vector< DMatch > good_matches;
    double curDistThreshold = min( (max ( 1.5*min_dist, 30.0 )), distThreshold);
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( matches[i].distance <= curDistThreshold )
        {
            good_matches.push_back ( matches[i] );
            corners[0].push_back(keypoints_1[matches[i].queryIdx].pt);
            corners[1].push_back(keypoints_2[matches[i].trainIdx].pt);
        }
    }

    //-- 第五步:绘制匹配结果
    if(bDraw){
        //Mat img_match;
        Mat img_goodmatch;
        //drawMatches ( img_1, keypoints_1, img_2, keypoints_2, matches, img_match );
        drawMatches ( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );
        //imshow ( "所有匹配点对", img_match );
        imshow ( "优化后匹配点对", img_goodmatch );
    }

    if(corners[0].empty() || corners[1].empty())
        return false;

    //Mat essential;
    //essential = findEssentialMat();

    const double ransac_thresh = 2.5f;
    Mat inlier_mask;
    vector<DMatch> inlier_matches;
    homography = findHomography(corners[0], corners[1], RANSAC, ransac_thresh, inlier_mask, 2000, 0.99);
    //homography = findHomography(corners[0], corners[1], RANSAC);
    //homography = findHomography(corners[0], corners[1], LMEDS);

    if(!homography.empty()){
        for(unsigned i = 0; i < corners[0].size(); i++) {
            if(inlier_mask.at<uchar>(i)) {
                int new_i = static_cast<int>(corners[0].size());
                corners_o1.push_back(corners[0][i]);
                corners_o2.push_back(corners[1][i]);
                inlier_matches.push_back(DMatch(new_i, new_i, 0));
            }
        }

        //return true;
        if(inlier_matches.size()>=4){
            Mat homography2 = findHomography(corners_o1, corners_o2, RANSAC);
            if(!homography2.empty()){
                homography = homography2.clone();
                return true;
            }
        }
    }

    return false;
}

int find_feature_matches ( const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches ,
                           double distThreshold = 30.0, bool bDraw = false)
{
    if(img_1.empty() || img_1.empty())
        return 0;

    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    if(descriptors_1.empty() || descriptors_2.empty())
        return 0;

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, match );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    //printf ( "-- Max dist : %f \n", max_dist );
    //printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    double curDistThreshold = min( (max ( 1.5*min_dist, 30.0 )), distThreshold);
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= curDistThreshold )
        {
            matches.push_back ( match[i] );
        }
    }

    //-- 第五步:绘制匹配结果
    if(bDraw){
        //Mat img_match;
        Mat img_match;
        drawMatches ( img_1, keypoints_1, img_2, keypoints_2, matches, img_match );
        resize(img_match, img_match, img_match.size()/2);
        imshow ( "匹配点对", img_match );
    }

    return matches.size();
}


Point2d pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d
           (
               ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
               ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
           );
}


void pose_2d2d ( std::vector<KeyPoint> keypoints_1,
                            std::vector<KeyPoint> keypoints_2,
                            std::vector< DMatch > matches,
                            const Mat& K,
                            Mat& R, Mat& t, Mat& H )
{
    // 相机内参,TUM Freiburg2
    //Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );

    //-- 把匹配点转换为vector<Point2f>的形式
    vector<Point2f> points1;
    vector<Point2f> points2;

    for ( int i = 0; i < ( int ) matches.size(); i++ )
    {
        points1.push_back ( keypoints_1[matches[i].queryIdx].pt );
        points2.push_back ( keypoints_2[matches[i].trainIdx].pt );
    }
#if 1
    //-- 计算基础矩阵
    Mat fundamental_matrix;
    //fundamental_matrix = findFundamentalMat ( points1, points2, CV_FM_8POINT );
    fundamental_matrix = findFundamentalMat ( points1, points2, FM_8POINT );
    //cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;

    //-- 计算本质矩阵
    Point2d principal_point(K.at<double>(0,2), K.at<double>(1,2));//( 325.1, 249.7 );	//相机光心, TUM dataset标定值
    double focal_length = K.at<double>(0,0);//521;			//相机焦距, TUM dataset标定值
    Mat essential_matrix;
    essential_matrix = findEssentialMat ( points1, points2, focal_length, principal_point );
    cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

    if(essential_matrix.rows == 3 && essential_matrix.cols == 3){
        //-- 从本质矩阵中恢复旋转和平移信息.
        recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point );
        cout<<"R is "<<endl<<R<<endl;
        cout<<"t is "<<endl<<t<<endl;
    }
#endif
    //-- 计算单应矩阵
    H = findHomography ( points1, points2, RANSAC, 3 );
    //cout<<"homography_matrix is "<<endl<<H<<endl;


}


int main(int argc, char** argv)
{
    Size boardSize;
    string gunCalibFile, ballCalibFile;
    string linkageCalibFile;
    int flags;
    bool showRectified;
    Mat cameraMatrix_gun, distCoeffs_gun;
    Mat cameraMatrix_ball, distCoeffs_ball;
    Size imageSize;
    cv::CommandLineParser parser(argc, argv,
                                 "{w|12|}{h|9|}{s|1.0|}{nr||}{help||}"
                                 "{@gun|gun_camera_data.yml|}"
                                 "{@ball|ball_camera_data.yml|}"
                                 "{@output|out_linkage_data.yml|}");
    if (parser.has("help"))
        return print_help();
    showRectified = !parser.has("nr");
    gunCalibFile = parser.get<string>("@gun");
    ballCalibFile = parser.get<string>("@ball");
    linkageCalibFile = parser.get<string>("@output");
    boardSize.width = parser.get<int>("w");
    boardSize.height = parser.get<int>("h");
    float squareSize = parser.get<float>("s");
    if (!parser.check())
    {
        parser.printErrors();
        return 1;
    }

    cout << " input param: " << boardSize.width << "x" << boardSize.height <<
            " showRectified " << showRectified << endl;

    loadCameraParams(gunCalibFile, flags, cameraMatrix_gun, distCoeffs_gun, imageSize);
    loadCameraParams(ballCalibFile, flags, cameraMatrix_ball, distCoeffs_ball, imageSize);

    Mat newCameraMatrix = cameraMatrix_ball;//getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1.0, imageSize);
    Mat map1, map2;
    initUndistortRectifyMap(cameraMatrix_gun, distCoeffs_gun, Mat(),
                            newCameraMatrix,
                            imageSize, CV_16SC2, map1, map2);


    VideoCapture capGun(0);
    if (!capGun.isOpened())
    {
        return -1;
    }

    VideoCapture capBall(1);
    if (!capBall.isOpened())
    {
        return -1;
    }

    double framewidth = imageSize.width;
    double frameheight = imageSize.height;
    capGun.set(CV_CAP_PROP_FRAME_WIDTH, framewidth);
    capGun.set(CV_CAP_PROP_FRAME_HEIGHT, frameheight);
    capBall.set(CV_CAP_PROP_FRAME_WIDTH, framewidth);
    capBall.set(CV_CAP_PROP_FRAME_HEIGHT, frameheight);

    printf( "\n%s", liveCaptureHelp );

    Mat homography;
    Mat org[2];
    Mat frame[2];
    bool undistortImage = true;
    bool bCal = false;
    vector<Point2f> pts;

    int i;
    unsigned long nFrame = 0;
    for(;;)
    {
        capGun >> org[0];
        capBall >> org[1];
        frame[0] = org[0];
        frame[1] = org[1];

        char key = (char)waitKey(1);
        if(key == 's' && !homography.empty()){
            saveLinkageParams(linkageCalibFile, org[1].size(), cameraMatrix_gun, distCoeffs_gun,
                    cameraMatrix_ball, distCoeffs_ball, homography);
        }
        if( key == 'g' ){
            bCal = true;
        }
        if (key == 27)
               break;

        if(org[0].empty()){
            continue;
        }

        {
            Mat imgDraw[2];
            resize(org[0], imgDraw[0], Size(org[0].cols/2, org[0].rows/2));
            resize(org[1], imgDraw[1], Size(org[1].cols/2, org[1].rows/2));
            imshow("gun org", imgDraw[0]);
            imshow("ball org", imgDraw[1]);
        }

        if(org[1].empty())
            continue;

        //
        if( undistortImage )
        {
            frame[0] = org[0].clone();
            //undistort(org[0], frame[0], cameraMatrix_gun, distCoeffs_gun, newCameraMatrix);
            remap(org[0], frame[0], map1, map2, INTER_LINEAR);
            //imshow("gun undistort", frame[0]);
        }

        vector<KeyPoint> keypoints_1, keypoints_2;
        vector<DMatch> matches;
        find_feature_matches ( frame[0], frame[1], keypoints_1, keypoints_2, matches , 60.0, true);

        if(matches.size()<= 4)
            continue;

        if(bCal){
            Mat R,t;
            pose_2d2d ( keypoints_1, keypoints_2, matches, newCameraMatrix, R, t, homography);
            pts.clear();
            for(int i=0; i<matches.size(); ++i)
                pts.push_back(keypoints_2[matches[i].trainIdx].pt);
            bCal = false;
            cout << "match points " << matches.size() << endl;
        }

        if(!homography.empty()){
            Mat warp;
            warpPerspective(frame[0], warp, homography, frame[0].size());
            drawChessboardCorners(warp, boardSize, pts, false);
            resize(warp, warp, warp.size()/2);
            imshow("camera gun warp", warp);
        } 

        nFrame ++;
    }

    return 0;
}
