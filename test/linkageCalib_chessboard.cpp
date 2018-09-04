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
    bool found[2];
    bool undistortImage = true;
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
        if (key == 27)
               break;

        if(org[0].empty()){
            continue;
        }

        imshow("gun org", org[0]);
        imshow("ball org", org[1]);

        if(org[1].empty())
            continue;

        //
        if( undistortImage )
        {
            frame[0] = org[0].clone();
            //undistort(org[0], frame[0], cameraMatrix, distCoeffs);
            Size imageSize = org[0].size();
            Mat map1, map2;
            initUndistortRectifyMap(cameraMatrix_gun, distCoeffs_gun, Mat(),
                                    newCameraMatrix,
                                    imageSize, CV_16SC2, map1, map2);
            remap(org[0], frame[0], map1, map2, INTER_LINEAR);
            imshow("gun undistort", frame[0]);
        }

        //if(frame[0].size() != frame[1].size()){
        //    resize(frame[0], frame[0], frame[1].size());
        //}

        Mat imgGray;
        vector<Point2f> corners[2];

        cvtColor(frame[0], imgGray, CV_RGB2GRAY);
        int scale = 1;
        Mat timg;
        if( scale == 1 )
            timg = imgGray;
        else
            resize(imgGray, timg, Size(), scale, scale);
        found[0] = findChessboardCorners(timg, boardSize, corners[0], CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
        if( scale > 1 )
        {
            Mat cornersMat(corners[0]);
            cornersMat *= 1./scale;
        }
        if( found[0] )
        {
            cornerSubPix(imgGray, corners[0], Size(11,11), Size(-1,-1),
                         TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,
                                      30, 0.01));
        }

        cvtColor(frame[1], imgGray, CV_RGB2GRAY);
        found[1] = findChessboardCorners(imgGray, boardSize, corners[1], CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
        if(found[1]){
            cornerSubPix(imgGray, corners[1], Size(11,11), Size(-1,-1),
                         TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,
                                      30, 0.01));
        }

        if(found[0] && found[1]){
            const double ransac_thresh = 2.5f;
            Mat inlier_mask;
            vector<Point2f> inliers[2];
            vector<DMatch> inlier_matches;
            homography = findHomography(corners[0], corners[1], RANSAC, ransac_thresh, inlier_mask);
            //homography = findHomography(corners[0], corners[1], RANSAC);
            //homography = findHomography(corners[0], corners[1], LMEDS);
            if(!homography.empty()){
                for(unsigned i = 0; i < corners[0].size(); i++) {
                    if(inlier_mask.at<uchar>(i)) {
                        int new_i = static_cast<int>(inliers[0].size());
                        inliers[0].push_back(corners[0][i]);
                        inliers[1].push_back(corners[1][i]);
                        inlier_matches.push_back(DMatch(new_i, new_i, 0));
                    }
                }

                Mat warp;
                if(inlier_matches.size()>=4){
                    Mat homography2 = findHomography(inliers[0], inliers[1], RANSAC);
                    if(!homography2.empty())
                        homography = homography2.clone();
                }

                warpPerspective(frame[0], warp, homography, frame[0].size());
                if(!warp.empty()){
                    //perspectiveTransform(object_bb, new_bb, homography);
                    //drawChessboardCorners(warp, boardSize, corners[1], found[1]);
                    drawChessboardCorners(warp, boardSize, inliers[1], false);
                    imshow("camera gun warp", warp);
                }
            }
        }

        if(showRectified){
            drawChessboardCorners(frame[0], boardSize, corners[0], found[0]);
            drawChessboardCorners(frame[1], boardSize, corners[1], found[1]);
            imshow("camera gun calibration", frame[0]);
            imshow("camera ball calibration", frame[1]);
        }
    }

    return 0;
}
