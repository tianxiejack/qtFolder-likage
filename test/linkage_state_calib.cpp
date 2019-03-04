
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

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "HCNetSDK.h"

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

typedef struct uSelect
{
    enum Mode
    {
        CLICK       = 1,
        BOX         = 2,
    };

    Mode mode;
    Point pt;
    Rect  rt;
    bool stat;
    bool valid;
    uSelect() {
        pt = Point(-1, -1);
        rt = Rect();
        mode = CLICK;
        stat = false;
        valid = false;
    };

}uSelect;

void on_mouse(int event, int x, int y, int flags, void *ustc)
{
    static uSelect usel;
    //char temp[16];
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        usel.pt = Point(x, y);
        usel.rt.x = x;
        usel.rt.y = y;
        usel.stat = false;
        usel.valid = true;
    }
    else if (event == CV_EVENT_MOUSEMOVE && !(flags & CV_EVENT_FLAG_LBUTTON))
    {
    }
    else if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON))
    {
        usel.pt = Point(x, y);
        usel.rt.width = x - usel.rt.x;
        usel.rt.height = y - usel.rt.y;
    }
    else if (event == CV_EVENT_LBUTTONUP)
    {
        usel.stat = true;
    }

    if(ustc != NULL)
        memcpy(ustc, &usel, sizeof(uSelect));
}

LONG hc_init()
{
    LONG lUserID = -1;
    NET_DVR_Init();
    NET_DVR_USER_LOGIN_INFO struLoginInfo = {0};
    NET_DVR_DEVICEINFO_V40 struDeviceInfo = {0};
    strcpy((char *)struLoginInfo.sDeviceAddress,"192.168.0.64"); //设备 IP 地址
    strcpy((char *)struLoginInfo.sUserName,"admin");
    strcpy((char *)struLoginInfo.sPassword,"admin$2018");
    //设备登录用户名
    //设备登录密码
    struLoginInfo.wPort = 8000;
    struLoginInfo.bUseAsynLogin = 0; //同步登录,登录接口返回成功即登录成功
    lUserID = NET_DVR_Login_V40(&struLoginInfo, &struDeviceInfo);
    if (lUserID < 0)
    {
    printf("NET_DVR_Login_V40 failed, error code: %d\n", NET_DVR_GetLastError());
    NET_DVR_Cleanup();
    }
    else
    {
        printf("NET_DVR_Login_V40 success, userID = %d\n", lUserID);
    }
    return lUserID;
}

void hc_uninit(LONG lUserId)
{
    if(lUserId>0)
    {
        NET_DVR_Logout(lUserId);
        NET_DVR_Cleanup();
    }
}
typedef enum{
    HCMVMD_STOP = 0x0000,
    HCMVMD_UP = 0x0001,
    HCMVMD_DOWN = 0x0002,
    HCMVMD_LEFT = 0x0004,
    HCMVMD_UPLEFT = 0x0005,
    HCMVMD_DOWNLEFT = 0x0006,
    HCMVMD_RIGHT = 0x0008,
    HCMVMD_UPRIGHT = 0x0009,
    HCMVMD_DOWNRIGHT = 0x000a,
    HCMVMD_ZOOMIN = 0x0010,
    HCMVMD_ZOOMOUT = 0x0020,
}HCMVMD;

bool hc_move(LONG lUserId, HCMVMD md, DWORD speed)
{
    //return true;
    static HCMVMD curMd = HCMVMD_STOP;
    bool ret = false;

    if(curMd != HCMVMD_STOP){
        switch (curMd) {
        case HCMVMD_UP:
            ret = NET_DVR_PTZControlWithSpeed_Other(lUserId, 1, TILT_UP, 1, speed);
            break;
        case HCMVMD_DOWN:
            ret = NET_DVR_PTZControlWithSpeed_Other(lUserId, 1, TILT_DOWN, 1, speed);
            break;
        case HCMVMD_LEFT:
            ret = NET_DVR_PTZControlWithSpeed_Other(lUserId, 1, PAN_LEFT, 1, speed);
            break;
        case HCMVMD_RIGHT:
            ret = NET_DVR_PTZControlWithSpeed_Other(lUserId, 1, PAN_RIGHT, 1, speed);
            break;
        case HCMVMD_UPLEFT:
            ret = NET_DVR_PTZControlWithSpeed_Other(lUserId, 1, UP_LEFT, 1, speed);
            break;
        case HCMVMD_UPRIGHT:
            ret = NET_DVR_PTZControlWithSpeed_Other(lUserId, 1, UP_RIGHT, 1, speed);
            break;
        case HCMVMD_DOWNLEFT:
            ret = NET_DVR_PTZControlWithSpeed_Other(lUserId, 1, DOWN_LEFT, 1, speed);
            break;
        case HCMVMD_DOWNRIGHT:
            ret = NET_DVR_PTZControlWithSpeed_Other(lUserId, 1, DOWN_RIGHT, 1, speed);
            break;
        case HCMVMD_ZOOMIN:
            ret = NET_DVR_PTZControlWithSpeed_Other(lUserId, 1, ZOOM_IN, 1, speed);
            break;
        case HCMVMD_ZOOMOUT:
            ret = NET_DVR_PTZControlWithSpeed_Other(lUserId, 1, ZOOM_OUT, 1, speed);
            break;
        default:
            break;
        }
    }

    curMd = md;

    if(curMd != HCMVMD_STOP){
        switch (curMd) {
        case HCMVMD_UP:
            ret = NET_DVR_PTZControlWithSpeed_Other(lUserId, 1, TILT_UP, 0, speed);
            break;
        case HCMVMD_DOWN:
            ret = NET_DVR_PTZControlWithSpeed_Other(lUserId, 1, TILT_DOWN, 0, speed);
            break;
        case HCMVMD_LEFT:
            ret = NET_DVR_PTZControlWithSpeed_Other(lUserId, 1, PAN_LEFT, 0, speed);
            break;
        case HCMVMD_RIGHT:
            ret = NET_DVR_PTZControlWithSpeed_Other(lUserId, 1, PAN_RIGHT, 0, speed);
            break;
        case HCMVMD_UPLEFT:
            ret = NET_DVR_PTZControlWithSpeed_Other(lUserId, 1, UP_LEFT, 0, speed);
            break;
        case HCMVMD_UPRIGHT:
            ret = NET_DVR_PTZControlWithSpeed_Other(lUserId, 1, UP_RIGHT, 0, speed);
            break;
        case HCMVMD_DOWNLEFT:
            ret = NET_DVR_PTZControlWithSpeed_Other(lUserId, 1, DOWN_LEFT, 0, speed);
            break;
        case HCMVMD_DOWNRIGHT:
            ret = NET_DVR_PTZControlWithSpeed_Other(lUserId, 1, DOWN_RIGHT, 0, speed);
            break;
        case HCMVMD_ZOOMIN:
            ret = NET_DVR_PTZControlWithSpeed_Other(lUserId, 1, ZOOM_IN, 0, speed);
            break;
        case HCMVMD_ZOOMOUT:
            ret = NET_DVR_PTZControlWithSpeed_Other(lUserId, 1, ZOOM_OUT, 0, speed);
            break;
        default:
            break;
        }
    }

    printf("ptz move %x speed %d\n", (int)curMd, speed);

    return ret;
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
                               const Mat& cameraMatrix_ball, const Mat& distCoeffs_ball, const Mat& homography,
                               int panPos, int tiltPos, int zoomPos)
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
    fs << "ballPanPos" << panPos;
    fs << "ballTiltPos" << tiltPos;
    fs << "ballZoomPos" << zoomPos;

    fs.release();

    return ret;
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
    LONG hcDev = -1;
    DWORD mvSpeed = 2;
    NET_DVR_PTZPOS pos;
    HCMVMD mvMd = HCMVMD_STOP, mvMdBak = HCMVMD_STOP;

    cv::CommandLineParser parser(argc, argv,
                                 "{s|0.5|}{help||}"
                                 "{@gun|gun_camera_data.yml|}"
                                 "{@ball|ball_camera_data.yml|}"
                                 "{@output|out_linkage_data.yml|}"
                                 "{@input_ball|rtsp://admin:admin$2018@192.168.0.64:554|}"
                                 "{@input_gun|rtsp://admin:admin$2018@192.168.0.65:554|}");
    if (parser.has("help"))
        return print_help();
    float scale = parser.get<float>("s");
    string strURL_ball = parser.get<string>("@input_ball");
    string strURL_gun  = parser.get<string>("@input_gun");
    string gunCalibFile, ballCalibFile, linkageCalibFile;
    gunCalibFile = parser.get<string>("@gun");
    ballCalibFile = parser.get<string>("@ball");
    linkageCalibFile = parser.get<string>("@output");
    Size boardSize;

    hcDev = hc_init();
/*
    VideoCapture cap_ball;
    cap_ball.open(strURL_ball);
    if (!cap_ball.isOpened())
    {
        return -1;
    }

    VideoCapture cap_gun;
    cap_gun.open(strURL_gun);
    if (!cap_gun.isOpened())
    {
        return -1;
    }
*/


    int flags;
    Mat cameraMatrix_gun, distCoeffs_gun;
    Mat cameraMatrix_ball, distCoeffs_ball;
    Size imageSize;
    loadCameraParams(gunCalibFile, flags, cameraMatrix_gun, distCoeffs_gun, imageSize);
    loadCameraParams(ballCalibFile, flags, cameraMatrix_ball, distCoeffs_ball, imageSize);

    Mat newCameraMatrix = cameraMatrix_ball;//getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1.0, imageSize);
    Mat map1, map2;
    initUndistortRectifyMap(cameraMatrix_gun, distCoeffs_gun, Mat(),
                            newCameraMatrix,
                            imageSize, CV_16SC2, map1, map2);

    Mat image_ball = imread("ball.bmp");
    Mat image_gun = imread("gun.bmp");

    Mat undisImage;
    //undistort(image, undisImage, cameraMatrix_gun, distCoeffs_gun, newCameraMatrix);
    //remap(image, undisImage, map1, map2, INTER_LINEAR);

    unsigned long nframe = 0;
    uSelect sel;
    string strWinName;
    bool bCal = false;
    Mat homography;
    vector<Point2f> pts;
    for(;;)
    {
        Mat frame_ball,frame_gun;
        //cap_ball >> image_ball;
frame_ball = image_ball;
frame_gun = image_gun;
       // cap_gun >> frame_gun;

        remap(frame_gun, undisImage, map1, map2, INTER_LINEAR);

        Mat gunDraw, ballDraw;
        resize(frame_gun, gunDraw, Size(frame_gun.cols*scale, frame_gun.rows*scale));
        resize(image_ball, ballDraw, Size(image_ball.cols*scale, image_ball.rows*scale));
        imshow("gun org", gunDraw);
        imshow("ball org", ballDraw);

        if(bCal){
            vector<KeyPoint> keypoints_1, keypoints_2;
            vector<DMatch> matches;
            find_feature_matches ( undisImage, image_ball, keypoints_1, keypoints_2, matches , 60.0, true);
            if(matches.size() > 4){
                Mat R,t;
                pose_2d2d ( keypoints_1, keypoints_2, matches, newCameraMatrix, R, t, homography);
                pts.clear();
                for(int i=0; i<matches.size(); ++i)
                    pts.push_back(keypoints_2[matches[i].trainIdx].pt);
                bCal = false;
                cout << "match points " << matches.size() << endl;
            }
        }
        if(!homography.empty()){
            Mat warp;
            warpPerspective(undisImage, warp, homography, undisImage.size());
            drawChessboardCorners(warp, boardSize, pts, false);
            resize(warp, warp, Size(warp.cols*scale, warp.rows*scale));
            imshow("camera gun warp", warp);
        }

        if(nframe == 0)
            setMouseCallback("ball", on_mouse, &sel);
        nframe ++;

        char key = (char)waitKey(10);
        mvMdBak = mvMd;
        if(key == 82){//up
            mvMd = HCMVMD_UP;
            hc_move(hcDev,mvMd,mvSpeed);
        }
        else if(key == 84){//down
            mvMd = HCMVMD_DOWN;
            hc_move(hcDev,mvMd,mvSpeed);
        }
        else if(key == 81){//left
            mvMd = HCMVMD_LEFT;
            hc_move(hcDev,mvMd,mvSpeed);
        }
        else if(key == 83){//right
            mvMd = HCMVMD_RIGHT;
            hc_move(hcDev,mvMd,mvSpeed);
        }
        else if(key == -107){//home
            mvMd = HCMVMD_UPLEFT;
            hc_move(hcDev,mvMd,mvSpeed);
        }
        else if(key == -102){//pgup
            mvMd = HCMVMD_UPRIGHT;
            hc_move(hcDev,mvMd,mvSpeed);
        }
        else if(key == -101){//pgdn
            mvMd = HCMVMD_DOWNRIGHT;
            hc_move(hcDev,mvMd,mvSpeed);
        }
        else if(key == -100){//end
            mvMd = HCMVMD_DOWNLEFT;
            hc_move(hcDev,mvMd,mvSpeed);
        }
        else if(key == 'e'){//
            mvSpeed ++;
            hc_move(hcDev,mvMd,mvSpeed);
        }
        else if(key == 'd'){//
            mvSpeed --;
            hc_move(hcDev,mvMd,mvSpeed);
        }
        else if(key == -83){//-
            //mvMd = (HCMVMD)(mvMd | HCMVMD_ZOOMIN);
            mvMd = HCMVMD_ZOOMIN;
            hc_move(hcDev,mvMd,mvSpeed);
        }
        else if(key == -85){//+
            //mvMd = (HCMVMD)(mvMd | HCMVMD_ZOOMOUT);
            mvMd = HCMVMD_ZOOMOUT;
            hc_move(hcDev,mvMd,mvSpeed);
        }
        else if(key == -1 || key == 'r' || key == 'g'){}
        else{
            printf("%c[%d] \n", key, key);
            hc_move(hcDev,HCMVMD_STOP,0);
            mvMd = HCMVMD_STOP;
        }
        if (key == 27)
               break;
        if((mvMd == HCMVMD_STOP && mvMdBak != HCMVMD_STOP) || key == 'r' || key == 's'){
            DWORD len;
            NET_DVR_GetDVRConfig(hcDev, NET_DVR_GET_PTZPOS,1, &pos, sizeof(pos), &len);
            cout << "ptz pos : action[ " << pos.wAction
                 << " ] pan[ " << pos.wPanPos
                 << " ] tilt [ " << pos.wTiltPos
                 << " ] zoom [ " << pos.wZoomPos << " ]" << endl;
        }
        if(key == 'g'){
            bCal = true;
        }
        if(key == 's' && !homography.empty()){
            saveLinkageParams(linkageCalibFile, imageSize, cameraMatrix_gun, distCoeffs_gun,
                    cameraMatrix_ball, distCoeffs_ball, homography,
                    pos.wPanPos, pos.wTiltPos, pos.wZoomPos);
            char filename[200] = "ballLinkageCalib.bmp";
            imwrite(filename, frame_ball);
        }
    }

    hc_uninit(hcDev);

    return 0;
}
