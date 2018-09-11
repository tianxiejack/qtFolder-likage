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
    bool notify;
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
    uSelect *usel = (uSelect *)ustc;
    //char temp[16];
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        usel->pt = Point(x, y);
        usel->rt.x = x;
        usel->rt.y = y;
        usel->stat = false;
        usel->valid = true;
        usel->notify = true;
    }
    else if (event == CV_EVENT_MOUSEMOVE && !(flags & CV_EVENT_FLAG_LBUTTON))
    {
    }
    else if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON))
    {
        usel->pt = Point(x, y);
        usel->rt.width = x - usel->rt.x;
        usel->rt.height = y - usel->rt.y;
        usel->notify = true;
    }
    else if (event == CV_EVENT_LBUTTONUP)
    {
        usel->stat = true;
    }
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

static bool loadLinkageParams( const string& filename,
                       Size& imageSize, Mat& cameraMatrix_gun, Mat& distCoeffs_gun,
                               Mat& cameraMatrix_ball, Mat& distCoeffs_ball, Mat& homography,
                               int& panPos, int& tiltPos, int& zoomPos)
{
    FileStorage fs2( filename, FileStorage::READ );

    bool ret = fs2.isOpened();
    if(!ret){
        cout << filename << " can't opened !\n" << endl;
    }

    String date;
    fs2["calibration_time"] >> date;
    int imgwidth = (int)fs2["image_width"];
    int imgheight = (int)fs2["image_height"];

    fs2["camera_matrix_gun"] >> cameraMatrix_gun;
    fs2["distortion_coefficients_gun"] >> distCoeffs_gun;
    fs2["camera_matrix_ball"] >> cameraMatrix_ball;
    fs2["distortion_coefficients_ball"] >> distCoeffs_ball;
    fs2["homography"] >> homography;
    fs2["ballPanPos"] >> panPos;
    fs2["ballTiltPos"] >> tiltPos;
    fs2["ballZoomPos"] >> zoomPos;

    cout << "calibration date: " << date << endl
         << "image width:      " << imgwidth << endl
         << "image height:     " << imgheight << endl
         << "camera matrix gun:\n" << cameraMatrix_gun << endl
         << "distortion coeffs gun:\n" << distCoeffs_gun << endl
         << "camera matrix ball:\n" << cameraMatrix_ball << endl
         << "distortion coeffs ball:\n" << distCoeffs_ball << endl
         << "homography matrix:\n" << homography << endl
         << "ballPanPos:\n" << panPos << endl
         << "ballTiltPos:\n" << tiltPos << endl
         << "ballZoomPos:\n" << zoomPos << endl;

    imageSize.width = imgwidth;
    imageSize.height = imgheight;
    fs2.release();

    return ret;
}

int main(int argc, char** argv)
{
    LONG hcDev = -1;
    DWORD mvSpeed = 2;
    HCMVMD mvMd = HCMVMD_STOP, mvMdBak = HCMVMD_STOP;
    NET_DVR_PTZPOS pos;

    cv::CommandLineParser parser(argc, argv,
                                 "{s|0.5|}{help||}"
                                 "{@gun|gun_camera_data.yml|}"
                                 "{@ball|ball_camera_data.yml|}"
                                 "{@output|out_linkage_data.yml|}"
                                 "{@input|rtsp://admin:admin$2018@192.168.0.64:554|}");
    if (parser.has("help"))
        return print_help();
    float scale = parser.get<float>("s");
    string strURL = parser.get<string>("@input");

    string gunCalibFile, ballCalibFile, linkageCalibFile;
    gunCalibFile = parser.get<string>("@gun");
    ballCalibFile = parser.get<string>("@ball");
    linkageCalibFile = parser.get<string>("@output");
    Size boardSize;

    hcDev = hc_init();

    VideoCapture cap;
    cap.open(strURL);
    if (!cap.isOpened())
    {
        return -1;
    }

    Mat cameraMatrix_gun, distCoeffs_gun;
    Mat cameraMatrix_ball, distCoeffs_ball;
    Mat homography;
    Size imageSize;
    int panPosBase, tiltPosBase, zoomPosBase;
    if(!loadLinkageParams(linkageCalibFile, imageSize, cameraMatrix_gun, distCoeffs_gun,
                          cameraMatrix_ball, distCoeffs_ball, homography,
                          panPosBase, tiltPosBase, zoomPosBase)){
        cout << " load " << linkageCalibFile << " linkage params false! " << endl;
        return -1;
    }

    Mat map1, map2;
    initUndistortRectifyMap(cameraMatrix_gun, distCoeffs_gun, Mat(),
                            cameraMatrix_ball,
                            imageSize, CV_16SC2, map1, map2);
    Mat imageGun = imread("images/2018-08-16-153720.jpg");
    Mat imageBall = imread("ballLinkageCalib.bmp");
    Mat undisImageGun;
    remap(imageGun, undisImageGun, map1, map2, INTER_LINEAR);

    namedWindow( "gun image", 1 );

    unsigned long nframe = 0;
    uSelect sel_gun;
    uSelect sel_ball;
    vector<Point2f> pts;
    for(;;)
    {
        Mat frame;
        Mat gunImageDraw, ballImageDraw, ballVideoDraw;
        cap >> frame;

        resize(frame, ballVideoDraw, Size(frame.cols*scale, frame.rows*scale));

        if(nframe == 0){
            setMouseCallback("ball video", on_mouse, &sel_gun);//sel_gun
            setMouseCallback("ball image", on_mouse, &sel_ball);//sel_ball
        }

        resize(imageGun, gunImageDraw, Size(imageGun.cols, imageGun.rows));
        resize(imageBall, ballImageDraw, Size(imageBall.cols*scale, imageBall.rows*scale));

        drawMarker(gunImageDraw, Point(gunImageDraw.cols/2, gunImageDraw.rows/2), Scalar(255, 255, 255, 0), MARKER_CROSS, 50);
        drawMarker(ballImageDraw, Point(ballImageDraw.cols/2, ballImageDraw.rows/2), Scalar(255, 255, 255, 0), MARKER_CROSS, 50);
        drawMarker(ballVideoDraw, Point(ballVideoDraw.cols/2, ballVideoDraw.rows/2), Scalar(255, 255, 255, 0), MARKER_CROSS, 50);

        if(sel_gun.valid){
            Point opt( sel_gun.pt.x, sel_gun.pt.y );
            std::vector<cv::Point2d> distorted, normalizedUndistorted;
            distorted.push_back(cv::Point2d(opt.x, opt.y));
            undistortPoints(distorted,normalizedUndistorted,cameraMatrix_gun,distCoeffs_gun);
            std::vector<cv::Point3d> objectPoints;
            for (std::vector<cv::Point2d>::const_iterator itPnt = normalizedUndistorted.begin();
                itPnt != normalizedUndistorted.end(); ++itPnt)
                objectPoints.push_back(cv::Point3d(itPnt->x, itPnt->y, 1));
            std::vector<cv::Point2d> imagePoints(objectPoints.size());
            projectPoints(objectPoints,
                cv::Vec3d(0,0,0),cv::Vec3d(0,0,0),
                cameraMatrix_ball,cv::Mat(),imagePoints);

            std::vector<cv::Point2d> ballImagePoints(imagePoints.size());
            perspectiveTransform(imagePoints, ballImagePoints, homography);

            std::vector<cv::Point2d>::iterator itp = imagePoints.begin();
            cv::Point2d pt = *itp;
            Point upt( pt.x, pt.y );
            itp = ballImagePoints.begin();
            pt = *itp;
            Point bpt( pt.x, pt.y );

            circle(gunImageDraw, opt, 3, Scalar(0, 0, 255, 0), 1, CV_AA, 0);
            circle(undisImageGun, upt*scale, 3, Scalar(0, 0, 255, 0), CV_FILLED, CV_AA, 0);
            circle(ballImageDraw, bpt*scale, 3, Scalar(0, 0, 255, 0), 1, CV_AA, 0);

            if(sel_gun.notify){
                sel_gun.notify = false;
                Point2f nzbpt = Point2f(bpt.x-imageBall.cols/2, bpt.y-imageBall.rows/2);
                cout << "opt: " << opt.x << " , " << opt.y << endl;
                cout << "upt: " << upt.x << " , " << upt.y << endl;
                cout << "bpt: " << bpt.x << " , " << bpt.y << endl;
                cout << "nzbpt: " << nzbpt.x << " , " << nzbpt.y << endl;

                NET_DVR_PTZPOS pos2;
                pos2.wAction = 1;
                int temp1 = panPosBase+nzbpt.x*0.86;
                pos2.wPanPos = (temp1<0) ? (13720+temp1) : temp1;
                int temp2 = tiltPosBase+nzbpt.y*0.95;
                pos2.wTiltPos = (temp2<0) ? (13720+temp2) : temp2;
                pos2.wZoomPos = 100;//zoomPosBase;
                NET_DVR_SetDVRConfig(hcDev, NET_DVR_SET_PTZPOS,1, &pos2, sizeof(pos2));
                cout << "t1 " << temp1 << " t2 " << temp2 << endl;
                cout << "wPanPos " << pos2.wPanPos << " wTiltPos " << pos2.wTiltPos << " wZoomPos " << pos2.wZoomPos << endl;
            }
        }

        imshow("gun image", gunImageDraw);
        imshow("ball image", ballImageDraw);
        imshow("ball video", ballVideoDraw);

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
            //mvMd = (HCMVMD)(mvMd | HCMVMD_ZOOMOUT);
            mvMd = HCMVMD_ZOOMOUT;
            hc_move(hcDev,mvMd,mvSpeed);
        }
        else if(key == -85){//+
            //mvMd = (HCMVMD)(mvMd | HCMVMD_ZOOMIN);
            mvMd = HCMVMD_ZOOMIN;
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
        if((mvMd == HCMVMD_STOP && mvMdBak != HCMVMD_STOP) || key == 'r' || nframe == 0){
            DWORD len;
            NET_DVR_GetDVRConfig(hcDev, NET_DVR_GET_PTZPOS,1, &pos, sizeof(pos), &len);
            cout << "ptz pos : action[ " << pos.wAction
                 << " ] pan[ " << pos.wPanPos
                 << " ] tilt [ " << pos.wTiltPos
                 << " ] zoom [ " << pos.wZoomPos << " ]" << endl;
        }

        nframe ++;
    }

    hc_uninit(hcDev);

    return 0;
}
