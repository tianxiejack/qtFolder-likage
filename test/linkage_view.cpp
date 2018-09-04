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

static bool loadLinkageParams( const string& filename,
                       Size& imageSize, Mat& cameraMatrix_gun, Mat& distCoeffs_gun,
                               Mat& cameraMatrix_ball, Mat& distCoeffs_ball, Mat& homography)
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

    cout << "calibration date: " << date << endl
         << "image width:      " << imgwidth << endl
         << "image height:     " << imgheight << endl
         << "camera matrix gun:\n" << cameraMatrix_gun << endl
         << "distortion coeffs gun:\n" << distCoeffs_gun << endl
         << "camera matrix ball:\n" << cameraMatrix_ball << endl
         << "distortion coeffs ball:\n" << distCoeffs_ball << endl
         << "homography matrix:\n" << homography << endl;

    imageSize.width = imgwidth;
    imageSize.height = imgheight;
    fs2.release();

    return ret;
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

int main(int argc, char** argv)
{
    Size boardSize;
    string linkageCalibFile;
    bool showRectified;
    Mat cameraMatrix_gun, distCoeffs_gun, cameraMatrix_ball, distCoeffs_ball, homography;
    Size imageSize;
    cv::CommandLineParser parser(argc, argv,
                                 "{w|12|}{h|9|}{s|1.0|}{nr|1|}{help||}{@input|out_linkage_data.yml|}");
    if (parser.has("help"))
        return print_help();
    showRectified = !parser.has("nr");
    linkageCalibFile = parser.get<string>("@input");
    boardSize.width = parser.get<int>("w");
    boardSize.height = parser.get<int>("h");
    //float squareSize = parser.get<float>("s");
    if (!parser.check())
    {
        parser.printErrors();
        return 1;
    }

    cout << " input param: " << boardSize.width << "x" << boardSize.height <<
            " showRectified " << showRectified << endl;

    if(!loadLinkageParams(linkageCalibFile, imageSize, cameraMatrix_gun, distCoeffs_gun,cameraMatrix_ball, distCoeffs_ball, homography)){
        cout << " load " << linkageCalibFile << " linkage params false! " << endl;
        return -1;
    }

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

    Mat org[2];
    Mat frame[2];
    bool found[2];
    bool undistortImage = true;
    unsigned long nframe = 0;
    uSelect sel;

    namedWindow( "gun", 1 );

    Mat map1, map2;
    if( undistortImage )
    {
        initUndistortRectifyMap(cameraMatrix_gun, distCoeffs_gun, Mat(),
                                cameraMatrix_ball,
                                imageSize, CV_16SC2, map1, map2);
    }


    for(;;)
    {
        capGun >> org[0];
        capBall >> org[1];
        frame[0] = org[0].clone();
        frame[1] = org[1];

        if(org[0].empty()){
            continue;
        }

        if(nframe == 0)
            setMouseCallback("gun", on_mouse, &sel);
        nframe ++;


        if(org[1].empty())
            continue;

        if( !map1.empty() && !map2.empty())
        {
            remap(frame[0], frame[0], map1, map2, INTER_LINEAR, BORDER_CONSTANT);

            /*if(sel.valid){
                typedef Vec<int16_t, 2> Vec2myi;
                Point opt(sel.pt.x, sel.pt.y);
                Point upt( (int)(map1.at<Vec2myi>(opt.y, opt.x)[0]), (int)(map1.at<Vec2myi>(opt.y, opt.x)[1]) );
                //circle(org[0], sel.pt, 3, Scalar(0, 0, 255, 0), CV_FILLED, CV_AA, 0);
                //circle(frame[0], upt, 3, Scalar(0, 0, 255, 0), CV_FILLED, CV_AA, 0);
                circle(frame[0], opt, 3, Scalar(0, 0, 255, 0), CV_FILLED, CV_AA, 0);
                circle(org[0], upt, 3, Scalar(0, 0, 255, 0), CV_FILLED, CV_AA, 0);
            }*/
        }

        if(sel.valid){
            Point opt( sel.pt.x*2, sel.pt.y*2 );
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

            circle(org[0], opt, 3, Scalar(0, 0, 255, 0), CV_FILLED, CV_AA, 0);
            circle(frame[0], upt, 3, Scalar(0, 0, 255, 0), CV_FILLED, CV_AA, 0);
            circle(frame[1], bpt, 3, Scalar(0, 0, 255, 0), CV_FILLED, CV_AA, 0);
        }
        {
            Mat imgDraw[3];
            resize(org[0], imgDraw[0], org[0].size()/2);
            resize(org[1], imgDraw[1], org[1].size()/2);
            resize(frame[0], imgDraw[2], frame[0].size()/2);
            imshow("gun", imgDraw[0]);
            imshow("ball", imgDraw[1]);
            imshow("gun undistort", imgDraw[2]);
        }

        char key = (char)waitKey(1);
        if (key == 27)
               break;
    }

    return 0;
}
