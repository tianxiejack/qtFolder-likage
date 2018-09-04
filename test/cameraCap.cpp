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

int main(int argc, char** argv)
{

    int cameraId = 0;
    string inputFilename = "";
#if 0
    cv::CommandLineParser parser(argc, argv,
                                 "{s|0.5|}{help||}{@input|rtsp://admin:admin$2018@192.168.0.64:554|}");
#else
    cv::CommandLineParser parser(argc, argv,
                               "{s|1.0|}{help||}{@input|0|}");
#endif
    if (parser.has("help"))
        return print_help();
    float scale = parser.get<float>("s");
    if ( isdigit(parser.get<string>("@input")[0]) )
        cameraId = parser.get<int>("@input");
    else
        inputFilename = parser.get<string>("@input");

    VideoCapture cap;
    if( !inputFilename.empty() )
    {
        cap.open(inputFilename);
    }
    else
        cap.open(cameraId);
    if (!cap.isOpened())
    {
        cout << "video not open." << endl;
        return -1;
    }

    double rate = cap.get(CV_CAP_PROP_FPS);
    int delay = 1000/rate;
    double framewidth = 1920;
    double frameheight = 1080;
    cap.set(CV_CAP_PROP_FRAME_WIDTH, framewidth);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, frameheight);
    framewidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    frameheight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

    cout << "FPS: " << rate << endl;
    cout << "FRAME_WIDTH : " << framewidth << endl;
    cout << "FRAME_HEIGHT: " << frameheight << endl;

    unsigned long nframe = 0;

    for(;;)
    {
        Mat frame;
        cap >> frame;
        resize(frame, frame, Size(frame.cols*scale, frame.rows*scale));

        nframe ++;

        char winName[200];
        sprintf(winName, "camera%02d %dx%d@%d", cameraId, frame.cols, frame.rows, (int)rate);
        imshow(winName, frame);
        char key = (char)waitKey(delay);

        if (key == 27)
               break;
        if( key == 'c' ){
            char filename[200];
            sprintf(filename, "camera%02d.bmp", cameraId);
            imwrite(filename, frame);
            cout << "shotcut to " << filename << endl;
        }
    }

    cap.release();

    return 0;
}
