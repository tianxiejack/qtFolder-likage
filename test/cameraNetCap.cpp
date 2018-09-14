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
#include <unistd.h>
#include <pthread.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>


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


#define 	MAXSENDSIZE (136)
typedef struct {
       int byteSizeSend;
       unsigned char sendBuff[MAXSENDSIZE];
}sendInfo;

int m_port = -1;
int sockfd = -1;
vector<unsigned char>  rcvBufQue;
sendInfo repSendBuffer;


void on_mouse(int event, int x, int y, int flags, void *ustc)
{
    //static uSelect usel;
    //char temp[16];

    uSelect* usel = (uSelect*)ustc;

    if(usel == NULL)
        return ;

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
        //usel.pt = Point(x, y);
        //usel.rt.width = x - usel.rt.x;
        //usel.rt.height = y - usel.rt.y;
        //usel.notify = true;
    }
    else if (event == CV_EVENT_LBUTTONUP)
    {
        usel->stat = true;
    }

    //if(ustc != NULL)
    //   memcpy(ustc, &usel, sizeof(uSelect));

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
#if 1
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
#else
bool hc_move(LONG lUserId, HCMVMD md, DWORD speed)
{
    static HCMVMD curMd = HCMVMD_STOP;
    bool ret = false;

    if(curMd != HCMVMD_STOP){
        switch (curMd) {
        case HCMVMD_UP:
            ret = NET_DVR_PTZControl_Other(lUserId, 1, TILT_UP, 1);
            break;
        case HCMVMD_DOWN:
            ret = NET_DVR_PTZControl_Other(lUserId, 1, TILT_DOWN, 1);
            break;
        case HCMVMD_LEFT:
            ret = NET_DVR_PTZControl_Other(lUserId, 1, PAN_LEFT, 1);
            break;
        case HCMVMD_RIGHT:
            ret = NET_DVR_PTZControl_Other(lUserId, 1, PAN_RIGHT, 1);
            break;
        default:
            break;
        }
    }

    curMd = md;

    if(curMd != HCMVMD_STOP){
        switch (curMd) {
        case HCMVMD_UP:
            ret = NET_DVR_PTZControl_Other(lUserId, 1, TILT_UP, 0);
            break;
        case HCMVMD_DOWN:
            ret = NET_DVR_PTZControl_Other(lUserId, 1, TILT_DOWN, 0);
            break;
        case HCMVMD_LEFT:
            ret = NET_DVR_PTZControl_Other(lUserId, 1, PAN_LEFT, 0);
            break;
        case HCMVMD_RIGHT:
            ret = NET_DVR_PTZControl_Other(lUserId, 1, PAN_RIGHT, 0);
            break;
        default:
            break;
        }
    }

    return ret;
}
#endif

#define RECV_BUF_SIZE   1024
#define RECVSIZE        32

u_int8_t sendCheck_sum(uint len, u_int8_t *tmpbuf)
{
    u_int8_t  ckeSum=0;
    for(int n=0;n<len;n++)
        ckeSum ^= tmpbuf[n];
    return ckeSum;
}

int  NETRecv(void *rcv_buf,int data_len)
{
        int fs_sel,len;
        fd_set fd_netRead;
        struct timeval timeout;
        FD_ZERO(&fd_netRead);
        FD_SET(sockfd,&fd_netRead);
        timeout.tv_sec = 3;
        timeout.tv_usec = 0;
        fs_sel = select(sockfd+1,&fd_netRead,NULL,NULL,&timeout);
        if(fs_sel){
            len = read(sockfd,rcv_buf,data_len);
            return len;
        }
        else if(-1 == fs_sel){
            printf("ERR: Socket  select  Error!!\r\n");
            return -1;
        }
        else if(0 == fs_sel){
            //printf("Warning: Uart Recv  time  out!!\r\n");
            return 0;
        }
}


u_int8_t  check_sum(uint len_t)
{
    u_int8_t cksum = 0;
    for(int n=1;n<len_t-1;n++)
    {
        cksum ^= rcvBufQue.at(n);
    }
    return  cksum;
}


int  prcRcvFrameBufQue(int method)
{
    int ret =  -1;
    int cmdLength= (rcvBufQue.at(2)|rcvBufQue.at(3)<<8)+5;

    if(cmdLength<6){
        printf("Worning::  Invaild frame\r\n");
        rcvBufQue.erase(rcvBufQue.begin(),rcvBufQue.begin()+cmdLength);  //  analysis complete erase bytes
        return ret;
    }
    u_int8_t checkSum = check_sum(cmdLength);

    if(checkSum==rcvBufQue.at(cmdLength-1))
    {
        switch(rcvBufQue.at(4)){
            case 0x50:

                break;
            default:
                printf("INFO: Unknow  Control Command, please check!!!\r\n ");
                ret =0;
                break;
             }
      }
      else
        printf("%s,%d, checksum error:,cal is %02x,recv is %02x\n",__FILE__,__LINE__,checkSum,rcvBufQue.at(cmdLength-1));
        rcvBufQue.erase(rcvBufQue.begin(),rcvBufQue.begin()+cmdLength);  //  analysis complete erase bytes
        return 1;
}


void* RecvDataThread(void *arg)
{
    int sizeRcv;
    uint8_t dest[1024]={0};
    int read_status = 0;
    int dest_cnt = 0;
    unsigned char  tmpRcvBuff[RECV_BUF_SIZE];
    memset(tmpRcvBuff,0,sizeof(tmpRcvBuff));

    unsigned int uartdata_pos = 0;
    unsigned char frame_head[]={0xEB, 0x53};
    static struct data_buf
    {
        unsigned int len;
        unsigned int pos;
        unsigned char reading;
        unsigned char buf[RECV_BUF_SIZE];
    }swap_data = {0, 0, 0,{0}};
    memset(&swap_data,0,sizeof(swap_data));

    while(1){
        sizeRcv = NETRecv(tmpRcvBuff,RECVSIZE);
        uartdata_pos = 0;
        if(sizeRcv>0)
        {
            printf("%s,%d,recv from socket %d bytes:\n",__FILE__,__LINE__,sizeRcv);
            for(int j=0;j<sizeRcv;j++){
                printf("%02x ",tmpRcvBuff[j]);
            }
            printf("\n");
            while (uartdata_pos < sizeRcv){
                if((0 == swap_data.reading) || (2 == swap_data.reading)){
                    if(frame_head[swap_data.len] == tmpRcvBuff[uartdata_pos]){
                        swap_data.buf[swap_data.pos++] =  tmpRcvBuff[uartdata_pos++];
                        swap_data.len++;
                        swap_data.reading = 2;
                        if(swap_data.len == sizeof(frame_head)/sizeof(char))
                        swap_data.reading = 1;
                    }else{
                        uartdata_pos++;
                        if(2 == swap_data.reading)
                            memset(&swap_data, 0, sizeof(struct data_buf));
                    }
                }else if(1 == swap_data.reading){
                    swap_data.buf[swap_data.pos++] = tmpRcvBuff[uartdata_pos++];
                    swap_data.len++;
                    if(swap_data.len>=4){
                        if(swap_data.len==((swap_data.buf[2]|(swap_data.buf[3]<<8))+5)){
                            for(int i=0;i<swap_data.len;i++)
                                rcvBufQue.push_back(swap_data.buf[i]);

                            prcRcvFrameBufQue(2);
                            memset(&swap_data, 0, sizeof(struct data_buf));
                        }
                    }
                }
            }
        }
    }
    return 0;
}


int  SendDataThread(int inx,int iny,int cofx,int cofy)
{
        int retVle,n;
        {
            u_int8_t sumCheck;
            repSendBuffer.sendBuff[0]=0xEB;
            repSendBuffer.sendBuff[1]=0x53;
            repSendBuffer.sendBuff[2]=0x09;
            repSendBuffer.sendBuff[3]=0x00;
            repSendBuffer.sendBuff[4]=0x50;
            repSendBuffer.sendBuff[5]=inx&0xff;
            repSendBuffer.sendBuff[6]=(inx>>8)&0xff;
            repSendBuffer.sendBuff[7]=iny&0xff;
            repSendBuffer.sendBuff[8]=(iny>>8)&0xff ;
            repSendBuffer.sendBuff[9]=cofx&0xff;
            repSendBuffer.sendBuff[10]=(cofx>>8)&0xff ;
            repSendBuffer.sendBuff[11]=cofy&0xff;
            repSendBuffer.sendBuff[12]=(cofy>>8)&0xff ;
            sumCheck=sendCheck_sum(12,repSendBuffer.sendBuff+1);
            repSendBuffer.sendBuff[13]=(sumCheck&0xff);
            repSendBuffer.byteSizeSend=0x0e;

            retVle = write(sockfd, &repSendBuffer.sendBuff,repSendBuffer.byteSizeSend);
            //printf("%s,%d, write to socket %d bytes:\n", __FILE__,__LINE__,retVle);
            #if 0
                for(int i = 0; i < retVle; i++)
                {
                    printf("%02x ", repSendBuffer.sendBuff[i]);
                }
                printf("-----end-----\n");
            #endif
        }
        return 0;
}




void *func(void *arg)
{
    int ret = -1;
    //pthread_t sendpth = -1;
    pthread_t recvpth = -1;

    struct sockaddr_in seraddr = {0};
    struct sockaddr_in cliaddr = {0};

    // 第1步：socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (-1 == sockfd)
    {
        perror("socket");
        return NULL;
    }
    printf("socketfd = %d.\n", sockfd);

    // 第2步：connect链接服务器
    seraddr.sin_family = AF_INET;		// 设置地址族为IPv4
    seraddr.sin_port = htons(10000);	// 设置地址的端口号信息
    seraddr.sin_addr.s_addr = inet_addr("192.168.0.210");	//　设置IP地址
    ret = connect(sockfd, (const struct sockaddr *)&seraddr, sizeof(seraddr));
    if (ret < 0)
    {
        perror("listen");
        return NULL;
    }
    printf("成功建立连接\n");


    //ret = pthread_create(&sendpth, NULL, SendDataThread, NULL);
    //ret = pthread_create(&recvpth, NULL, RecvDataThread, NULL);


    return NULL;
}


int main(int argc, char** argv)
{

    func(NULL);

    LONG hcDev = -1;
    DWORD mvSpeed = 2;
    HCMVMD mvMd = HCMVMD_STOP, mvMdBak = HCMVMD_STOP;
    NET_DVR_PTZPOS pos;

    cv::CommandLineParser parser(argc, argv,
                                 "{s|0.5|}{help||}{@file|rtsp://admin:admin$2018@192.168.0.64:554|}");
    if (parser.has("help"))
        return print_help();
    float scale = parser.get<float>("s");
    string strURL = parser.get<string>("@file");
    //string strURL = "rtsp://admin:admin$2018@192.168.0.64:554";

    hcDev = hc_init();

    VideoCapture cap;
    cap.open(strURL);
    if (!cap.isOpened())
    {
        return -1;
    }

    unsigned long nframe = 0;
    uSelect sel;
    string strWinName;

    namedWindow("ball", 1 );

    for(;;)
    {
        Mat frame;
        cap >> frame;
        resize(frame, frame, Size(frame.cols*scale, frame.rows*scale));

        if(nframe == 0)
            setMouseCallback("ball", on_mouse, &sel);

        nframe ++;

        if(sel.valid){
            Point pos = sel.pt;

            if(sel.notify){
                sel.notify = false;
                cout<<"x,y=("<<pos.x<<","<<pos.y<<")"<<endl;

                //NET_DVR_PTZPOS getPos;
                //DWORD lentmp;
                //NET_DVR_GetDVRConfig(hcDev, NET_DVR_GET_PTZPOS,1, &getPos, sizeof(getPos), &lentmp);
                //cout << "wPanPos " << getPos.wPanPos << " wTiltPos " << getPos.wTiltPos << " wZoomPos " << getPos.wZoomPos << endl;


                //pos.x = 480 + 50;
                //pos.y = 270 + 50;
                //SendDataThread(pos.x,pos.y,388,392);
                SendDataThread(pos.x,pos.y,380,385);

                //static int add = 0;
                //int temp1 = add*10;
                //int temp2 = 0;
                //add += 10;
                //SendDataThread(temp1,temp2);

                /*
                    NET_DVR_PTZPOS getPos;
                    NET_DVR_PTZPOS pos2;
                    DWORD lentmp;
                    NET_DVR_GetDVRConfig(hcDev, NET_DVR_GET_PTZPOS,1, &getPos, sizeof(getPos), &lentmp);
                    static int add = 0;
                    pos2.wAction = 1;
                    int temp1 = add;
                    pos2.wPanPos = (temp1<0) ? (13720+temp1) : temp1;
                    int temp2 = 0;
                    add += 10;
                    pos2.wTiltPos = (temp2<0) ? (13720+temp2) : temp2;
                    pos2.wZoomPos = 0;//zoomPosBase;
                    NET_DVR_SetDVRConfig(hcDev, NET_DVR_SET_PTZPOS,1, &pos2, sizeof(pos2));
                    cout << "t1 " << temp1 << " t2 " << temp2 << endl;
                    cout << "wPanPos " << pos2.wPanPos << " wTiltPos " << pos2.wTiltPos << " wZoomPos " << pos2.wZoomPos << endl;
                */
            }
        }

        drawMarker(frame,
                   Point(frame.cols/2, frame.rows/2),
                   Scalar(255, 255, 255, 0), MARKER_CROSS, 100);

        imshow("ball", frame);
        char key = (char)waitKey(1);

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
        else if(key == -1 || key == 'r'){}
        else{
            printf("%c[%d] \n", key, key);
            hc_move(hcDev,HCMVMD_STOP,0);
            mvMd = HCMVMD_STOP;
        }
        if (key == 27)
               break;
        if((mvMd == HCMVMD_STOP && mvMdBak != HCMVMD_STOP) || key == 'r'){
            NET_DVR_PTZPOS pos;
            DWORD len;
            NET_DVR_GetDVRConfig(hcDev, NET_DVR_GET_PTZPOS,1, &pos, sizeof(pos), &len);
            cout << "ptz pos : action[ " << pos.wAction
                 << " ] pan[ " << pos.wPanPos
                 << " ] tilt [ " << pos.wTiltPos
                 << " ] zoom [ " << pos.wZoomPos << " ]" << endl;
        }
    }

    hc_uninit(hcDev);

    return 0;
}
