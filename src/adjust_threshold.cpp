#include <iostream>
#include <string>
#include <sstream>
using namespace std;

#include <opencv2/video.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

int iLowH = 0;
int iHighH = 179;

int iLowS = 141;
int iHighS = 255;

int iLowV = 115;
int iHighV = 210;

// 初始图像特征矩的坐标，用来更新前一帧坐标的
int iLastX = -1;
int iLastY = -1;

cv::Mat imgTmp;
cv::Mat imgLines;

int exetime = 0;

void Init(Mat picture)
{
    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
    waitKey(30);

    //Create trackbars in "Control" window
    createTrackbar("LowH", "Control", &iLowH, 255); //Hue (0 - 179)
    createTrackbar("HighH", "Control", &iHighH, 255);

    createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    createTrackbar("HighS", "Control", &iHighS, 255);

    createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
    createTrackbar("HighV", "Control", &iHighV, 255);

    Mat PI = picture;

    imgLines = Mat::zeros(PI.size(), CV_8UC3);;
}

vector<double> detect(Mat picture)
{
    vector<double> coordinate;

    if (exetime == 0)
    {
        Init(picture);
        exetime = 1;
    }

    Mat imgHSV;
    cvtColor(picture, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
    Mat imgThresholded;

    //Threshold the image
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);
    //morphological opening (removes small objects from the foreground)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    //morphological closing (removes small holes from the foreground)
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

    //Calculate the moments of the thresholded image
    Moments oMoments = moments(imgThresholded);
    // 轮廓的特征矩
    double dM01 = oMoments.m01;
    double dM10 = oMoments.m10;
    double dArea = oMoments.m00;

    // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero
    if (dArea > 10000)
    {
        //calculate the position of the ball
        double posX = dM10 / dArea;
        double posY = dM01 / dArea;

        cout << "posX  " << posX;
        cout << "   posY  " << posY << endl;

        coordinate.push_back(posX);
        coordinate.push_back(posY);

        if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
        {
            //Draw a red line from the previous point to the current point
            line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0, 0, 255), 2);
        }

        iLastX = posX;
        iLastY = posY;
    }
    //	cout << "iLastX  " << iLastX ;
    //	cout << "   iLastY  " << iLastX << endl;

    namedWindow("Thresholded Image");
    imshow("Thresholded Image", imgThresholded); //show the thresholded image
    waitKey(30);

    // 把红线和图像同时显示
    picture = picture + imgLines;
    namedWindow("Original");
    imshow("Original", picture); //show the original image
    waitKey(30);

    return coordinate;
}

int main(int argc, char** argv)
{
    VideoCapture cap(0); //capture the video from webcam

    Mat frame;

    if (!cap.isOpened())  // if not success, exit program
    {
        cout << "Cannot open the web cam" << endl;
        return -1;
    }
    while (true)
    {
        cap >> frame;
        detect(frame);
    }

    return 0;
}