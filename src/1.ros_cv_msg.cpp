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

// Create a variable to save the position value in track
int blurAmount=15;
// Trackbar call back function
static void onChange(int pos, void* userInput)
{
    if(pos <= 0)
        return;
    // Aux variable for result
    Mat imgBlur;

    // Get the pointer input image
    Mat* img= (Mat*)userInput;

    // Apply a blur filter
    blur(*img, imgBlur, Size(pos, pos));

    // Show the result
    imshow("Lena", imgBlur);
}
//Mouse callback
static void onMouse( int event, int x, int y, int, void* userInput )
{
    if( event != EVENT_LBUTTONDOWN )
        return;

    // Get the pointer input image
    Mat* img= (Mat*)userInput;

    // Draw circle
    circle(*img, Point(x, y), 10, Scalar(0,255,0), 3);

    // Call on change to get blurred image
    onChange(blurAmount, img);

}

int main( int argc, char** argv )
{
    ros::init(argc,argv,"redwall_arm_vision");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub1 = it.advertise("camera/image_color", 1);
    image_transport::Publisher pub2 = it.advertise("camera/image_gray", 1);
    /**************ROS与Opencv图像转换***********************/
    Mat image= imread("/home/redwall/catkin_ws/src/redwall_arm/redwall_arm_vision/image/stuff.jpg", CV_LOAD_IMAGE_COLOR);
    sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    Mat gray= imread("/home/redwall/catkin_ws/src/redwall_arm/redwall_arm_vision/image/stuff.jpg", CV_LOAD_IMAGE_GRAYSCALE);
    sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray).toImageMsg();
//    imshow("Lena BGR", image);
//    imshow("Lena Gray", gray);
    // wait for any key press
//    waitKey(30);
    ros::Rate loop_rate(5);
    while (nh.ok()) {
        pub1.publish(msg1);
        pub2.publish(msg2);
        ros::spinOnce();
        loop_rate.sleep();
    }


    /**************调用电脑摄像头***********************/
//    VideoCapture cap; // open the default camera
//    cap.open(0);
//    if(!cap.isOpened())  // check if we succeeded
//        return -1;
//    namedWindow("Video",1);
//    for(;;)
//    {
//        Mat frame;
//        cap >> frame; // get a new frame from camera
//        imshow("Video", frame);
//        if(waitKey(30) >= 0) break;
//    }
//    // Release the camera or video cap
//    cap.release();
    /**********************************************/

    /**************使用window***********************/
//    namedWindow("stuff", WINDOW_NORMAL);
//    // Move window
//    moveWindow("stuff", 10, 10);
//    // create a trackbark
//    createTrackbar("stuff", "stuff", &blurAmount, 30, onChange, &image);
//    setMouseCallback("stuff", onMouse, &image);
//    // Call to onChange to init
//    onChange(blurAmount, &image);
//    // show images
//    cv::imshow("stuff", image);
//    // Display Overlay
//    displayOverlay("stuff", "Overlay 5secs", 5000);
//    // Display Status Bar
//    displayStatusBar("stuff", "Status bar 5secs", 5000);
//    // Save window parameters
//    saveWindowParameters("stuff");
//    // load Window parameters
//    loadWindowParameters("stuff");
//    // Resize window, only non autosize
//    resizeWindow("stuff", 512, 512);
//    // wait
//    waitKey(0);
//    // Destroy the windows
//    destroyWindow("stuff");
    /**********************************************/

    return 0;

}