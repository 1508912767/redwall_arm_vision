#include <iostream>
#include <string>
#include <sstream>
#include <cmath>
using namespace std;

// OpenCV includes
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "MultipleImageWindow.h"
using namespace cv;

//////////// 界面 ////////////
MultipleImageWindow::MultipleImageWindow(string window_title, int cols, int rows, int flags)
{
    this->window_title= window_title;
    this->cols= cols;
    this->rows= rows;
    namedWindow(window_title, flags);
    // ToDo: detect resolution of desktop and show fullresolution canvas
    this->canvas_width= 1200;
    this->canvas_height= 700;
    this->canvas= Mat(this->canvas_height, this->canvas_width, CV_8UC3);
    imshow(this->window_title, this->canvas);
}

int MultipleImageWindow::addImage(string title, Mat image, bool render)
{
    this->titles.push_back(title);
    this->images.push_back(image);
    if(render)
        this->render();
    return this->images.size()-1;
}

void MultipleImageWindow::removeImage(int pos)
{
    this->titles.erase(this->titles.begin()+pos);
    this->images.erase(this->images.begin()+pos);
}

void MultipleImageWindow::render()
{
    // Clean our canvas
    this->canvas.setTo( Scalar(20,20,20) );
    // width and height of cell. add 10 px of padding between images
    int cell_width= (canvas_width/cols);
    int cell_height= (canvas_height/rows);
    int margin= 10;
    int max_images=(this->images.size()>cols*rows)?cols*rows:this->images.size();
    int i=0;
    vector<string>::iterator titles_it= this->titles.begin();
    for(vector<Mat>::iterator it= this->images.begin(); it!= this->images.end(); ++it)
    {
        string title= *titles_it;
        int cell_x= (cell_width)*((i)%cols);
        int cell_y= (cell_height)*floor((i)/(float)cols);
        Rect mask(cell_x, cell_y, cell_width, cell_height);
        // Draw a rectangle for each cell mat
        rectangle(canvas, Rect(cell_x, cell_y, cell_width, cell_height), Scalar(200,200,200), 1);
        //For each cell draw an image if exists
        Mat cell(this->canvas, mask);
        // resize image to cell size
        Mat resized;
        double cell_aspect= (double)cell_width/(double)cell_height;
        Mat img= *it;
        double img_aspect= (double)img.cols/(double)img.rows;
        double f=(cell_aspect<img_aspect)?(double)cell_width/(double)img.cols:(double)cell_height/(double)img.rows;
        resize(img, resized, Size(0,0), f, f);
        if(resized.channels()==1){
            cvtColor(resized, resized, COLOR_GRAY2BGR);
        }

        // Assign the image
        Mat sub_cell(this->canvas, Rect(cell_x,cell_y,resized.cols, resized.rows));
        resized.copyTo(sub_cell);
        putText(cell, title.c_str(), Point(20,20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200,0,0), 1, LINE_AA);
        i++;
        ++titles_it;
        if(i==max_images)
            break;
    }

    // show image
    imshow(this->window_title, this->canvas);
}

MultipleImageWindow* miw;
//////////// 界面 ////////////

static Scalar randomColor( RNG& rng )
{
    int icolor = (unsigned) rng;
    return Scalar( icolor&255, (icolor>>8)&255, (icolor>>16)&255 );
}


void ConnectedComponents(Mat img)
{
  // Use connected components to divide our possibles parts of images 
  Mat labels;
  int num_objects = connectedComponents(img, labels);
  // Check the number of objects detected
  if(num_objects < 2 ){
    cout << "No objects detected" << endl;
    return;
  }else{
    cout << "Number of objects detected: " << num_objects - 1 << endl;
  }
  // Create output image coloring the objects
  Mat output= Mat::zeros(img.rows,img.cols, CV_8UC3);
  RNG rng( 0xFFFFFFFF );
  for(int i=1; i<num_objects; i++){
    Mat mask= labels==i;
    output.setTo(randomColor(rng), mask);
  }
  //imshow("Result", output);
  miw->addImage("Result", output);
}

void ConnectedComponentsStats(Mat img)
{
  // Use connected components with stats
  Mat labels, stats, centroids;
  int num_objects= connectedComponentsWithStats(img, labels, stats, centroids);
  // Check the number of objects detected
  if(num_objects < 2 ){
    cout << "No objects detected" << endl;
    return;
  }else{
    //cout << "Number of objects detected: " << num_objects - 1 << endl;
  }
  // Create output image coloring the objects and show area
  Mat output= Mat::zeros(img.rows,img.cols, CV_8UC3);
  RNG rng( 0xFFFFFFFF );

  int tmp_num = num_objects;
  for(int i=1; i<num_objects; i++){
      // 我加了个判断，面积小于500的不计入考虑
      if(stats.at<int>(i, CC_STAT_AREA)>=500)
      {
          cout << "Object "<< i << " with pos: " << centroids.at<Point2d>(i) << " with area " << stats.at<int>(i, CC_STAT_AREA) << endl;
          Mat mask= labels==i;
          output.setTo(randomColor(rng), mask);
          // draw text with area
          stringstream ss;
          ss << "area: " << stats.at<int>(i, CC_STAT_AREA);

          putText(output,
                  ss.str(),
                  centroids.at<Point2d>(i),
                  FONT_HERSHEY_SIMPLEX,
                  0.4,
                  Scalar(255,255,255));
      }
      else
      {
          tmp_num--;
      }

  }
    cout << "Number of objects detected: " << tmp_num - 1 << endl;
    imshow("Result", output);
  miw->addImage("Result", output);
}

void FindContoursBasic(Mat img)
{
  vector<vector<Point> > contours;
  findContours(img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  Mat output= Mat::zeros(img.rows,img.cols, CV_8UC3);
  // Check the number of objects detected
  if(contours.size() == 0 ){
    cout << "No objects detected" << endl;
    return;
  }else{
    cout << "Number of objects detected: " << contours.size() << endl;
  }
  RNG rng( 0xFFFFFFFF );
  for(int i=0; i<contours.size(); i++)
    drawContours(output, contours, i, randomColor(rng));
  //imshow("Result", output);
  miw->addImage("Result", output);
}

/**
 * Remove th light and return new image without light
 */
Mat removeLight(Mat img, Mat pattern, int method)
{
    Mat aux;
    // 除法
    if(method==1)
    {
        // Require change our image to 32 float for division
        Mat img32, pattern32;
        img.convertTo(img32, CV_32F);
        pattern.convertTo(pattern32, CV_32F);
        // Divide the image by the pattern
        aux= 1-(img32/pattern32);
        // Scale it to convert o 8bit format
        aux=aux*255;
        // Convert 8 bits format
        aux.convertTo(aux, CV_8U);
    }
    // 差分
    else{
        aux= pattern-img;
    }
    return aux;
}

int main( int argc, const char** argv )
{
    int method_light,method_seg;
    cout<<"输入0差分(效果最好),输入1除法,输入2则仅通过阈值提取(效果较差,在前景提取之后在阈值效果较好)"<<endl;
    cin >> method_light;
    cout<<"输入1使用components函数分割,输入2联通组件用于显示计数状态,输入3使用findContours函数分割(轮廓)"<<endl;
    cin >> method_seg;

    // 输入有噪声的图片
    Mat img = imread("/home/redwall/catkin_ws/src/redwall_arm/redwall_arm_vision/image/test_noise.pgm", CV_LOAD_IMAGE_GRAYSCALE);
    // 创建界面3列2行
    miw= new MultipleImageWindow("Main window", 3, 2, WINDOW_AUTOSIZE);
  
    // 消除噪声
    Mat img_noise, img_box_smooth;
    // 中值滤波器用于去除椒盐噪声
    medianBlur(img, img_noise, 3);
    // 平滑操作
    blur(img, img_box_smooth, Size(3,3));

    // 加载背景照片
    Mat light_pattern = imread("/home/redwall/catkin_ws/src/redwall_arm/redwall_arm_vision/image/light.pgm", CV_LOAD_IMAGE_GRAYSCALE);
    medianBlur(light_pattern, light_pattern, 3);

    // 去除背景的操作,0表示差法,1表示除法,
    Mat img_no_light;
    img_noise.copyTo(img_no_light);
    if(method_light!=2){
        img_no_light= removeLight(img_noise, light_pattern, method_light);
    }

    // 去除背景的操作,输入2使用阈值分割的方法
    Mat img_thr;
    if(method_light!=2){
        // 在没有背景提取之前,直接通过阈值风格,取值范围要变化大一点
        threshold(img_no_light, img_thr, 30, 255, THRESH_BINARY);
    }
    // 通过背景提取之后再进行阈值操作的图像
    else{
        threshold(img_no_light, img_thr, 140, 255, THRESH_BINARY_INV);
    }
    // 总之，对于此处背景比较鲜明的,阈值其实影响不打，不管30还是140.
    // 而进行背景提取之后，效果肯定是比较好的

    // Show images
    miw->addImage("Input", img);
    miw->addImage("Input with filtering", img_noise);
    miw->addImage("Light Pattern", light_pattern);
    miw->addImage("Front extract", img_no_light);
    miw->addImage("Threshold", img_thr);
  
    switch(method_seg){
        case 1:
            // 连通区域:如果两个像素相临且具有相同值,分为8连通和4连通算法
            ConnectedComponents(img_thr);
            break;
        case 2:
            // 为分割得到的物体贴上标签
            ConnectedComponentsStats(img_thr);
            break;
        case 3:
            // 最常用目标分割算法,并可以绘制轮廓
            FindContoursBasic(img_thr);
            break;
    }

    miw->render();
    waitKey(0);
    return 0;

}
