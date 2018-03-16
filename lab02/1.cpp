#include <iostream>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    Mat inputImg = imread(argv[1]);
    int histogram[256] = {0};
    
    cvtColor(inputImg, inputImg, cv::COLOR_BGR2GRAY);
    Size mat_size = inputImg.size();
    int rows = mat_size.height;
    int cols = mat_size.width;
    Mat outputImg = inputImg.clone();
    
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            histogram[inputImg.at<uchar>(i, j)] += 1;
        }
    }
    
    int last = 0;
    
    for (int i = 0; i < 256; i++)
    {
        last += histogram[i];
        float prob = (last * 255.0) / (rows * cols);
        histogram[i] = prob;
    }
    
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            
            outputImg.at<uchar>(i, j) = uchar(histogram[inputImg.at<uchar>(i, j)]);
            
        }
    }
    
    imshow("mj", outputImg);
    imwrite("output.tif", outputImg);
    
    waitKey(0);
    
    return 0;
}

