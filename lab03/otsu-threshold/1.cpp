#include <iostream>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>

using namespace std;
using namespace cv;

void Ostu_algo(Mat& inputImg, Mat& outputImg);

int main(int argc, char **argv)
{
    Mat inputImg = imread(argv[1]);
    cvtColor(inputImg, inputImg, cv::COLOR_BGR2GRAY);
    Mat outputImg = inputImg.clone();
    
    Ostu_algo(inputImg, outputImg);
    
    
    imshow("output", outputImg);
    imwrite("output.tif", outputImg);
    
    waitKey(0);
    
    return 0;
}

void Ostu_algo(Mat& inputImg, Mat& outputImg) {
    
    int histogram[256] = {0};
    
    Size mat_size = inputImg.size();
    int rows = mat_size.height;
    int cols = mat_size.width;
    
    //histogram calculation
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            histogram[inputImg.at<uchar>(i, j)] += 1;
        }
    }
    
    //Ostu algo.
    float num_of_background = histogram[0];
    float mean_of_background = (1 * histogram[0]) / num_of_background;
    float num_of_object = 0;
    for(int i = 1; i < 256; i++) {
        num_of_object += histogram[i];
    }
    float mean_of_object = 0.0;
    for(int i = 1; i < 256; i++) {
        mean_of_object += (i + 1) * histogram[i];
    }
    mean_of_object /= num_of_object;
    float max_val_of_threshold = num_of_object * num_of_background * pow((mean_of_background - mean_of_object), 2);
    int threshold = 0;
    
    for(int i = 1; i < 255; i++) {
        
        mean_of_background  = ((mean_of_background * num_of_background) + ((i + 1) * histogram[i])) / (num_of_background + histogram[i]);
        mean_of_object = ((mean_of_object * num_of_object) - ((i + 1) * histogram[i])) / (num_of_object - histogram[i]);
        num_of_background += histogram[i];
        num_of_object -= histogram[i];
        
        float val_of_threshold = num_of_object * num_of_background * pow((mean_of_background - mean_of_object), 2);
        
        if(max_val_of_threshold < val_of_threshold) {
            max_val_of_threshold = val_of_threshold;
            threshold = i;
        }
        
        //cout << val_of_threshold << endl;
    }
    
    
    //printf("%d", threshold);
    
    //histogram cutout
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            if(inputImg.at<uchar>(i, j) < threshold) {
                outputImg.at<uchar>(i, j) = 0;
            } else {
                outputImg.at<uchar>(i, j) = 255;
            }
        }
    }
    
    return ;
}

