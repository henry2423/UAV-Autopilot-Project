#include <iostream>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>

using namespace std;
using namespace cv;

void     bilinear_interpolation(Mat& input, Mat& output, float scalingFactor);

int     main(int argc, char** argv) {
    
    Mat inputImg = imread("/Users/Henry/Downloads/lab01/interpolation/IU_small.jpg");
    float scalingFactor = atof("3.7");
    
    int scaledWidth = round(1. * inputImg.cols * scalingFactor);
    int scaledHeight = round(1. * inputImg.rows * scalingFactor);
    
    Mat outputImg1 = Mat(scaledHeight, scaledWidth, inputImg.type());
    Mat outputImg2; // for opencv build-in function
    
    // resize the input image by your bilinear_interpolation funcion
    bilinear_interpolation(inputImg, outputImg1, scalingFactor);
    // resize the input image by opencv
    resize(inputImg, outputImg2, Size(inputImg.cols * scalingFactor, inputImg.rows * scalingFactor), INTER_LINEAR);
    
    imshow("My Interpolation", outputImg1);
    imshow("Opencv build-in function", outputImg2);
    waitKey(0);
    
    imwrite("output.jpg", outputImg1);
    
    return 0;
}


void     bilinear_interpolation(Mat& input, Mat& output, float scalingFactor) {
    
    Size mat_size = output.size();
    int rows = mat_size.height;
    int cols = mat_size.width;
    int nearest_0, nearest_1, nearest_2, nearest_3;
    uchar intensity_0, intensity_1, intensity_2, intensity_3;
    float weight1, weight2, weight3, weight4;
    
    
    for(int i=0; i<rows; i++) {
        for(int j=0; j<cols; j++) {
            
            nearest_0 = i / scalingFactor;
            if(nearest_0 == input.size().height - 1) {
                nearest_1 = nearest_0;
            } else {
                nearest_1 = nearest_0 + 1;
            }
            
            nearest_2 = j / scalingFactor;
            if(nearest_2 == input.size().width - 1) {
                nearest_3 = nearest_2;
            } else {
                nearest_3 = nearest_2 + 1;
            }
            
            weight1 = i / scalingFactor - nearest_0;
            weight2 = 1 - weight1;
            //nearest_1 - i / scalingFactor;
            weight3 = j / scalingFactor - nearest_2;
            weight4 = 1 - weight3;
            //nearest_3 - j / scalingFactor;
            
            //red, green, blue change
            for(int k=0; k<3; k++) {
                intensity_0 = input.at<Vec3b>(nearest_0, nearest_2)[k];
                intensity_1 = input.at<Vec3b>(nearest_1, nearest_2)[k];
                intensity_2 = input.at<Vec3b>(nearest_0, nearest_3)[k];
                intensity_3 = input.at<Vec3b>(nearest_1, nearest_3)[k];
                
                output.at<Vec3b>(i,j)[k] = weight1 * weight3 * intensity_3 + weight1 * weight4 * intensity_1 + weight2 * weight3 * intensity_2 + weight2 * weight4 * intensity_0;
            }


        }
    }
    
    
}

