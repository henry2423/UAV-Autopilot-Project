#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

void    bgr2rgb(Mat& input, Mat& output);

int main(int argc, char** argv){
    
    Mat input_img = imread("/Users/Henry/Downloads/lab01/BGR2RGB/kobe.jpg");
    Mat output_img = input_img.clone();
    
    bgr2rgb(input_img, output_img);
    
    imshow("/Users/Henry/Downloads/lab01/BGR2RGB/kobe.jpg", input_img);
    imshow("/Users/Henry/Downloads/lab01/BGR2RGB/bgr.jpg", output_img);
    waitKey(0);
    
    imwrite("output.jpg", output_img);
    
    return 0;
}
void    bgr2rgb(Mat& input, Mat& output) {
    
    Size mat_size = input.size();
    int rows = mat_size.height;
    int cols = mat_size.width;
    
    for(int i=0; i<rows; i++) {
        for(int j=0; j<cols; j++) {
            uchar intensity_red = input.at<Vec3b>(i, j)[2];
            uchar intensity_blue = input.at<Vec3b>(i, j)[0];
            
            output.at<Vec3b>(i, j)[2] = intensity_blue;
            output.at<Vec3b>(i, j)[0] = intensity_red;
        }
    }
    
    
}


