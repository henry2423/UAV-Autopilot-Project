#include <iostream>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
	Mat inputImg = imread(argv[1]);
	cvtColor(inputImg, inputImg, COLOR_BGR2GRAY);
	Size mat_size = inputImg.size();
	int rows = mat_size.height;
	int cols = mat_size.width;

	Mat outputImg = inputImg.clone();

	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			int intensity = (int)inputImg.at<uchar>(i, j) * (-4);
			
			if (i - 1 >= 0)
				intensity += inputImg.at<uchar>(i - 1, j);
			if(i + 1 < rows)
				intensity += inputImg.at<uchar>(i + 1, j);
			if(j - 1 >= 0)
				intensity += inputImg.at<uchar>(i, j - 1);
			if(j + 1 < cols)
				intensity += inputImg.at<uchar>(i, j + 1);

			if (intensity < 0)
				intensity = 0;
			outputImg.at<uchar>(i, j) = intensity;
		}
	}

	imshow("hihi", outputImg);
	imwrite("output2.jpg", outputImg);
	waitKey(0);
	return 0;
}