#include <cstdio>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>

using namespace std;
using namespace cv;

// write down your warping function here

void onMouse(int event, int x, int y, int flags, void* param) {
    vector<Point2f>* ptr = (vector<Point2f>*) param;
    if (event == CV_EVENT_LBUTTONDOWN) {
        ptr->push_back(Point2f(x, y));
    }
}

void _warpPerspective(Mat& Input, Mat& output, Mat& _3by3matrix)
{
    //    Mat output = Output.clone();
    //    for (int i = 0; i < output.rows; i++)
    //    {
    //        for (int j = 0; j < output.cols; j++)
    //        {
    //            output.at<Vec3b>(i, j)[0] = 0;
    //            output.at<Vec3b>(i, j)[1] = 0;
    //            output.at<Vec3b>(i, j)[2] = 0;
    //        }
    //    }
    int rows = Input.rows;
    int cols = Input.cols;
    float x = 0;
    float y = 0;
    
    for (int i = 0; i < cols; i++)
    {
        for (int j = 0; j < rows; j++)
        {
            Mat b = Mat(3, 1, CV_32FC1);
            b.at<float>(0, 0) = i;
            b.at<float>(1, 0) = j;
            b.at<float>(2, 0) = 1;
            b = _3by3matrix * b;
            x = b.at<float>(0, 0) / b.at<float>(2, 0);
            y = b.at<float>(1, 0) / b.at<float>(2, 0);
            output.at<Vec3b>(y, x) = Input.at<Vec3b>(j, i);
        }
    }
    
    //return output;
}

int main() {
    VideoCapture cap(1);
    if (!cap.isOpened()) {
        return -1;
    }
    Mat image;
    image = imread("/Users/Henry/Downloads/osaka.jpg");
    
    Mat frame;
    //frame = imread("hi.jpg");
    cap >> frame;
    
    vector<Point2f> cap_corner;
    vector<Point2f> img_corner;
    
    // add the corner of frame into cap_corner
    cap_corner.push_back(Point2f(0, 0));
    cap_corner.push_back(Point2f(0, frame.rows - 1));
    cap_corner.push_back(Point2f(frame.cols - 1, frame.rows - 1));
    cap_corner.push_back(Point2f(frame.cols - 1, 0));
    
    namedWindow("img", CV_WINDOW_AUTOSIZE);
    setMouseCallback("img", onMouse, &img_corner);
    
    while (img_corner.size()<4) {
        imshow("img", image);
        if (waitKey(1) == 27) break;
    }
    
    
    Mat img_out = image.clone();
    Mat img_temp = image.clone();
    
    for (int i = 0; i < img_temp.rows; i++)
    {
        for (int j = 0; j < img_temp.cols; j++)
        {
            img_temp.at<Vec3b>(i, j)[0] = 0;
            img_temp.at<Vec3b>(i, j)[1] = 0;
            img_temp.at<Vec3b>(i, j)[2] = 0;
        }
    }
    
    
    Mat result(3, 3, CV_32FC1);
    result = findHomography(cap_corner, img_corner);
    // call your warping function
    
    //result = getPerspectiveTransform(cap_corner, img_corner);
    result.convertTo(result, 5);
    _warpPerspective(frame, img_temp, result);
    //cout << result << endl;
    //float c[3][1] = { {2},{3},{1} };
    //Mat b(3, 1, CV_32FC1,c);
    //cout << b << endl;
    //Mat ans(3, 1, CV_32FC1);
    
    //ans = result * b;
    //multiply(result, b,ans);
    //cout << ans<<endl;
    //warpPerspective(frame, img_temp, result, img_temp.size());
    //imshow("warped", img_temp);
    
    Point poly[4];
    for (int i = 0; i < img_corner.size(); i++) {
        poly[i] = img_corner[i];
    }
    //waitKey(0);
    while (1) {
        cap >> frame;
        // call your warping function
        _warpPerspective(frame, img_temp, result);
        //warpPerspective(frame, img_temp, result, img_temp.size());
        fillConvexPoly(img_out, poly, 4, Scalar::all(0), CV_AA);
        img_out = img_out + img_temp;
        imshow("img", img_out);
        if (waitKey(1) == 27) break;
    }
    return 0;
}

