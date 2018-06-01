#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/objdetect.hpp>
#include <vector>
#include <string>

using namespace std;
using namespace cv;

int main(int argc, char** argv) {

	vector<Mat> images;
	string list[3] = { "test_1.jpg", "test_2.jpg", "test_3.jpg" };
	for (int i = 0; i < 3; i++) {
		Mat tmp;
		tmp = imread(list[i]);
		images.push_back(tmp);
	}
	Mat img = images[0];
	CascadeClassifier CC[4];
	bool ok = CC[0].load("haarcascade_frontalface_default.xml");
	if (!ok)
		cout << "fuck" << endl;
	//system("Pause");
	//CC[1].load("haarcascade_frontalface_alt.xml");
	CC[2].load("haarcascade_frontalface_alt2.xml");
	//CC[3].load("haarcascade_frontalface_alt_tree.xml");

	vector<Rect> faces[3];
	//goooooood for 1:CC[2].detectMultiScale(img, faces[0], 1.05, 5, 0 | CASCADE_SCALE_IMAGE, Size(40, 40));
	CC[2].detectMultiScale(img, faces[0], 1.05, 6, 0 | CASCADE_SCALE_IMAGE, Size(35, 35));
	//CC[1].detectMultiScale(images[0], faces[1], 1.1, 5, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
	//CC[2].detectMultiScale(images[0], faces[2], 1.1, 5, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
	for (size_t j = 0; j < 3; j++)
	{
		for (size_t i = 0; i < faces[j].size(); i++)
		{
			int scale = 1;
			Rect r = faces[j][i];
			Scalar color = Scalar(0, 255, 255);
			double aspect_ratio = (double)r.width / r.height;
			rectangle(img, cvPoint(cvRound(r.x*scale), cvRound(r.y*scale)),
				cvPoint(cvRound((r.x + r.width - 1)*scale), cvRound((r.y + r.height - 1)*scale)),
				color, 3, 8, 0);
		}
	}
	cout << "number of faces = " << faces[0].size() << endl;
	namedWindow("img", 0);
	resize(img, img, Size(img.size().height / 2, img.size().width / 2), 1, 1, 0);
	imshow("img", img);
	cout << "hihi" << endl;
	waitKey(0);


	return 0;
}
