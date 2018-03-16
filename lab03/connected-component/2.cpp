#include <iostream>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <ctime>

using namespace std;
using namespace cv;

struct Label {
	int value;
	struct Label *equal;
};

vector<Label*> labelList;

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

void LabelEqual(int left, int up)
{
	struct Label *left_root, *up_root;
	for (left_root = labelList[left]; left_root->equal != NULL; left_root = left_root->equal) {/*
		cout << left_root->value << endl;
		if (left_root->equal == NULL)
			cout << "NULL" << endl;
		else
			cout << left_root->equal->value << endl;*/
	}
	for (up_root = labelList[up]; up_root->equal != NULL; up_root = up_root->equal);
	if (left_root->value != up_root->value) {
		if (left_root->value > up_root->value) {
			cout << left_root->value << " > " << up_root->value << endl;
			left_root->equal = up_root;
			//cout << "test " << left_root->equal->value << endl;
		}
		else {
			cout << up_root->value << " > " << left_root->value << endl;
			up_root->equal = left_root;
			//cout << "test " << up_root->equal->value << endl;
		}
	}
}

int main(int argc, char **argv)
{
	srand(time(NULL));
	Mat inputImg = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
	Size mat_size = inputImg.size();
	int rows = mat_size.height;
	int cols = mat_size.width;

	Ostu_algo(inputImg, inputImg);

	Mat outputImg(rows, cols, CV_8UC3, Scalar(0, 0, 0));
	vector<vector<int> > labelMap(rows, vector<int>(cols, 0));
	Label *uselessIndex0 = new Label;
	uselessIndex0->value = 0;
	uselessIndex0->equal = NULL;
	labelList.push_back(uselessIndex0);

	//imshow("test", inputImg);
	//imshow("Test", outputImg);
	//system("Pause");

	int flag = 0;
	int now, left, up;
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {

			now = inputImg.at<uchar>(i, j);

			// first pixel
			if (i == 0 && j == 0) {
				if (now == 0)
					labelMap[i][j] = 0;			// don't update
				else {
					labelMap[i][j] = ++flag;	// component head
					Label *lTemp = new Label;
					lTemp->value = flag;
					lTemp->equal = NULL;
					labelList.push_back(lTemp);
				}
			}
			// first row
			else if (i == 0) {
				// only check left
				left = labelMap[i][j - 1];	

				if (now != 0) {						// update : left || flag
					if (left == 0) {
						labelMap[i][j] = ++flag;	// component head
						Label *lTemp = new Label;
						lTemp->value = flag;
						lTemp->equal = NULL;
						labelList.push_back(lTemp);
					}
					else
						labelMap[i][j] = left;		// same left
				}
				else {	
					labelMap[i][j] = 0;				// don't update
				}

			}
			// first col
			else if (j == 0) {
				// only check up
				up = labelMap[i - 1][j];

				if (now != 0) {						// update : up || flag
					if (up == 0) {
						labelMap[i][j] = ++flag;	// component head
						Label *lTemp = new Label;
						lTemp->value = flag;
						lTemp->equal = NULL;
						labelList.push_back(lTemp);
					}
					else
						labelMap[i][j] = up;		// same up
				}
				else {
					labelMap[i][j] = 0;				// don't update
				}

			}
			else {
				// check left and up
				left = labelMap[i][j - 1];
				up   = labelMap[i - 1][j];

				if (now != 0) {
					if (left == 0 && up == 0) {
						labelMap[i][j] = ++flag;	// component head
						Label *lTemp = new Label;
						lTemp->value = flag;
						lTemp->equal = NULL;
						labelList.push_back(lTemp);
					}
					else if (left == 0) {
						labelMap[i][j] = up;		// same up
					}
					else if (up == 0) {
						labelMap[i][j] = left;		// same left
					}
					else {
						// check ambiguous
						if (left == up) {
							labelMap[i][j] = up;	// same up and left
						}
						else {
							// update vector
							labelMap[i][j] = left;
							LabelEqual(left, up);
							/*int tt = max(left, up);
							cout << "tt " << labelList[tt]->equal->value << endl;*/
						}
					}
				}
				else {
					labelMap[i][j] = 0;				// don't update
				}
				
			}
		}
	}

	for (int i = 0; i < labelList.size(); i++) {
		struct Label *root;
		for (root = labelList[i]; root->equal != NULL; root = root->equal);
		labelList[i]->value = root->value;
	}

	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			labelMap[i][j] = labelList[labelMap[i][j]]->value;
		}
	}

	vector<vector<int> > color(labelList.size(), vector<int>(3, -1));
	color[0][0] = 0;
	color[0][1] = 0;
	color[0][2] = 0;
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			int label = labelMap[i][j];
			if (color[label][0] == -1) {
				color[label][0] = rand() % 256;
				color[label][1] = rand() % 256;
				color[label][2] = rand() % 256;
			}
			outputImg.at<Vec3b>(i, j)[0] = color[label][0];
			outputImg.at<Vec3b>(i, j)[1] = color[label][1];
			outputImg.at<Vec3b>(i, j)[2] = color[label][2];
		}
	}

	imshow("hihi", outputImg);
	imwrite("output_color.jpg", outputImg);
	waitKey(0);

	return 0;
}