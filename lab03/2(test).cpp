#include <iostream>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <vector>
#include <map>

using namespace std;
using namespace cv;

void Ostu_algo(Mat& inputImg, Mat& outputImg);
void Connect_Component(Mat& inputImg, Mat& outputImg);

int main(int argc, char **argv)
{
    Mat inputImg = imread("/Users/Henry/Downloads/lab03/connected-component/output(ostu).jpg");
    cvtColor(inputImg, inputImg, cv::COLOR_BGR2GRAY);
    Mat ostuImg = inputImg.clone();
    Mat outputImg = inputImg.clone();
    
    Ostu_algo(inputImg, ostuImg);
    
    Connect_Component(inputImg, outputImg);
    
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

vector <long> cluster_set[1500];
int cluster_amount = 0;

void value_migrate(long val_a, long val_b){
    
    
    int existing_set_index_a = -1;
    int existing_set_index_b = -1;
    for(int i = 0; i < cluster_amount; i++) {
        //iterate every cluster
        for(int j = 0; j < cluster_set[i].size(); j++) {
            if(val_a == cluster_set[i][j]) {
                existing_set_index_a = i;
            }
            if(val_b == cluster_set[i][j]) {
                existing_set_index_b = i;
            }

        }
    }
    
    
    //if both find set, and both in same set then don't need to update
    if(existing_set_index_a == existing_set_index_b && existing_set_index_a != -1) {
        return ;
    }
    //if both find set, and both in different set, then need to migrate two set
    else if(existing_set_index_a != -1 && existing_set_index_b != -1) {
        
        int existing_new_index = -1;
        //migrate to minimum one
        if(cluster_set[existing_set_index_a][0] < cluster_set[existing_set_index_b][0]) {
            existing_new_index = existing_set_index_a;
            
            //append the existing_set_b to a
            cluster_set[existing_new_index].insert(cluster_set[existing_new_index].end(), cluster_set[existing_set_index_b].begin(), cluster_set[existing_set_index_b].end());
            
            //clean the existing_set_b
            cluster_set[existing_set_index_b].clear();
            for(int i = existing_set_index_b; i< cluster_amount ; i++) {
                cluster_set[i].swap(cluster_set[i + 1]);
            }
            
            //update cluster_amount
            cluster_amount--;
            
        } else {
            existing_new_index = existing_set_index_b;
            
            //append the existing_set_a to b
            cluster_set[existing_new_index].insert(cluster_set[existing_new_index].end(), cluster_set[existing_set_index_a].begin(), cluster_set[existing_set_index_a].end());
            
            //clean the existing_set_a
            cluster_set[existing_set_index_a].clear();
            for(int i = existing_set_index_a; i< cluster_amount ; i++) {
                cluster_set[i].swap(cluster_set[i + 1]);
            }
            
            //update cluster_amount
            cluster_amount--;
            
        }

    }
    //if find the existing_set_index_a index, add val_b to the set
    else if(existing_set_index_a != -1) {
        cluster_set[existing_set_index_a].push_back(val_b);
    }
    //if find the existing_set_index_b index, add val_a to the set
    else if(existing_set_index_b != -1) {
        cluster_set[existing_set_index_b].push_back(val_a);
    }
    //if no existing set found, add create new set for val_a, val_b
    else {
        
        //set the minimun one
        if(val_a < val_b) {
            //create new set
            cluster_set[cluster_amount].push_back(val_a);
            cluster_set[cluster_amount].push_back(val_b);
            cluster_amount++;
        } else {
            //create new set
            cluster_set[cluster_amount].push_back(val_b);
            cluster_set[cluster_amount].push_back(val_a);
            cluster_amount++;
        }
        
    }
    
    
    
    /*
    int existing_set_index = -1;
    // find the cluster_map see where the cluster_set
    iter_a = cluster_map.find(val_a);
    iter_b = cluster_map.find(val_b);

    
    if(iter != cluster_map.end()) {
        //if find the existing set index, add val_b to the set
        existing_set_index = iter->first;
        cluster_map.insert(make_pair(existing_set_index, val_b));
        return ;
    }
    

    if(iter != cluster_map.end()) {
        //if find the existing set index, add val_a to the set
        existing_set_index = iter->first;
        cluster_map.insert(make_pair(existing_set_index, val_a));
        return ;
    }
    
    //if no existing set found, add new set
    if(existing_set_index == -1) {
        cluster_map.insert(make_pair(val_a, val_b));
        return ;
    }
    */
    
    return ;
}

void Connect_Component(Mat& inputImg, Mat& outputImg) {

    Size mat_size = inputImg.size();
    int rows = mat_size.height;
    int cols = mat_size.width;
    long recordImg[rows][cols];
    
    
//    for(int i = 0; i < rows; i ++) {
//        for (int j = 0; j < cols; j++) {
//            recordImg[i][j] = inputImg.at<uchar>(i, j);
//        }
//    }
    
    //record the cluster index
    long seperate_index = 1;

    for(int i = 0; i < rows; i++) {

        //pass 1
        for(int j = 0; j < cols; j++) {
            
            //record top, left value
            long beside_top = 0;
            long beside_left = 0;
            
            if(inputImg.at<uchar>(i, j) != 0) {
                
                recordImg[i][j] = inputImg.at<uchar>(i, j);
                
                //get the top value
                if(recordImg[i-1][j] != 0 && i - 1 >= 0) {
                    beside_top = recordImg[i-1][j];
                }
                
                //get the left value
                if(recordImg[i][j-1] != 0 && j - 1 >= 0) {
                    beside_left = recordImg[i][j-1];
                }
                
                //check if there have both top and left -> migrate point
                if(beside_top != 0 && beside_left != 0 && beside_top != beside_left) {
                    value_migrate(beside_top, beside_left);
                    //set the temp value to top
                    recordImg[i][j] = beside_top;
                }
                
                //if only top or left get != 0 value, set it to top or left. Otherwiese set to new cluster
                if(beside_top != 0) {
                    recordImg[i][j] = beside_top;
                } else if(beside_left != 0) {
                    recordImg[i][j] = beside_left;
                } else {
                    //new cluster exist
                    recordImg[i][j] = seperate_index;
                    seperate_index++;
                }
            } else {
                recordImg[i][j] = 0;
            }
            
        }
        
        //printf("%d ", cluster_amount);
    }
 
    
    
    //pass 2
    for(int i = 0; i < rows; i++) {
        for(int j = 0; j < cols; j++) {
            if(recordImg[i][j] != 0) {
                
                bool migrate_found = false;
                
                //find the migrate value for Img
                for(int k = 0; k < cluster_amount; k++) {
                    //iterate every cluster
                    for(int m = 0; m < cluster_set[k].size(); m++) {
                        if(recordImg[i][j] == cluster_set[k][m]) {
                            outputImg.at<uchar>(i, j) = cluster_set[k][0];
                            migrate_found = true;
                            break;
                        }
                    }
                }
                
                //if no migrate value for Img, single
                if(migrate_found == false)
                    outputImg.at<uchar>(i, j) = recordImg[i][j];

                if( recordImg[i][j] > 255 ) {
                    
                    //printf("%ld ", &recordImg[i][j]);
                    
                    
                }
                
            } else {
                outputImg.at<char>(i, j) = 0;
            }
        }
    }
    
   
    
    
    return ;
}

