#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <iostream>
#include <math.h>
#define PI 3.14159265

using namespace cv; 
using namespace std;

double image_dist(Mat& image, Point pt1, Point pt2){
    // image euclidean distance factors in image value at point 
    double p1 = image.at<uchar>(pt1.x, pt1.y); // image intensity at pt1 
    double p2 = image.at<uchar>(pt2.x, pt2.y); // image intensity at pt2
    double dst = sqrt((p1-p2)*(p1-p2) + (pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y)); 
    return dst;
}

void k_means_assignment(Mat& image, vector<Point> means, vector<vector<Point> > &regions){
    int c = image.cols; 
    int r = image.rows; 
    for (int m = 0; m < means.size(); m++){
        vector<Point> v; 
        regions.push_back(v);
    }
    for (int i = 0; i < r; i++){
        for (int j = 0; j < c; j++){
            Point pt(i,j); 
            int regIdx = 0;
            double minDist = image_dist(image, means[regIdx], pt);
            for (int m = 1; m < means.size(); m++){ 
                double dist;
                dist = image_dist(image, means[m], pt);
                if (dist < minDist){
                    regIdx = m; minDist = dist;  
                }
                dist = 1797.9;
            }
            regions[regIdx].push_back(pt);
        }
    }
}

void k_means_update(Mat& image, vector<Point> &means, vector<vector<Point> > regions){
    // basically find centroid 
    for (int m = 0; m < means.size(); m++){
        vector<Point> reg = regions[m]; 
        double sumx = 0; double sumy = 0;
        for (int i = 0; i < reg.size(); i++){
            sumx = sumx + reg[i].x; 
            sumy = sumy + reg[i].y; 
        }
        if (reg.size() > 0){
            means[m].x = sumx/reg.size(); 
            means[m].y = sumy/reg.size();
        }
    }
}

double vector_diff(vector<Point> &v1, vector<Point> &v2){
    // assume two vectors have same length
    if (v1.size() != v2.size()){
        cout << "vector difference requires vector to have same length"; 
        return -1; 
    }
    double diff = 0; 
    for (int i = 0; i < v1.size(); i++){
        diff = diff + sqrt((v1[i].x-v2[i].x)*(v1[i].x-v2[i].x)+(v1[i].y-v2[i].y)*(v1[i].y-v2[i].y));
    }
    return diff; 
}

void k_means(Mat& image, vector<Point> initGuess, vector<vector<Point> > &sets ){
    bool done = false; 
    vector<Point> means = initGuess;
    while (!done){
        vector<vector<Point> > regions; 
        vector<Point> old_means(means);
        k_means_assignment(image, means, regions); 
        // cout << means.size() << " " << regions.size() << endl; 
        k_means_update(image, means, regions); 
        if (vector_diff(means, old_means) < 10){
            done = true; 
            sets = regions; 
        }
    }
}

void color_regions(Mat& image, vector<vector<Point> > &regions){
    uchar c[] = {0, 100, 225}; 
    vector<uchar> colors(c, c+3); 
    for (int i = 0; i < colors.size(); i++){
        for (int j = 0; j < regions[i].size(); j++){
            Point pt = regions[i][j]; 
            image.at<uchar>(pt.x, pt.y) = colors[i]; 
        }
    }
}

int main(int argc, char *argv[]){
    Mat img = imread(argv[1], IMREAD_GRAYSCALE );
    vector<Point> guess; 
    vector<vector<Point> > regs; 
    Point pt1(2,2); 
    Point pt2(100, 3000); 
    Point pt3(2, 3000); 
    guess.push_back(pt1); guess.push_back(pt2); guess.push_back(pt3);
    k_means(img, guess, regs); 
    color_regions(img, regs);
    imshow("colored image", img); 
    waitKey(0); 
}