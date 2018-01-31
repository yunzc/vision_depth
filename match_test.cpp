#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"
#include <fstream>
#include <iostream>
#include <math.h>

using namespace cv; 
using namespace cv::xfeatures2d;
using namespace std;

void match(Mat& templ, Mat& img2){
	int window_size = 99; 
	int start = (window_size - 1)/2; 
	Mat out; 
	matchTemplate( img2, templ, out, CV_TM_SQDIFF );
	normalize( out, out, 0, 1, NORM_MINMAX, -1, Mat() );
	double minVal; double maxVal; Point minLoc; Point maxLoc; 
	minMaxLoc( out, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
	Point pt2(minLoc.x+start, minLoc.y+start);
	Point upleft(pt2.x-start, pt2.y-start); 
	Point downrt(pt2.x+start, pt2.y+start);
	cout<< pt2.x << " " << pt2.y << endl; 
	rectangle(img2, upleft, downrt, 0, 1, 8, 0); 
	namedWindow("match", WINDOW_NORMAL); 
	namedWindow("out", WINDOW_NORMAL); 
	imshow("match", img2); 
	imshow("out", out); 
	waitKey(0); 
}

int main(int argc, char *argv[]){
	Mat img1 = imread(argv[1], IMREAD_GRAYSCALE );
	Mat img2 = imread(argv[2], IMREAD_GRAYSCALE);
	pyrDown( img1, img1, Size( img1.cols/2, img1.rows/2 ));
	pyrDown( img2, img2, Size( img2.cols/2, img2.rows/2 ));	int window_size = 99;  
	uchar k[window_size][window_size];
	int start = (window_size - 1)/2; 
	int orig_cols = img1.cols; int orig_rows = img1.rows; 
	int cols = orig_cols + window_size-1; int rows = orig_rows + window_size-1; 
	Mat pimg1;
	copyMakeBorder( img1, pimg1, start, start, start, start, BORDER_REPLICATE );
	Point tplft(3*start,2*start); Point btrt(5*start, 4*start);  
	cout << 3*start << " " << 2*start << "[]" << endl; 
	Rect R(tplft, btrt); 
	Mat templ = pimg1(R); 
	namedWindow("temp", WINDOW_NORMAL); 
	imshow("temp", templ); 
	match(templ, img2); 
}