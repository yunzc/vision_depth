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
#define PI 3.14159265

using namespace cv; 
using namespace cv::xfeatures2d;
using namespace std;

void readme();

void match(Mat& img1, Mat& img2, vector<pair<Point, Point> > &matches){
	// use a search window to limit search and speed up time
	// for now it is vertically: 2 window_sizes, horizontally: 3 window sizes 
	// perform block matching on two images to create disparity map
	int window_size = 99; // make sure this is odd
	int start = (window_size - 1)/2; 
	// pad image 
	int orig_cols = img1.cols; int orig_rows = img1.rows; 
	int cols = orig_cols + window_size-1; int rows = orig_rows + window_size-1; 
	Mat pimg1;// padding 
	copyMakeBorder( img1, pimg1, start, start, start, start, BORDER_REPLICATE );
	for (int i = start; i < start + orig_rows; i++){
		cout << i-start << " out of " << orig_rows << endl;
		for (int j = start; j < start + orig_cols; j++){
			// create matching template 
			Point tplft(j-start,i-start); Point btrt(j+start, i+start);  
			Rect R(tplft, btrt); 
			Mat templ = pimg1(R); 
			Mat wimg2 = img2; 
			// template matching 
			Mat out; 
			matchTemplate( wimg2, templ, out, CV_TM_SQDIFF );
			normalize( out, out, 0, 1, NORM_MINMAX, -1, Mat() );
			double minVal; double maxVal; Point minLoc; Point maxLoc; 
			minMaxLoc( out, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
			Point pt1(j-start, i-start); // point is col, row (x, y)
			Point pt2(minLoc.x+start, minLoc.y+start);
			// cout << pt1 << " " << pt2 << endl;  
			pair<Point, Point> ptx(pt1, pt2); 
			matches.push_back(ptx); 
		}
	}
}

int get_cam_param(string filename, Mat& CM, Mat& DC){
	FileStorage fs; 
	fs.open(filename, FileStorage::READ);
	fs["camera_matrix"] >> CM;
	fs["distortion_coefficients"] >> DC;  
	fs.release();
	return 0;
}

void physical_XYZ(Mat& Camera_Matrix, Point P, Mat& out){
	// P = Point(px, py)
	// px, py are the x y pixel locations (col row)
	// [A;B;1] = inv(Cam_Matrix)*[px;py;1]
	// Cam_Matrix = [f 0 cx; 0 f cy; 0 0 1]
	// A = X/Z; B = Y/Z
	Mat invCM; 
	invert(Camera_Matrix, invCM); 
	double v[] = {P.x, P.y, 1}; 
	vector<double> V(v, v+3);
	out = invCM*Mat(V); 
}

void find_intersection(Mat& v1, Mat& v2, Mat& t, double &depth){
	// v1 from first img, v2 for img2, t is translation
	// note v2 has already been rotated 
	double a11 = v1.dot(v1); double a12 = -v1.dot(v2); 
	double a21 = -v1.dot(v2); double a22 = v2.dot(v2); 
	double a[2][2] = {{a11, a12}, {a21, a22}}; 
	Mat A(2,2,CV_64F,&a); 
	// cout << A << endl; 
	double x[2] = {-t.dot(v1), t.dot(v2)};
	Mat X(2,1,CV_64F,&x);
	Mat invA; 
	invert(A, invA); 
	Mat result = invA*X; 
	double lmda = result.at<double>(0,0); 
	double alpha = result.at<double>(1,0);
	Mat pt1 = lmda*v1; 
	Mat pt2 = alpha*v2; 
	// cout << pt1 << " " << pt2 << endl; 
	// for now only extract depth (Z) note this is relative, to get exact need calibration 
	depth = (pt1.at<double>(2,0) + pt2.at<double>(2,0))/2; 
}

void depths2colors(vector<double> depths, vector<uchar> &colors){
	// convert a range of depth to grayscale intensity for visualization 
	// first find max min
	double max = depths[0]; double min = depths[0];
	for (int i = 0; i < depths.size(); i ++){
		if (depths[i] > max){
			max = depths[i];
		}else if (depths[i] < min){
			min = depths[i];
		}
	}
	double range = max - min; 
	cout << "depth range: " << range; 
	for (int i = 0; i < depths.size(); i ++){
		double intensity = (depths[i] - min)/range*255; 
		colors.push_back((uchar)intensity); 
	}
}

void visualize_depth(Mat& out, vector<Point> d_pts, vector<double> depths){
	vector<uchar> colors;
    depths2colors(depths, colors);
    for (int i = 0; i < colors.size(); i++){
    	int r = d_pts[i].y; int c = d_pts[i].x;  
    	out.at<uchar>(r,c) = colors[i]; 
    } 
}

int main(int argc, char *argv[]){
	if( argc != 6 ){ 
		readme(); return -1; 
	}
	Mat img1 = imread(argv[1], IMREAD_GRAYSCALE ); 
	Mat img2 = imread(argv[2], IMREAD_GRAYSCALE );
	// downsample image: lower resolution but faster  
	vector<pair<Point, Point> >m; 
	match(img1, img2, m); 
	// camera calibration and calculation  
	string filename = "camera_params.xml";
	Mat Camera_Matrix; Mat Distortion_Coefficients; 
	get_cam_param(filename, Camera_Matrix, Distortion_Coefficients);
  	double t[3] = {atof(argv[3]), 0, atof(argv[4])}; 
  	Mat T(3,1,CV_64F,&t);
  	double ang = atof(argv[5])/360*PI; // angle difference in radians
  	double r[3][3] = {{cos(ang), 0, ang}, {0, 1, 0}, {-sin(ang), 0, cos(ang)}};
    Mat R(3,3,CV_64F,&r); // rotation matrix
    vector<Point> depthPts;
    vector<double> depths; // correspoing depths the point in first image  
  	for (int i = 0; i < m.size(); i++){
  		Mat ptvect1, ptvect2;
  		physical_XYZ(Camera_Matrix, m[i].first, ptvect1);
  		physical_XYZ(Camera_Matrix, m[i].second, ptvect2); 
  		cout << ptvect1 << " " << ptvect2 << endl;  
  		double d; 
  		find_intersection(ptvect1, ptvect2, T, d);
		depths.push_back(d); 
		depthPts.push_back(m[i].first); 
  	}
  	vector<vector<Point> > regs; 
    namedWindow("original", WINDOW_NORMAL);
    imshow("original", img1);
    namedWindow("depth", WINDOW_NORMAL); 
    visualize_depth(img1, depthPts, depths);
    imshow("depth", img1);   
    waitKey(0); 
}
	
void readme(){ 
	cout << " Usage: ./process_img <img1> <img2> <translation_x> <translation_z> <angle difference> (from img2 to img1) note that this program only support one rotation (angle difference in degrees, translation in cm) " << std::endl; }