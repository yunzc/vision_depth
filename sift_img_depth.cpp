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

int get_cam_param(string filename, Mat& CM, Mat& DC){
	FileStorage fs; 
	fs.open(filename, FileStorage::READ);
	fs["camera_matrix"] >> CM;
	fs["distortion_coefficients"] >> DC;  
	fs.release();
	return 0;
}

void find_features(Mat& image, vector<KeyPoint> &keypts, Mat& descriptors){
	// Make sure image is black and white 
	// cvtColor(src, bwsrc, cv::COLOR_RGB2GRAY);
	Ptr<SIFT> detector = SIFT::create();
	detector -> detect( image, keypts ); //detect keypts  
	Ptr<SIFT> extractor = SIFT::create(); 
	extractor -> compute( image, keypts, descriptors ); 
}

void match_points(Mat& descriptors1, Mat& descriptors2, vector<DMatch> &good_matches ){
	FlannBasedMatcher matcher; 
	vector<DMatch> matches; 
	matcher.match( descriptors1, descriptors2, matches );
	double max_dist = 0; double min_dist = 100; 
	
	//-- Quick calculation of max and min distances between keypoints
	for( int i = 0; i < descriptors1.rows; i++ ){ 
		double dist = matches[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}

	for( int i = 0; i < descriptors1.rows; i++ ){ 
		if( matches[i].distance <= max(2*min_dist, 0.02) ){ 
			good_matches.push_back( matches[i]); 
		}
	}	
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

double image_dist(Mat& image, Point pt1, Point pt2){
	// image euclidean distance factors in image value at point 
	double p1 = image.at<uchar>(pt1.x, pt1.y);// image intensity at pt1 
	double p2 = image.at<uchar>(pt2.x, pt2.y); // image intensity at pt2
	return sqrt((pt1.x - pt2.x)*(pt1.x-pt2.y) + (pt1.y - pt2.y)*(pt1.y - pt2.y))*abs(p1-p2)/255; 
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
                double dist = image_dist(image, means[m], pt);
                if (dist < minDist){
                    regIdx = m; minDist = dist; 
                }
            }
            regions[regIdx].push_back(pt);
        }
    }
}

void k_means_update(Mat& image, vector<Point> &means, vector<vector<Point> > regions){
	// basically find centroid  
	// need to bring color into this 
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
        k_means_update(image, means, regions); 
        double update_diff = vector_diff(means, old_means); 
        cout << update_diff << endl; 
        if (update_diff < 1000*means.size()){
            done = true; 
            sets = regions; 
        }
    }
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
	for (int i = 0; i < depths.size(); i ++){
		double intensity = (depths[i] - min)/range*255; 
		colors.push_back((uchar)intensity); 
	}
}

void color_regions(Mat& depthImg, vector<vector<Point> > &regions, vector<uchar> colors){  
    for (int i = 0; i < colors.size(); i++){
        for (int j = 0; j < regions[i].size(); j++){
            Point pt = regions[i][j]; 
            depthImg.at<uchar>(pt.x, pt.y) = colors[i]; 
        }
    }
}

int main(int argc, char *argv[]){
	if( argc != 6 ){ 
		readme(); return -1; 
	}
	string filename = "camera_params.xml";
	Mat Camera_Matrix; Mat Distortion_Coefficients; 
	get_cam_param(filename, Camera_Matrix, Distortion_Coefficients); 
  	Mat img1 = imread(argv[1], IMREAD_GRAYSCALE ); //get first image
  	Mat img2 = imread(argv[2], IMREAD_GRAYSCALE ); //get second image 
  	double t[3] = {atof(argv[3]), 0, atof(argv[4])}; 
  	Mat T(3,1,CV_64F,&t);
  	double ang = atof(argv[5])/360*PI; // angle difference in radians
  	double r[3][3] = {{cos(ang), 0, ang}, {0, 1, 0}, {-sin(ang), 0, cos(ang)}};
    Mat R(3,3,CV_64F,&r); // rotation matrix
  	// find feature vectors for the fist image 
  	vector<KeyPoint> kp1; //keypoints
  	Mat dsp1; //descriptors
  	find_features(img1, kp1, dsp1); 
  	vector<Mat> keyptvect1;
  	for (vector<KeyPoint>::iterator it = kp1.begin(); it != kp1.end(); ++it){
  		Mat ptvect; 
  		physical_XYZ(Camera_Matrix, it->pt, ptvect);
  		keyptvect1.push_back(ptvect);
  	}
  	// find feature vectors for the second image 
  	vector<KeyPoint> kp2; //keypoints
  	Mat dsp2; //descriptors
  	find_features(img2, kp2, dsp2); 
  	vector<Mat> keyptvect2;
  	for (vector<KeyPoint>::iterator it = kp2.begin(); it != kp2.end(); ++it){
  		Mat ptvect; 
  		physical_XYZ(Camera_Matrix, it->pt, ptvect);
  		// rotate vector 
  		ptvect = R*ptvect; 
  		keyptvect2.push_back(ptvect);
  	}
  	vector<DMatch> matches1to2;
  	match_points(dsp1, dsp2, matches1to2);
  	vector<Point> depthPts; // the points corresponding to the location in first image 
  	vector<double> depths; // correspoing depth to depthPts 
  	// matches 
  	for (size_t m =0; m < matches1to2.size(); m++){
  		int i1 = matches1to2[m].queryIdx; //match point of keypoints1
  		int i2 = matches1to2[m].trainIdx; //match point of keypoints2
  		CV_Assert(i1 >= 0 && i1 < static_cast<int>(kp1.size()));
		CV_Assert(i2 >= 0 && i2 < static_cast<int>(kp2.size()));
		// cout<<i1<<" "<<i2<<endl;
		// find intersection vector keyptvect1[i1] and vector keyptvect2[i2]
		double d; 
		find_intersection(keyptvect1[i1], keyptvect2[i2], T, d);
		depthPts.push_back(kp1[i1].pt);
		depths.push_back(d); 
  	}
  	// cluster image with the depth points 
  	vector<Point> guess(depthPts);
  	vector<vector<Point> > regs; 
    // k_means(img1, guess, regs);
    namedWindow("original", WINDOW_NORMAL);
    // namedWindow("depth", WINDOW_NORMAL); 
    namedWindow("depthPoints", WINDOW_NORMAL); 
    imshow("original", img1); 
    // Mat depth = img1.clone(); 
    vector<uchar> colors;
    depths2colors(depths, colors); 
    // color_regions(depth, regs, colors); 
    // imshow("depth", depth); 
    Mat ptsim = img1.clone();
    for (int i = 0; i < depthPts.size(); i++){
    	circle(ptsim, depthPts[i], 10, colors[i], 10, 8); 
    }
    imshow("depthPoints", ptsim); 
    waitKey(0); 
}

// cv::Mat image = cv::imread( "Foam_Image,jpg" );
// cv::Mat imageUndistorted; // Will be the undistorted version of the above image.

// undistort(image, imageUndistorted, Camera_Matrix, Distortion_Coefficients);
void readme(){ 
	cout << " Usage: ./process_img <img1> <img2> <translation_x> <translation_z> <angle difference> (from img2 to img1) note that this program only support one rotation (angle difference in degrees, translation in cm) " << std::endl; }