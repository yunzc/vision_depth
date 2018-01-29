#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

Mat img; 
// // Canny arguments 
// Mat dst, detected_edges;

// int edgeThresh = 1;
// int lowThreshold;
// int const max_lowThreshold = 100;
// int ratio = 3;
// int kernel_size = 3;

// static void adjust_thresh(int, void*){
//     blur(img, detected_edges, Size(3,3) ); 
//     Canny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
//     dst = Scalar::all(0);
//     img.copyTo(dst, detected_edges);
//     imshow("Edge", dst);
// }

int main(int argc, char *argv[]){
    img = imread(argv[1], IMREAD_GRAYSCALE);
    Size s = img.size();
    Scalar intensity = img.at<uchar>(100, 3000);
    // imshow("Original Image", img); 
    // waitKey(0);
    // // for color images use <Vec3b> instead of <uchar>
    // cout << intensity.val[0];
    // for (int i=100; i< 300; ++i){
    //     for (int j=100; j<300; ++j){
    //         img.at<uchar>(i,j) = 0;
    //     }
    // }
    // Mat sobelx;
    // Sobel(img, sobelx, CV_32F, 1, 0);
    // imshow("Modified Image", sobelx);
    // waitKey(0);
    // // Playing with Image pyramids 
    // cout << "\n Zoom In-Out demo \n "
    //         "------------------  \n"
    //         " * [i] -> Zoom in   \n"
    //         " * [o] -> Zoom out  \n"
    //         " * [ESC] -> Close program \n" << endl;

    // for(;;)
    // {
    //     imshow( "Modified Image",sobelx);
    //     char c = (char)waitKey(0);
    //     if( c == 27 )
    //     { breakin; }
    //     else if( c == 'i' )
    //     { pyrUp( sobelx, sobelx, Size( sobelx.cols*2, sobelx.rows*2 ) );
    //         printf( "** Zoom In: Image x 2 \n" );
    //     }
    //     else if( c == 'o' )
    //     { pyrDown( sobelx, sobelx, Size( sobelx.cols/2, sobelx.rows/2 ) );
    //         printf( "** Zoom Out: Image / 2 \n" );
    //     }
    // }

    // // Playing with Canny edge detection 
    // cout << "Canny Edge Detection" << endl; 
    // // dst.create(img.size(), img.type() );
    // namedWindow("Edge", WINDOW_NORMAL);
    // createTrackbar( "Min Threshold:", "Edge", &lowThreshold, max_lowThreshold, adjust_thresh);
    // adjust_thresh(0,0);
    // waitKey(0);

    // Surf test 
    int minHessian = 400; 
    Ptr<SURF> detector = SURF::create(minHessian); 
    vector<KeyPoint> keypoints; 
    detector->detect(img, keypoints);
    // // Draw KeyPoints
    // Mat img_keypoints; 
    // drawKeypoints(img, keypoints, img_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    // // Show detected (drawn) keypoints
    // imshow("KeyPoints", img_keypoints); 
    // waitKey(0);



    return 0;

}



