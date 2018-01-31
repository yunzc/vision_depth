#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace cv; 
using namespace std;
// pyrDown( img1, img1, Size( img1.cols/2, img1.rows/2 ));
	// pyrDown( img2, img2, Size( img2.cols/2, img2.rows/2 ));
	// pyrDown( img1, img1, Size( img1.cols/2, img1.rows/2 ));
	// pyrDown( img2, img2, Size( img2.cols/2, img2.rows/2 ));
int main(int argc, char *argv[]){
	Mat img1 = imread(argv[1]); 
	pyrDown( img1, img1, Size( img1.cols/2, img1.rows/2 ));
	pyrDown( img1, img1, Size( img1.cols/2, img1.rows/2 ));
	string filename = argv[2];
	imwrite( filename, img1 );

}