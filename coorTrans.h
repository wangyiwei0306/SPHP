#include <iostream>
#include <time.h>
#include <fstream>

//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/legacy/legacy.hpp"
//#include "opencv2/nonfree/features2d.hpp"
//#include "opencv2/stitching/stitcher.hpp"

#include "opencv3\opencv.hpp"

using namespace cv;
using namespace std;

Mat matrix2Array(Mat matrix, Mat mask);
void array2Matrix(Mat array, Mat mask, Mat &matrix);
void dotDivide(Mat src1, Mat src2, Mat &dst);
void applyTransform(double i, double j, Mat H, double& outi, double &outj);
Mat applyTransform(Mat input, Mat t);
Mat matNot(Mat input);