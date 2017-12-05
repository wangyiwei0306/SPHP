#include "coorTrans.h"


void dotDivide(Mat src1, Mat src2, Mat &dst)
{
	assert(src1.rows == src2.rows&&src1.cols == src2.cols);
	dst = Mat(src1.rows,src1.cols,src1.type());

	for (int i = 0; i < src1.rows; i++)
	{
		double *data1 = src1.ptr<double>(i);
		double *data2 = src2.ptr<double>(i);
		double *data_dst = dst.ptr<double>(i);
		for (int j = 0; j < src1.cols; j++)
		{
			if (data2[j] == 0)
			{
				cout << "矩阵有误，分子为0" << endl;
				break;
			}
			data_dst[j] = data1[j] / data2[j];
		}
	}
}

Mat matrix2Array(Mat matrix, Mat mask)//是按列生成数组的
{
	Mat mask_double = Mat(mask.rows,mask.cols, matrix.type());
	mask.convertTo(mask_double, matrix.type());
	//cout << mask << endl;
	Mat out = matrix.mul(mask_double);
	
	Mat mask_NonZero;
	mask.copyTo(mask_NonZero);
	findNonZero(mask, mask_NonZero);

	int rows = mask_NonZero.rows;
	Mat mask_array = Mat(rows,1,CV_64F);

	int k = 0;
	for (int i = 0; i < matrix.cols; i++)
	{
		//double* data = out.ptr<double>(i);

		for (int j = 0; j < matrix.rows; j++)
		{
			double value = out.at<double>(j, i);
			if (value!= 0)
			{
				//double value = value;
				mask_array.at<double>(k, 0) = value;
				k++;
			}
		}
	}
	return mask_array;
}

void array2Matrix(Mat array, Mat mask,Mat &matrix)
{
	//cout << mask << endl;
	//Mat matrix = Mat(mask.rows, mask.cols, CV_64F, Scalar(0));
	int k = 0;
	for (int i = 0; i < mask.cols; i++)
	{
		//double* data = matrix.ptr<double>(i);
		//uchar* data_mask = mask.ptr<uchar>(i);
		for (int j = 0; j < mask.rows; j++)
		{
			if (mask.ptr<uchar>(j)[i] != 0)
			{
				matrix.at<double>(j, i) = array.at<double>(k, 0);
				//data[j] = array.at<double>(k, 0);
				k++;
			}
		}
	}
	
}

void applyTransform(double i, double j, Mat H, double& outi, double &outj)
{
	double scale = H.at<double>(2, 0)*i + H.at<double>(2, 1)*j + H.at<double>(2, 2);

	outi = (H.at<double>(0, 0)*i + H.at<double>(0, 1)*j + H.at<double>(0, 2)) / scale;
	outj = (H.at<double>(1, 0)*i + H.at<double>(1, 1)*j + H.at<double>(1, 2)) / scale;
}
Mat applyTransform(Mat input,Mat t)
{
	Mat dst = Mat(input.rows, 2,CV_64F);
	double t00 = t.at<double>(0, 0); double t01 = t.at<double>(0, 1); double t02 = t.at<double>(0, 2);
	double t10 = t.at<double>(1, 0); double t11 = t.at<double>(1, 1); double t12 = t.at<double>(1, 2);
	double t20 = t.at<double>(2, 0); double t21 = t.at<double>(2, 1); double t22 = t.at<double>(2, 2);
	for (int i = 0; i < input.rows; i++)
	{
		double *data_out = dst.ptr<double>(i);
		double *data_input = input.ptr<double>(i);
		double scale = data_input[0] * t20 + data_input[1] * t21 +  t22;

		data_out[0] = (data_input[0] * t00 + data_input[1] * t01 +  t02)/scale;
		data_out[1] = (data_input[0] * t10 + data_input[1] * t11 +  t12) / scale;
	}
	return dst;
}
Mat matNot(Mat input)//input中所有不等于0的数都等于1
{
	Mat output = Mat(input.rows, input.cols, CV_64F, Scalar(0));
	for (int i = 0; i < input.rows; i++)
	{
		double* data_input = input.ptr<double>(i);
		double* data_output = output.ptr<double>(i);
		for (int j = 0; j < input.cols; j++)
		{
			if (data_input[j] != 0)
			{
				data_output[j] = 1;
			}
		}
	}
	return output;
}