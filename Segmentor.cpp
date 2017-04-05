#include "Segmentor.h"

void Segmentor::Process(cv::Mat & input, cv::Mat & output)
{
	// do canny
	int low = 125;
	int high = 350;
	cv::Canny(input, output, low, high);

	cv::imshow("test input", output);

	return;
}