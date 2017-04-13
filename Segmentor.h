#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc.hpp>

#include "videoprocessor.h"

class Segmentor : public FrameProcessor
{
public:
	Segmentor() {}

	~Segmentor() {}

	virtual void Process(cv::Mat & input, cv::Mat & output);	

private:
	void DrawContour(
		cv::Mat & frame, 
		const std::vector< std::vector < cv::Point > > & contours, 
		cv::Scalar color);
};