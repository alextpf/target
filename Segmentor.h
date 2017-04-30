#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc.hpp>

#include "videoprocessor.h"
#include <list>

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

	void FindConnectedComp(const cv::Mat & img);
	bool FitEllipse(const cv::Mat & img);
	bool ReOrganizeData(const std::vector<bool> & keepElement);

	std::vector< std::vector< cv::Point2i > >	m_Data;
	std::vector< cv::RotatedRect >				m_EllipseBox;

	//////////////////////
	// Voting System
	//////////////////////

	// m_numCircleStack is a queue (FIFO),
	// each element stores the number of circles found in a frame
	std::list< int >							m_numCircleStack;

	// m_Count is a vector of std::pairs
	// each pair stores: 1. number of circles, 2. the votes (summing from the queue)
	std::vector<std::pair<int/*num circles*/, int/*num vote*/>> m_Count;

};