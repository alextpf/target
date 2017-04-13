#include "Segmentor.h"
#include <tuple>

#define NUM_ITER 1

// color definition
#define BLUE   cv::Scalar(255, 0, 0) //BGR
#define GREEN  cv::Scalar(0, 255, 0)
#define RED    cv::Scalar(0, 0, 255)
#define YELLOW cv::Scalar(0, 255, 255)
#define WHITE  cv::Scalar(255, 255, 255)
#define BLACK  cv::Scalar(0,0,0)

#define DEBUG

//=======================================================================
// helper function to show type
std::string type2str(int type)
{
	std::string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch (depth) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}

	r += "C";
	r += (chans + '0');

	return r;
}//std::string type2str(int type)

//=======================================================================
void Segmentor::Process(cv::Mat & input, cv::Mat & output)
{
	int kernelSize = 3;
	double std = 2.0;

	// canny low & heigh threshold
	int low = 100;
	int high = 200;

	//// do canny
	static bool doCanny = true;
	if (doCanny)
	{
		cv::GaussianBlur(input, output, cv::Size(kernelSize, kernelSize),std,std);

		//std::string typeName = type2str(output.type());
		//printf("%s\n", typeName.c_str());

		cv::Mat tmp;
		output.convertTo(tmp, CV_8UC1);
		
#ifdef DEBUG
		cv::imshow("gauss:", output);
#endif // DEBUG

		//typeName = type2str(tmp.type());
		//printf("%s\n", typeName.c_str());

		cv::Canny(tmp, output, low, high);

#ifdef DEBUG
		cv::imshow("canny:", output);
#endif // DEBUG

		// do noise reduction
		/*dilate(output, output, cv::Mat(), cv::Point(-1, -1), NUM_ITER);
		erode(output, output, cv::Mat(), cv::Point(-1, -1), NUM_ITER );*/

		cv::Mat ellipse = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));	// Structuring element to find clay target

		cv::morphologyEx(output, output, cv::MORPH_CLOSE, ellipse, cv::Point(-1, -1), 1);

#ifdef DEBUG
		cv::imshow("noise reduction:", output);
#endif // DEBUG

//		// find contours
//		std::vector< cv::Vec4i > m_Hierarchy;
//		std::vector< std::vector < cv::Point > > contours;
//		cv::findContours(output, contours, m_Hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
//
//#ifdef DEBUG
//		cv::imshow("contours:", output);
//		cv::Mat tmpp = cv::Mat::zeros(output.rows, output.cols, output.type());
//		DrawContour(tmpp, contours, WHITE);
//		cv::imshow("contours", tmpp);
//#endif // DEBUG

		///////////////////////////////////////////////////
		// Modified RANSAC (random sample consensus):		
		///////////////////////////////////////////////////
		
		// RANSAC does the following:
		// 1. randomly select samples
		// 2. from the samples selected in 1, fit a model to them
		// 3. based on the model obtained in 2, go back and check each data, remove the outliers (based on some threshold)
		// 4. loop 1 - 3 until converge
		//
		// Now for our case, we have some assumption:
		// i. each circle are spatially far away from each other. They are physically separate, 
		// and there's no 2 circles that connects to each other through noise (for some target paper, there are
		// numbers located in between each circle, and after doing "closing", it's possible 2 circles are then 
		// connected to each other, i.e. being "closed").
		//
		// ii. the target paper remains a plane, but the plane isn't necessarily perpendicular to the viewing angle.
		//     This assumption can diviate from the true especailly when the paper vibrates/distored by the winds.
		//
		// Modified algorithm:
		// a. Each connected component is a sample set. So if there are n connected components,
		// there are n sample set. 
		// b. for each sample set, use all of the sample to find the fitted model. In our case, an ellipse
		// c. go back to check the sample set, and remove the sample set if some percentage of sample don't fall on the ellipse
		// d. loop a - c for all sample set


		// 1. find connected components
		cv::Mat connComps;	// Connected components
		cv::Mat stats;		// Stats on each connected component
		cv::Mat centroids;	// Centroids of each connected component
		cv::connectedComponentsWithStats(output, connComps, stats, centroids);

		std::string typeName = type2str(connComps.type());
		printf("%s\n", typeName.c_str());

		int numComponents = stats.rows - 1; // excluding the background
		std::vector< std::vector< cv::Point2i > > data;
		data.reserve(numComponents);

		for (int i = 0; i < numComponents; i++)
		{
			std::vector< cv::Point2i > tmp;
			data.push_back( tmp );
		}

		for (int r = 0; r < connComps.rows; r++)
		{
			for (int c = 0; c < connComps.cols; c++)
			{
				int label = connComps.at<int>(r, c) - 1;
				if (label != -1)//excluding background
				{
					cv::Point2i pt(c,r);//Notice the order: (c,r), not (r,c), for fitEllipse use
					data[label].push_back( pt );
				}
			}
		}

#ifdef DEBUG //show connected component
		
		cv::Mat tmp1 = cv::Mat::zeros(output.rows, output.cols, CV_8UC3);		
		cv::RNG rng;

		std::cout << numComponents << std::endl << std::endl;

		for (int i = 0; i < numComponents; i++)
		{
			cv::Vec3b color(rng.uniform(0,256), rng.uniform(0, 256), rng.uniform(0, 256));
			
			int numElements = data[i].size();
			std::cout << numElements << std::endl;

			for (int idx = 0; idx < numElements; idx++)
			{
				int c = data[i][idx].x;
				int r = data[i][idx].y;
				tmp1.at<cv::Vec3b>(r, c) = color;
			}
		}

		cv::imshow("connected components:", tmp1);
#endif // DEBUG

		// 2. loop through each component and fit it to ellipse

		int MIN_ELEMENT_SIZE = output.rows / 12;

		for (int i = 0; i < numComponents; i++)
		{
			int size = data[i].size();
			if (size > MIN_ELEMENT_SIZE)
			{
				cv::RotatedRect box = cv::fitEllipse(data[i]);

#ifdef DEBUG //show ellipse
				cv::ellipse(tmp1, box, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
				cv::ellipse(tmp1, box.center, box.size*0.5f, box.angle, 0, 360, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
				cv::Point2f vtx[4];
				box.points(vtx);
				for (int j = 0; j < 4; j++)
				{
					cv::line(tmp1, vtx[j], vtx[(j + 1) % 4], cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
				}
#endif
				// 3. go back and check the data by the model (ellipse)
			}
			else
			{
				std::cout << "noise: " << i << std::endl;
			}
		}

#ifdef DEBUG //show connected component
		cv::imshow("ellipse:", tmp1);
#endif
		// 3. 
	}//if (doCanny)

	// try Hough Transform for circle
	static bool doHough = false;
	if (doHough)
	{
		/// Reduce the noise so we avoid false circle detection
		cv::GaussianBlur(input, output, cv::Size(kernelSize, kernelSize), std, std);

		cv::imshow("gauss input", output);

		cv::cvtColor(output, output, CV_BGR2GRAY);

		std::vector<cv::Vec3f> circles;

		/// Apply the Hough Transform to find the circles
		cv::HoughCircles(output, circles, CV_HOUGH_GRADIENT, 1, output.rows / 8, 200, 100, 0, 0);

		/// Draw the circles detected
		int size = circles.size();
		for (size_t i = 0; i < size ; i++)
		{
			cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			int radius = cvRound(circles[i][2]);
			// circle center
			cv::circle(output, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
			// circle outline
			cv::circle(output, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
		}
	}// if (doHough)

	cv::imshow("test input", output);

	return;
}//Process

//=======================================================================
void Segmentor::DrawContour(
	cv::Mat & frame, 
	const std::vector< std::vector < cv::Point > > & contours,
	cv::Scalar color)
{
	// iterate through all the top-level contours,
	// draw each connected component with its own random color
	int contourThickness = 1;
	const int numContours = contours.size();
	for (int idx = 0; idx < numContours; idx++)
	{
		const int contourSize = contours[idx].size();

		for (int m = 0; m < contourSize; m++)
		{
			cv::line(frame, contours[idx][m], contours[idx][(m + 1)%contourSize], color);
		}
	}// for idx
}//DrawContour