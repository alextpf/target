#include <math.h>       /* sin */
#include "Segmentor.h"
#include <tuple>

#define PI						3.1415926
#define DEG_TO_RAD				PI / 180.0f
#define NUM_ITER				1
#define ELLIPSE_THRESH			9
#define ELLIPSE_ERR_PERCENTAGE	0.03f //3%

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
void Segmentor::FindConnectedComp(const cv::Mat & img)
{
	cv::Mat connComps;	// Connected components
	cv::Mat stats;		// Stats on each connected component
	cv::Mat centroids;	// Centroids of each connected component
	cv::connectedComponentsWithStats(img, connComps, stats, centroids);

	std::string typeName = type2str(connComps.type());
	printf("%s\n", typeName.c_str());

	int numComponents = stats.rows - 1; // excluding the background

	m_Data.reserve(numComponents);

	for (int i = 0; i < numComponents; i++)
	{
		std::vector< cv::Point2i > tmp;
		m_Data.push_back(tmp);
	}

	for (int r = 0; r < connComps.rows; r++)
	{
		for (int c = 0; c < connComps.cols; c++)
		{
			int label = connComps.at<int>(r, c) - 1;
			if (label != -1)//excluding background
			{
				cv::Point2i pt(c, r);//Notice the order: (c,r), not (r,c), for fitEllipse use
				m_Data[label].push_back(pt);
			}
		}
	}

#ifdef DEBUG //show connected component

	cv::Mat tmp1 = cv::Mat::zeros(img.rows, img.cols, CV_8UC3);
	cv::RNG rng;

	std::cout << numComponents << std::endl << std::endl;

	for (int i = 0; i < numComponents; i++)
	{
		cv::Vec3b color(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));

		int numElements = m_Data[i].size();
		std::cout << numElements << std::endl;

		for (int idx = 0; idx < numElements; idx++)
		{
			int c = m_Data[i][idx].x;
			int r = m_Data[i][idx].y;
			tmp1.at<cv::Vec3b>(r, c) = color;
		}
	}

	cv::imshow("connected components:", tmp1);
#endif // DEBUG

}//FindConnectedComp

void Segmentor::FitEllipse(const cv::Mat & img)
{
	int MIN_ELEMENT_SIZE = img.rows / 5;

	std::vector<bool> keepElement;
	int numComponents = m_Data.size();

#ifdef DEBUG //show ellipse
	cv::Mat tmp1 = cv::Mat::zeros(img.rows, img.cols, CV_8UC3);
#endif

	for (int i = 0; i < numComponents; i++)
	{
		keepElement.push_back(true);//default is to keep this element

		int size = m_Data[i].size();
		if (size > MIN_ELEMENT_SIZE)
		{
			cv::RotatedRect box = cv::fitEllipse(m_Data[i]);

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
			cv::Point ellipseCenter = box.center;
			cv::Point axis ( box.size.width * 0.5f, box.size.height * 0.5f );
			float angleDeg = box.angle;
			float angleRad = angleDeg * DEG_TO_RAD;

			//---------------------
			// Rotation matrix: rotate counter-clockwise by "u" radian
			//
			// | cos(u), -sin(u) |
			// | sin(u),  cos(u) |
			//
			//---------------------
			cv::Matx22f rotMat;
			rotMat(0, 0) = std::cos(angleRad);
			rotMat(0, 1) = -std::sin(angleRad);
			rotMat(1, 0) = -rotMat(0, 1);
			rotMat(1, 1) = rotMat(0, 0);

			// 3. go back and check the m_Data by the model (ellipse)
			int errCount = 0;
			int numErrThresh = size * ELLIPSE_ERR_PERCENTAGE;

			for (int idx = 0; idx < size; idx++)
			{
				cv::Matx21f pt(m_Data[i][idx].y - ellipseCenter.y/*col*/, m_Data[i][idx].x - ellipseCenter.x/*row*/);
				pt = rotMat * pt;

				// now check if the point locates near the ellipse
				float term1 = pt(0, 0) / axis.x;
				float term2 = pt(1, 0) / axis.y;
				float t = term1 * term1 + term2 * term2;

				if (fabs( t - 1) > ELLIPSE_THRESH)
				{
					errCount++;
					if (errCount > numErrThresh)
					{
						keepElement[i] = false;
						break;
					}//if
				}// if
			}//for
		}
		else
		{
			keepElement[i] = false;
			std::cout << "noise: " << i << std::endl;
		}
	}//for (int i = 0; i < numComponents; i++)

#ifdef DEBUG //show connected component
	cv::imshow("ellipse:", tmp1);
#endif
	// 3. re-organize the data
	ReOrganizeData(keepElement);

#ifdef DEBUG //show connected component
	tmp1 = cv::Mat::zeros(img.rows, img.cols, CV_8UC3);
	numComponents = m_Data.size();

	for (int i = 0; i < numComponents; i++)
	{
		cv::RotatedRect box = cv::fitEllipse(m_Data[i]);

		cv::ellipse(tmp1, box, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
		
	}

	cv::imshow("Connected:", tmp1);

#endif
}//FitEllipse

 //=======================================================================
void Segmentor::ReOrganizeData(const std::vector<bool> & keepElement)
{
	int numElement = m_Data.size();
	
	if (keepElement.size() != numElement)
	{
		return;
	}

	int k = 0;
	for (int i = 0; i < numElement; i++)
	{
		if (!keepElement[i])
		{
			continue;
		}

		m_Data[k++] = m_Data[i];
	}

	m_Data.resize(k);

	std::cout << "num elements: " << k << std::endl;
}//ReOrganizeData

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
		
		///////////////////////////////////////////////////
		// Modified RANSAC (random sample consensus):		
		///////////////////////////////////////////////////
		
		// RANSAC does the following:
		// 1. randomly select samples
		// 2. from the samples selected in 1, fit a model to them
		// 3. based on the model obtained in 2, go back and check each m_Data, remove the outliers (based on some threshold)
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
		FindConnectedComp(output);

		// 2. loop through each component and fit it to ellipse
		FitEllipse(output);

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