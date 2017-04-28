#include <math.h>       /* sin */
#include "Segmentor.h"
#include <tuple>

#define PI							3.1415926
#define DEG_TO_RAD					PI / 180.0f
#define NUM_ITER					1
#define ELLIPSE_THRESH				10
#define ELLIPSE_ERR_PERCENTAGE		0.05f
#define EPSLON						5 * 5 // for ellipse less than 5 pixels, ignore it
#define ASPECT_RATIO_THRESH			4.0f
#define ELLIPSE_OFF_CENTER_THRESH	0.7f
#define SAME_ELLIPSE_THRESH_RATIO   0.11f
#define CENTER_DIF_THRESH_RATIO     0.05f			

// color definition
#define BLUE   cv::Scalar(255, 0, 0) //BGR
#define GREEN  cv::Scalar(0, 255, 0)
#define RED    cv::Scalar(0, 0, 255)
#define YELLOW cv::Scalar(0, 255, 255)
#define WHITE  cv::Scalar(255, 255, 255)
#define BLACK  cv::Scalar(0,0,0)

//#define DEBUG
//#define DEBUG_ELLIPSE
//#define DEBUG_ELLIPSE3
//#define DEBUG_ELLIPSE2

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

	m_Data.clear();// init m_Data

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

bool Segmentor::FitEllipse(const cv::Mat & img)
{
	int MIN_ELEMENT_SIZE = img.rows / 5;

	std::vector<bool> keepElement;
	int numComponents = m_Data.size();

#ifdef DEBUG //show ellipse
	cv::Mat tmp1 = cv::Mat::zeros(img.rows, img.cols, CV_8UC3);
#endif

	m_EllipseBox.clear();

	for (int i = 0; i < numComponents; i++)
	{
		keepElement.push_back(true);//default is to keep this element
		
		m_EllipseBox.push_back(cv::RotatedRect());// dummy var

		// 1. if the ellipse is too small, delete it
		int size = m_Data[i].size();
		if (size < MIN_ELEMENT_SIZE)
		{
			std::cout << "noise" << std::endl;
			keepElement[i] = false;
			continue;
		}

		cv::RotatedRect box = cv::fitEllipse(m_Data[i]);
		m_EllipseBox[i] = box;

#ifdef DEBUG_ELLIPSE //show individual ellipse

		cv::Mat tmp3 = cv::Mat::zeros(img.rows, img.cols, CV_8UC3);

		cv::Vec3b color(255, 255, 255);

		for (int idx = 0; idx < size; idx++)
		{
			int c = m_Data[i][idx].x;
			int r = m_Data[i][idx].y;
			tmp3.at<cv::Vec3b>(r, c) = color;
		}

		cv::ellipse(tmp3, box, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

		char name[50];
		sprintf_s(name, "ellipse %d:", i);
		cv::imshow(name, tmp3);
#endif

#ifdef DEBUG //show ellipse
		cv::ellipse(tmp1, box, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
		
		/*cv::Point2f vtx[4];
		box.points(vtx);
		for (int j = 0; j < 4; j++)
		{
			cv::line(tmp1, vtx[j], vtx[(j + 1) % 4], cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
		}*/
#endif

		cv::Point ellipseCenter = box.center; // in Image coordinate, i.e. origin at top-left corner, x pointing right, y pointing down
		cv::Point axis(box.size.width * 0.5f, box.size.height * 0.5f);
		float angleDeg = box.angle;
		float angleRad = angleDeg * DEG_TO_RAD;

		// 2. if the aspect ratio of the detected ellipse is greater than some threshold, delete it
		float longSide, shortSide;

		if (box.size.width > box.size.height)
		{
			longSide = box.size.width;
			shortSide = box.size.height;
		}
		else
		{
			longSide = box.size.height;
			shortSide = box.size.width;
		}

		const float aspectRatio = longSide / shortSide;

		if (aspectRatio > ASPECT_RATIO_THRESH)
		{
			std::cout << "aspect ratio too large" << std::endl;
			keepElement[i] = false;
			continue;
		}

		// 3. if the center of the ellipse is too far away from the image center, delete it
		cv::Point imageCenter((float)img.cols / 2.0f, (float)img.rows / 2.0f);
		cv::Point dif = imageCenter - ellipseCenter;
		float difRatioX = std::fabs(dif.x) * 2.0f / (float)img.cols;
		float difRatioY = std::fabs(dif.y) * 2.0f / (float)img.rows;

		if (difRatioX > ELLIPSE_OFF_CENTER_THRESH || difRatioY > ELLIPSE_OFF_CENTER_THRESH)
		{
			std::cout << "ellipse center too close to edge" << std::endl;
			keepElement[i] = false;
			continue;
		}

		//---------------------
		// Rotation matrix: rotate counter-clockwise by "u" radian
		//
		// | cos(u), -sin(u) |
		// | sin(u),  cos(u) |		
		//---------------------
		cv::Matx22f rotMat;
		rotMat(0, 0) = std::cos(angleRad);
		rotMat(0, 1) = -std::sin(angleRad);
		rotMat(1, 0) = -rotMat(0, 1);
		rotMat(1, 1) = rotMat(0, 0);

		// 4. go back and check the m_Data by the model (ellipse)
		int errCount = 0;
		int numErrThresh = size * ELLIPSE_ERR_PERCENTAGE;

#ifdef DEBUG_ELLIPSE2		
		cv::Mat tmp4 = cv::Mat::zeros(img.rows, img.cols, CV_8UC3);
		
		cv::Vec3b color(255, 255, 255);

		//m_Data stores (column, row)
		for (int idx = 0; idx < size; idx++)
		{	
			cv::Matx21f pt(m_Data[i][idx].x - ellipseCenter.x/*col*/, -m_Data[i][idx].y + ellipseCenter.y/*row*/); // such that x pointing right, and y pointing up
			pt = rotMat * pt;

			// convert back to image coordinate
			int c = pt(0, 0) + img.cols / 2;
			int r = img.rows / 2 - pt(1, 0);
			if (c >= img.cols)
			{
				c = img.cols - 1;
			}
			else if (c < 0)
			{
				c = 0;
			}

			if (r >= img.rows)
			{
				r = img.rows - 1;
			}
			else if (r < 0)
			{
				r = 0;
			}

			tmp4.at<cv::Vec3b>(r, c) = color;
		}

		cv::Vec3b red(0, 0, 255);
		// now draw the ellipse
		// (x/a)^2+(y/b)^2 = 1
		
		for (int r = 0 ; r < img.rows; r++)
		{
			for (int c = 0; c < img.cols; c++)
			{
				float term1 = (float)(c - img.cols / 2) / axis.x;
				float term2 = (float)(img.rows / 2 - r) / axis.y;
				float tt = std::fabs(term1 * term1 + term2 * term2 - 1.0f);
				if (tt < 0.01)
				{
					tmp4.at<cv::Vec3b>(r, c) = red;
				}
			}
		}

		char name[50];
		sprintf_s(name, "Ortho ellipse %d:", i);
		cv::imshow(name, tmp4);
#endif
		for (int idx = 0; idx < size; idx++)
		{
			cv::Matx21f pt(m_Data[i][idx].x - ellipseCenter.x/*col*/, -m_Data[i][idx].y + ellipseCenter.y/*row*/); // such that x pointing right, and y pointing up
			pt = rotMat * pt;

			// now check how far the point is from the ellipse
			//
			//             Y
			//             ^    * Pt(x,y)
			//           b_|_  /
			//       .     |  /  .
			//     .       | / u   . a
			//  --|--------|--------|---> X
			//     .       |       .
			//       .    _|_    .
			//             |
			//
			// Ellipse eq: (x/a)^2 + (y/b)^2 = 1
			//
			// Problem: given a point Pt(x,y), find its distance to the ellipse.
			// - Suppose the distance between Pt and origin is t (i.e. t = sqrt(x^2 + y^2) )
			// - Suppose the closest point on the ellipse to Pt is m, and it has distance to the origin of r.
			// The problem here becomes to find r, and the answer to this problem is (t - r).
			//
			// Derivation:
			// The coordinate of m can be written as m = ( r*cos(u), r*sin(u) )
			// where u is the angle between r and X-axis.
			// Substitude m into ellipse eq:
			// (r*cos(u)/a)^2+(r*sin(u)/b)^2 = 1
			// r^2 * ( (cos(u)/a)^2 + (sin(u)/b)^2 ) = 1
			// r^2 = 1 / ( (cos(u)/a)^2 + (sin(u)/b)^2 )
			// where cos(u) and sin(u) can be easily obtained by:
			// - cos(u) = x / sqrt(x^2+y^2) = x / t
			// - sin(u) = y / sqrt(x^2+y^2) = y / t

			const float tSqr = pt(0, 0) * pt(0, 0) + pt(1, 0) * pt(1, 0);

			if (tSqr < EPSLON)
			{
				std::cout << "ellipse too small" << std::endl;
				keepElement[i] = false;
				break;
			}

			const float t = std::sqrt(tSqr);

			const float cos_u = pt(0, 0) / t;
			const float sin_u = pt(1, 0) / t;

			float term1 = cos_u / axis.x;
			float term2 = sin_u / axis.y;

			const float r = 1.0f / std::sqrt(term1 * term1 + term2 * term2);
			const float diff = fabs(t - r);
			if ( diff > ELLIPSE_THRESH)
			{
				errCount++;
				if (errCount > numErrThresh)
				{
					std::cout << "data doesn't resemble an ellipse" << std::endl;
					keepElement[i] = false;
					break;
				}//if
			}// if
		}//for
	}//for (int i = 0; i < numComponents; i++)

#ifdef DEBUG //show connected component
	cv::imshow("ellipse:", tmp1);
#endif

	// 3. re-organize the data
	if (!ReOrganizeData(keepElement))
	{
		return false;
	}

	// 4. merge the ellipse if needed (this happens if the ellipse is broken, not in a single connected component)
	numComponents = m_Data.size();
	std::vector<int> mark;
	for (int i = 0; i < numComponents; i++)
	{
		mark.push_back(-1);
	}
	
	for (int i = 0; i < numComponents; i++)
	{

#ifdef DEBUG_ELLIPSE3 //show individual ellipse

		cv::Mat tmp3 = cv::Mat::zeros(img.rows, img.cols, CV_8UC3);

		cv::Vec3b color(255, 255, 255);
		int size = m_Data[i].size();

		for (int idx = 0; idx < size; idx++)
		{
			int c = m_Data[i][idx].x;
			int r = m_Data[i][idx].y;
			tmp3.at<cv::Vec3b>(r, c) = color;
		}
		const cv::RotatedRect & box = m_EllipseBox[i];

		cv::ellipse(tmp3, box, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

		char name[50];
		sprintf_s(name, "ellipse3 %d:", i);
		cv::imshow(name, tmp3);
#endif
		if (mark[i] == -1)
		{
			mark[i] = i;
			const cv::RotatedRect & box1 = m_EllipseBox[i];
			for (int j = i + 1; j < numComponents; j++)
			{
				if (mark[j] == -1)
				{
					const cv::RotatedRect & box2 = m_EllipseBox[j];
					const float widthDif = std::fabs(box1.size.width - box2.size.width);
					const float heightDif = std::fabs(box1.size.height - box2.size.height);
					const float centerDif = std::fabs(box1.center.x - box2.center.x) + std::fabs(box1.center.y - box2.center.y); // city block distance

					float siz = std::min(box1.size.width, box2.size.width);

					const float SAME_ELLIPSE_THRESH = siz * SAME_ELLIPSE_THRESH_RATIO;
					
					if (widthDif < SAME_ELLIPSE_THRESH && heightDif < SAME_ELLIPSE_THRESH && centerDif < SAME_ELLIPSE_THRESH * 2.0f )
					{
						mark[j] = i; // same mark, to be merged
					} // if
				}// if
			}// for j
		}//if
	}// for i

	// do the merge
	keepElement.clear();

	for (int i = 0; i < numComponents; i++)
	{	
		keepElement.push_back(true);

		if (mark[i] == -1)
		{
			keepElement[i] = false;
			continue;
		}

		bool doMerge = false;

		for (int j = i + 1; j < numComponents; j++)
		{
			if (mark[i] == mark[j])
			{
				mark[j] = -1;
				doMerge = true;
				m_Data[i].insert(m_Data[i].end(), m_Data[j].begin(), m_Data[j].end());
			}
		}
	}// for i
	
	if (!ReOrganizeData(keepElement))
	{
		return false;
	}

	// 5. remove those ellipse that doesn't have a common center
	// by clustering
	std::vector<std::pair<cv::Point,int>> concentricCircles;
	std::vector<int> map;

	float CENTER_DIF_THRESH = (float)img.rows * CENTER_DIF_THRESH_RATIO * 2.0f;

	numComponents = m_Data.size();
	for (int i = 0; i < numComponents; i++)
	{
		map.push_back(-1); // init

		const cv::Point & center1 = m_EllipseBox[i].center;
		const int siz = concentricCircles.size();
		bool found = false;

		for (int j = 0; j < siz; j++)
		{
			const cv::Point center2 = concentricCircles[j].first;

			const float centerDif = std::fabs(center1.x - center2.x) + std::fabs(center1.y - center2.y); // city block distance
			
			if (centerDif < CENTER_DIF_THRESH)
			{
				found = true;
				map[i] = j;
				concentricCircles[j].second++;
				break;
			}// if 
		}// for j

		if (!found)
		{
			std::pair<cv::Point, int> p(center1, 0);
			concentricCircles.push_back(p);
			map[i] = concentricCircles.size() - 1;
		}// if
	}// for i

	const int siz = concentricCircles.size();
	int max = -1;
	int maxIdx = -1;

	for (int i = 0; i < siz; i++)
	{
		int vote = concentricCircles[i].second;
		if ( vote > max)
		{
			max = vote;
			maxIdx = i;
		}
	}// for i

	keepElement.clear();
	
	for (int i = 0; i < numComponents; i++)
	{
		if (map[i] != maxIdx)
		{
			keepElement.push_back(false);
		}
		else
		{
			keepElement.push_back(true);
		}
	}

	if (!ReOrganizeData(keepElement))
	{
		return false;
	}

#ifdef DEBUG //show connected component
	cv::Mat tmp2 = cv::Mat::zeros(img.rows, img.cols, CV_8UC3);
	numComponents = m_EllipseBox.size();

	for (int i = 0; i < numComponents; i++)
	{
		const cv::RotatedRect & box = m_EllipseBox[i];

		cv::ellipse(tmp2, box, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
	}

	cv::imshow("Connected:", tmp2);
#endif

}//FitEllipse

 //=======================================================================
bool Segmentor::ReOrganizeData(const std::vector<bool> & keepElement)
{
	int numElement = m_Data.size();
	int numEllipse = m_EllipseBox.size();
	
	if (keepElement.size() != numElement || numEllipse != numElement )
	{
		std::cout << "error, inconsistent size" << std::endl;
		return false;
	}

	int k = 0;
	for (int i = 0; i < numElement; i++)
	{
		if (!keepElement[i])
		{
			continue;
		}

		m_EllipseBox[k] = m_EllipseBox[i];
		m_Data[k++] = m_Data[i];
	}

	m_Data.resize(k);
	m_EllipseBox.resize(k);

	std::cout << "num elements: " << k << std::endl;

	return true;
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
		// iii. each circle may end up not in ONE connected component, so we need to MERGE connected components
		//
		// Modified algorithm:
		// a. Each connected component is a sample set. So if there are n connected components,
		// there are n sample set. 
		// b. for each sample set, use all of the sample to find the fitted model. In our case, an ellipse
		// c. go back to check the sample set, and remove the sample set if some criteria is not met
		// d. merge the connected components if needed
		// e. loop a - d for all sample set

		// 1. find connected components
		FindConnectedComp(output);

		// 2 + 3 + 4: loop through each component and fit it to ellipse, delete the noise.
		if (!FitEllipse(output))
		{
			return;
		}

		// overlay the ellipse on top of the image
		output = input;
		int numComponents = m_EllipseBox.size();

		for (int i = 0; i < numComponents; i++)
		{
			const cv::RotatedRect & box = m_EllipseBox[i];

			cv::ellipse(output, box, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
		}

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