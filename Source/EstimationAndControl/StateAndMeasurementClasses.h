#ifndef ___PENDCART_STATE_AND_MEASUREMENT_CLASSES_H____
#define ___PENDCART_STATE_AND_MEASUREMENT_CLASSES_H____

#include <opencv2/opencv.hpp>
#include "PendulumCart_Constants.h"
#include <iostream>

class CV_PendCart_Measurement
{
public:
	enum types
	{
		positions,
		velocities,
		UNKNOWN_TYPE
	};
	double timestamp;
	types type;
	cv::Mat data;
	bool I_was_simulated;
	
	//for ascending order (earliest times first, latest times last)
	static bool SortByTimestamp(const CV_PendCart_Measurement &lhs, const CV_PendCart_Measurement &rhs) {return (lhs.timestamp < rhs.timestamp);}
	
	CV_PendCart_Measurement() : timestamp(-1000000.0), type(UNKNOWN_TYPE), I_was_simulated(false) {}
	CV_PendCart_Measurement(const CV_PendCart_Measurement & other) : timestamp(other.timestamp),
																	 type(other.type),
																	 I_was_simulated(other.I_was_simulated)
																	 {other.data.copyTo(data);}
	
	friend std::ostream& operator<< (std::ostream &out, const CV_PendCart_Measurement &bmeas) {
		out << "(" << bmeas.data.at<double>(0,0) << ", " << bmeas.data.at<double>(1,0)<< ", timest:"<< bmeas.timestamp << ")";
		return out;
	}
};

class pcstate_class
{
public:
	double timestamp;
	cv::Mat xvec;
	cv::Mat Pmat;
	
	//for ascending order (earliest times first, latest times last)
	static bool SortByTimestamp(const pcstate_class &lhs, const pcstate_class &rhs) {return (lhs.timestamp < rhs.timestamp);}
	
	pcstate_class() : timestamp(-1000000.0) {}
	pcstate_class(const pcstate_class & other) : timestamp(other.timestamp) {other.xvec.copyTo(xvec); other.Pmat.copyTo(Pmat);}
};

class controlforce_class
{
public:
	double timestamp;
	double data;
	bool I_was_simulated;
	
	controlforce_class() : timestamp(-1000000.0), data(0.0), I_was_simulated(false) {}
	
	//for ascending order (earliest times first, latest times last)
	static bool SortByTimestamp(const controlforce_class &lhs, const controlforce_class &rhs) {return (lhs.timestamp < rhs.timestamp);}
};


#endif
