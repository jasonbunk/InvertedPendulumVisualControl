/*
 * Use computer vision (OpenCV) to find the angle of the pendulum.
 *
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: Apache License Version 2.0, January 2004
 * Copyright (c) 2015 Jason Bunk
 */
#include "ColoredPendOrientationFinder.h"
using std::cout; using std::endl;
#include "../Utils/SUtils.h"
#include "../Utils/MathUtils.h"
#include "EstimationAndControl/PendCart_State_CVMat_defines.h"
#include "../Utils/MultiPlatformSleep.h"
#include "Webcam.h"


static void Erosion(cv::Mat & givenMat, int type, int ksize)
{
//  type = MORPH_RECT; 
//  type = MORPH_CROSS; 
//  type = MORPH_ELLIPSE; 

  cv::Mat element = cv::getStructuringElement(type,
                                       cv::Size(2*ksize + 1, 2*ksize+1),
                                       cv::Point(ksize, ksize));

  /// Apply the erosion operation
  cv::erode(givenMat, givenMat, element);
  //imshow( "Erosion Demo", erosion_dst );
}

static void Dilation(cv::Mat & givenMat, int type, int ksize)
{
//  type = MORPH_RECT; 
//  type = MORPH_CROSS; 
//  type = MORPH_ELLIPSE; 

  cv::Mat element = cv::getStructuringElement(type,
                                       cv::Size(2*ksize + 1, 2*ksize+1),
                                       cv::Point(ksize, ksize));
  /// Apply the dilation operation
  cv::dilate(givenMat, givenMat, element);
}


 
 
 

void ColoredPendOrientationFinder::CheckForCapturedImage(double *& possible_returned_delaytime, cv::Mat *& possible_returned_img)
{
	assert(mywebcam != nullptr);
	
	mywebcam->UpdateWindow(false);
	
	if(mywebcam->GetLastReceivedImage()->empty() == false)
	{
		possible_returned_delaytime = new double();
		(*possible_returned_delaytime) = 0.05;
		
		possible_returned_img = new cv::Mat();
		mywebcam->GetLastReceivedImage()->copyTo(*possible_returned_img);
		
		assert(possible_returned_img->empty() == false);
	}
}


//#define SHOWSTUF_WHILE 1


static CV_PendCart_Measurement* GetPendAngleFromSegmentedImg(cv::Mat * segImg, bool drawstuff, cv::Mat * returnedDrawnHere = nullptr)
{
	cv::Moments M = cv::moments(*segImg, true);
	
	double x = (M.m10 / M.m00);
	double y = (M.m01 / M.m00);
	
	double u20 = (M.m20/M.m00) - (x*x);
	double u02 = (M.m02/M.m00) - (y*y);
	double u11 = (M.m11/M.m00) - (x*y);
	
	double theta = 0.5*atan2(2.0*u11, u20 - u02);
	
	CV_PendCart_Measurement* retval = new CV_PendCart_Measurement();
	retval->type = CV_PendCart_Measurement::positions;
	retval->data = cv::Mat(2,1,CV_64F);
	retval->data.POSMEAS__theta = theta;
	
	if(drawstuff == false) {
		return retval;
	}
	
	std::vector<cv::Mat> chanels3(3);
	segImg->copyTo(chanels3[0]);
	segImg->copyTo(chanels3[1]);
	segImg->copyTo(chanels3[2]);
	cv::Mat mergedColor;
	cv::merge(chanels3, mergedColor);
	
	const double linelen = 200.0;
	cv::line(mergedColor, cv::Point(x,y), cv::Point(x+linelen*cos(theta), y+linelen*sin(theta)), CV_RGB(255,0,0), 2);
	
	cv::imshow("mergedColor", mergedColor);
	mergedColor.copyTo(*returnedDrawnHere);
	
	//cv::waitKey(0);
	
	return retval;
}


CV_PendCart_Measurement * ColoredPendOrientationFinder::ProcessImageToMeasurements(cv::Mat * givenMat)
{
	cv::Mat processingm;
	cv::medianBlur(*givenMat, processingm, 3);
	
	cv::cvtColor(processingm, processingm, CV_BGR2HLS);
	cv::Scalar colorsLow(HH_l, LL_l, SS_l);
	cv::Scalar colorsHigh(HH_h, LL_h, SS_h);
	cv::inRange(processingm, colorsLow, colorsHigh, processingm);

	return GetPendAngleFromSegmentedImg(&processingm, false);
}


void ColoredPendOrientationFinder::CalibrateColorFinder()
{
	cv::Mat latestImg, blurredImg, processedImg;
	
	std::string winNameStr("calibration");
	cv::namedWindow(winNameStr, cv::WINDOW_AUTOSIZE);
	HH_l=0; LL_l=0; SS_l=0; HH_h=180; LL_h=255; SS_h=255;
	int doneTrackbarCheck = 0;
	cv::createTrackbar("HH_l", winNameStr, &HH_l, 180);
	cv::createTrackbar("HH_h", winNameStr, &HH_l, 180);
	cv::createTrackbar("LL_l", winNameStr, &LL_l, 255);
	cv::createTrackbar("LL_h", winNameStr, &LL_h, 255);
	cv::createTrackbar("SS_l", winNameStr, &SS_l, 255);
	cv::createTrackbar("SS_h", winNameStr, &SS_h, 255);
	cv::createTrackbar("done", winNameStr, &doneTrackbarCheck, 10);
	cv::Mat resultRanged;
	cv::Mat resultDrawnLine;
	bool keypressed = false;
	while(keypressed == false)
	{
		HH_l = cv::getTrackbarPos("HH_l", winNameStr);
		HH_h = cv::getTrackbarPos("HH_h", winNameStr);
		LL_l = cv::getTrackbarPos("LL_l", winNameStr);
		LL_h = cv::getTrackbarPos("LL_h", winNameStr);
		SS_l = cv::getTrackbarPos("SS_l", winNameStr);
		SS_h = cv::getTrackbarPos("SS_h", winNameStr);
		doneTrackbarCheck = cv::getTrackbarPos("done", winNameStr);
		
		if(doneTrackbarCheck > 5) {keypressed = true;}
		
		mywebcam->UpdateWindow(false);
		mywebcam->GetLastReceivedImage()->copyTo(latestImg);
		
		if(keypressed == false && latestImg.empty() == false)
		{
			//cv::medianBlur(latestImg, blurredImg, 5);
			
			latestImg.copyTo(blurredImg);
			
			cv::cvtColor(blurredImg, processedImg, CV_BGR2HLS);
			
			cv::Scalar colorsLow(HH_l, LL_l, SS_l);
			cv::Scalar colorsHigh(HH_h, LL_h, SS_h);
			
			cv::inRange(processedImg, colorsLow, colorsHigh, processedImg);
			
			Erosion(processedImg, cv::MORPH_CROSS, 3);
			Dilation(processedImg, cv::MORPH_RECT, 3);
			
			cv::imshow(winNameStr, processedImg*255);
			cv::imshow("blurred", blurredImg);
			cv::waitKey(3);
			GetPendAngleFromSegmentedImg(&processedImg, true, &resultDrawnLine);
		}
		
		MultiPlatformSleep(10);
	}
	cv::destroyAllWindows();
}


void ColoredPendOrientationFinder::DoInitialization()
{
#if 0
	cv::Mat testimg = cv::imread("imgs/greenruler2_withjunk.jpg");
	//cv::Mat testimg = cv::imread("imgs/greenruler1.jpg");
	
#if 0
	cv::imshow("test", testimg);
	cv::waitKey(0);
	ProcessImageToMeasurements(&testimg);
	
#else
	//cv::Mat rsult;
	double numLoopsScalar = 1.0;
	
	sf::Clock testTimer;
	testTimer.restart();
	
	/*for(int ii=0; ii<100; ii++) {
		//cv::medianBlur(testimg, rsult, 5);
		//cv::GaussianBlur(testimg, rsult, cv::Size(11,11), 0.0, 0.0);//, cv::BORDER_REFLECT_101);
		//cv::medianBlur(rsult, rsult, 5);
	}*/
	CV_PendCart_Measurement * meas = ProcessImageToMeasurements(&testimg);
	
	double timetaken = testTimer.getElapsedTime().asSeconds();
	std::cout<<std::endl<<"time for EACH img process: "<<(numLoopsScalar*timetaken)<<" seconds"<<std::endl;
	
	std::cout<<"MEAS FOUND: theta: "<<(meas->data.POSMEAS__theta)<<std::endl;
	//cv::imshow("ttt",rsult);
	//cv::waitKey(0);
#endif
	
	exit(0);
#endif


	mywebcam = new WebcamCV(1);
	mywebcam->InitWindow(false);
	
	CalibrateColorFinder();
}





