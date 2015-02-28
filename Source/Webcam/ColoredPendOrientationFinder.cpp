/*
 * Use computer vision (OpenCV) to find the angle of the pendulum.
 *
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * 
 * Copyright (c) 2015 Jason Bunk
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/
#include "ColoredPendOrientationFinder.h"
using std::cout; using std::endl;
#include "../Utils/SUtils.h"
#include "../Utils/MathUtils.h"
#include "EstimationAndControl/PendCart_State_CVMat_defines.h"
#include "../Utils/MultiPlatformSleep.h"
#include "Webcam.h"
#include "TryIncludeJPhysics.h"
#include "PrinterPhys124_PendCart_Params.h"
#include <fstream>


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
	
	possible_returned_img = mywebcam->CheckForNewImageFromThread();
	
	if(possible_returned_img != nullptr)
	{
		possible_returned_delaytime = new double();
		(*possible_returned_delaytime) = 0.05; //fixed estimated webcam delay
	}
}


//#define SHOWSTUF_WHILE 1


static double GetPendAngleFromSegmentedImg(cv::Mat * segImgForCart, cv::Mat * segImgForPend,
											bool drawstuff,
											double cart_x, double cart_y, double & returned_theta,
											cv::Mat * returnedDrawnHere = nullptr)
{
	cv::bitwise_or(*segImgForCart, *segImgForPend, *segImgForPend);
	cv::Moments M = cv::moments(*segImgForPend, true);
	
	double x = (M.m10 / M.m00);
	double y = (M.m01 / M.m00);
	
	double u20 = (M.m20/M.m00) - (x*x);
	double u02 = (M.m02/M.m00) - (y*y);
	double u11 = (M.m11/M.m00) - (x*y);
	
	double theta_est_1 = 0.5*atan2(2.0*u11, u20 - u02); //pendulum pixels alone
	double theta_est_2 = atan2(cart_y-y, cart_x-x); //drawing line from pendulum to cart
	
	double diff_between_theta_ests = fabs(physmath::differenceBetweenAnglesSigned(theta_est_1, theta_est_2));
	if(diff_between_theta_ests > physmath::ONE_HALF_PI) {
		theta_est_1 += physmath::PI;
	}
	
	returned_theta = theta_est_1;//atan2(0.75*(2.0*u11) + (cart_y-y), 0.75*(u20 - u02) + (cart_x-x));
	
	if(drawstuff) {
		std::vector<cv::Mat> chanels3(3);
		segImgForPend->copyTo(chanels3[0]);
		segImgForPend->copyTo(chanels3[1]);
		segImgForPend->copyTo(chanels3[2]);
		cv::Mat mergedColor;
		cv::merge(chanels3, mergedColor);
		
		const double linelen = 200.0;
		cv::line(mergedColor, cv::Point(x,y), cv::Point(x+linelen*cos(returned_theta), y+linelen*sin(returned_theta)), CV_RGB(255,0,0), 2);
		
		cv::line(mergedColor, cv::Point(cart_x,cart_y-2), cv::Point(cart_x,cart_y+2), CV_RGB(0,255,0), 2);
		
		cv::imshow("processed", mergedColor);
		if(returnedDrawnHere != nullptr) {
			mergedColor.copyTo(*returnedDrawnHere);
		}
	}
}


static void GetCartLocationFromSegmentedImg(cv::Mat * segImgForCart,
											bool drawstuff,
											double & cart_x, double & cart_y,
											cv::Mat * returnedDrawnHere = nullptr, int cartx_track_pix_min=0, int cartx_track_pix_max=0)
{
	cv::Moments M = cv::moments(*segImgForCart, true);
	
	cart_x = (M.m10 / M.m00);
	cart_y = (M.m01 / M.m00);
	
	if(drawstuff) {
		std::vector<cv::Mat> chanels3(3);
		segImgForCart->copyTo(chanels3[0]);
		segImgForCart->copyTo(chanels3[1]);
		segImgForCart->copyTo(chanels3[2]);
		cv::Mat mergedColor;
		cv::merge(chanels3, mergedColor);
		
		const double linelen = 200.0;
		cv::line(mergedColor, cv::Point(cart_x,cart_y-5), cv::Point(cart_x,cart_y+5), CV_RGB(0,255,0), 2);
		
		cv::imshow("processed", mergedColor);
		if(returnedDrawnHere != nullptr) {
			mergedColor.copyTo(*returnedDrawnHere);
		}
	}
}


void ColoredPendOrientationFinder::DrawCartTracks(cv::Mat & onImg, double cart_x, double cart_y)
{
	cv::line(onImg, cv::Point(cartx_track_pix_min,cart_y-5), cv::Point(cartx_track_pix_min,cart_y+5), CV_RGB(0,0,255), 2);
	cv::line(onImg, cv::Point(cartx_track_pix_max,cart_y-5), cv::Point(cartx_track_pix_max,cart_y+5), CV_RGB(0,0,255), 2);
	
	cv::line(onImg, cv::Point(cart_x,cart_y-5), cv::Point(cart_x,cart_y+5), CV_RGB(0,255,0), 2);
}


void ColoredPendOrientationFinder::GetBinaryImg_PendAngle(cv::Mat & colorImg, cv::Mat & returnedBinary)
{
	cv::cvtColor(colorImg, returnedBinary, CV_BGR2HLS);
	
	cv::Scalar colorsLow(HH_l, LL_l, SS_l);
	cv::Scalar colorsHigh(HH_h, LL_h, SS_h);
	
	cv::inRange(returnedBinary, colorsLow, colorsHigh, returnedBinary);
	
	Erosion(returnedBinary, cv::MORPH_CROSS, 3);
	Dilation(returnedBinary, cv::MORPH_RECT, 3);
}

void ColoredPendOrientationFinder::GetBinaryImg_CartX(cv::Mat & colorImg, cv::Mat & returnedBinary)
{
	cv::cvtColor(colorImg, returnedBinary, CV_BGR2HLS);

	cv::Scalar colorsLow(HHcar_l, LLcar_l, SScar_l);
	cv::Scalar colorsHigh(HHcar_h, LLcar_h, SScar_h);
	
	cv::inRange(returnedBinary, colorsLow, colorsHigh, returnedBinary);
	
	Erosion(returnedBinary, cv::MORPH_CROSS, 3);
	Dilation(returnedBinary, cv::MORPH_RECT, 3);
}


CV_PendCart_Measurement * ColoredPendOrientationFinder::ProcessImageToMeasurements(cv::Mat * givenMat)
{
	cv::Mat processingm;
	cv::medianBlur(*givenMat, processingm, 3);
	
	cv::Mat pendBinary, cartBinary;
	GetBinaryImg_CartX(processingm, cartBinary);
	GetBinaryImg_PendAngle(processingm, pendBinary);
	
#if 1
	cv::bitwise_and(lastBinaryCart, cartBinary, cartBinary);
	cv::bitwise_and(lastBinaryPend, pendBinary, pendBinary);
	
	int nonzeroPixelsInCartBinary = cv::countNonZero(cartBinary);
	int nonzeroPixelsInPendBinary = cv::countNonZero(pendBinary);
	
	if(nonzeroPixelsInCartBinary < 2) {
		cartBinary = lastBinaryCart;
	} else {
		lastBinaryCart = cartBinary;
		Dilation(lastBinaryCart, cv::MORPH_RECT, 5);
	}
	if(nonzeroPixelsInPendBinary < 2) {
		pendBinary = lastBinaryPend;
	} else {
		lastBinaryPend = pendBinary;
		Dilation(lastBinaryPend, cv::MORPH_RECT, 5);
	}
#endif
	
	double cartx, carty, pendtheta;
	GetCartLocationFromSegmentedImg(&cartBinary, false, cartx, carty);
	GetPendAngleFromSegmentedImg(&cartBinary, &pendBinary, false, cartx, carty, pendtheta);
	
	if(isnan(cartx) == false && isnan(pendtheta) == false) {
		CV_PendCart_Measurement* retval = new CV_PendCart_Measurement();
		retval->type = CV_PendCart_Measurement::positions;
		retval->data = cv::Mat(2,1,CV_64F);
		retval->data.POSMEAS__cartx = ConvertCartXFromPixelCoords(cartx);
		retval->data.POSMEAS__theta = ConvertThetaFromPixelCoords(pendtheta);
		
		assert(isnan(retval->data.POSMEAS__cartx) == false);
		assert(isnan(retval->data.POSMEAS__theta) == false);
		
		return retval;
	}
	return nullptr;
}


void ColoredPendOrientationFinder::CalibratePendulumFinder()
{
	cv::Mat latestImg, blurredImg, processedImg;
	
	std::string winNameStr("calibration");
	cv::namedWindow(winNameStr, cv::WINDOW_AUTOSIZE);
	HH_l=0; LL_l=0; SS_l=0; HH_h=180; LL_h=255; SS_h=255;
	int doneTrackbarCheck = 0;
	int saveThetaTrackbar = 0;
	int resetThetaTrackbar = 0;
	cv::createTrackbar("HH_l", winNameStr, &HH_l, 180);
	cv::createTrackbar("HH_h", winNameStr, &HH_l, 180);
	cv::createTrackbar("LL_l", winNameStr, &LL_l, 255);
	cv::createTrackbar("LL_h", winNameStr, &LL_h, 255);
	cv::createTrackbar("SS_l", winNameStr, &SS_l, 255);
	cv::createTrackbar("SS_h", winNameStr, &SS_h, 255);
	cv::createTrackbar("savetheta", winNameStr, &saveThetaTrackbar, 10);
	cv::createTrackbar("resettheta", winNameStr, &resetThetaTrackbar, 10);
	cv::createTrackbar("done", winNameStr, &doneTrackbarCheck, 10);
	
	bool keypressed = false;
	while(keypressed == false)
	{
		HH_l = cv::getTrackbarPos("HH_l", winNameStr);
		HH_h = cv::getTrackbarPos("HH_h", winNameStr);
		LL_l = cv::getTrackbarPos("LL_l", winNameStr);
		LL_h = cv::getTrackbarPos("LL_h", winNameStr);
		SS_l = cv::getTrackbarPos("SS_l", winNameStr);
		SS_h = cv::getTrackbarPos("SS_h", winNameStr);
		saveThetaTrackbar = cv::getTrackbarPos("savetheta", winNameStr);
		resetThetaTrackbar = cv::getTrackbarPos("resettheta", winNameStr);
		doneTrackbarCheck = cv::getTrackbarPos("done", winNameStr);
		
		if(resetThetaTrackbar > 5) {
			calibration_running_total_angle = 0.0;
			calibration_num_angles_saved_hanging = 0.0;
			cv::setTrackbarPos("resettheta", winNameStr, 0);
		}
		
		if(doneTrackbarCheck > 5) {keypressed = true;}
		
		mywebcam->UpdateWindowMainThread();
		latestImg = mywebcam->GetLastReceivedImageMainThread();
		
		if(keypressed == false && latestImg.empty() == false)
		{
			cv::Mat pendBinary, cartBinary;
			cv::medianBlur(latestImg, blurredImg, 3);
			GetBinaryImg_CartX(blurredImg, cartBinary);
			GetBinaryImg_PendAngle(blurredImg, pendBinary);
			
			double cartx, carty, pendtheta;
			GetCartLocationFromSegmentedImg(&cartBinary, false, cartx, carty);
			
			cv::imshow("blurred", blurredImg);
			cv::imshow("latest", latestImg);
			cv::imshow(winNameStr, pendBinary*255);
			cv::waitKey(3);
			GetPendAngleFromSegmentedImg(&cartBinary, &pendBinary, true, cartx, carty, pendtheta);
			
			if(saveThetaTrackbar > 5 && isnan(pendtheta)==false) {
				calibration_running_total_angle += pendtheta;
				calibration_num_angles_saved_hanging += 1.0;
				
				angle_when_vertical = (calibration_running_total_angle / calibration_num_angles_saved_hanging);
				cout<<"angle when inverted (vertical): "<<angle_when_vertical<<endl;
			}
			
			cout<<"measurements: cartx: "<<ConvertCartXFromPixelCoords(cartx)<<", theta: "<<ConvertThetaFromPixelCoords(pendtheta)<<endl;
		}
		
		MultiPlatformSleep(10);
	}
	cv::destroyAllWindows();
	
	angle_when_vertical = (calibration_running_total_angle / calibration_num_angles_saved_hanging);
}


void ColoredPendOrientationFinder::CalibrateCartFinder()
{
	cv::Mat latestImg, blurredImg, cartBinary;
	
	std::string winNameStr("calibration");
	cv::namedWindow(winNameStr, cv::WINDOW_AUTOSIZE);
	HHcar_l=0; LLcar_l=0; SScar_l=0; HHcar_h=180; LLcar_h=255; SScar_h=255;
	int doneTrackbarCheck = 0;
	cv::createTrackbar("HHcar_l", winNameStr, &HHcar_l, 180);
	cv::createTrackbar("HHcar_h", winNameStr, &HHcar_l, 180);
	cv::createTrackbar("LLcar_l", winNameStr, &LLcar_l, 255);
	cv::createTrackbar("LLcar_h", winNameStr, &LLcar_h, 255);
	cv::createTrackbar("SScar_l", winNameStr, &SScar_l, 255);
	cv::createTrackbar("SScar_h", winNameStr, &SScar_h, 255);
	cv::createTrackbar("trackmin", winNameStr, &cartx_track_pix_min, 255);
	cv::createTrackbar("trackmax", winNameStr, &cartx_track_pix_max, 255);
	cv::createTrackbar("done", winNameStr, &doneTrackbarCheck, 10);
	
	bool keypressed = false;
	while(keypressed == false)
	{
		HHcar_l = cv::getTrackbarPos("HHcar_l", winNameStr);
		HHcar_h = cv::getTrackbarPos("HHcar_h", winNameStr);
		LLcar_l = cv::getTrackbarPos("LLcar_l", winNameStr);
		LLcar_h = cv::getTrackbarPos("LLcar_h", winNameStr);
		SScar_l = cv::getTrackbarPos("SScar_l", winNameStr);
		SScar_h = cv::getTrackbarPos("SScar_h", winNameStr);
		cartx_track_pix_min = cv::getTrackbarPos("trackmin", winNameStr);
		cartx_track_pix_max = cv::getTrackbarPos("trackmax", winNameStr);
		doneTrackbarCheck = cv::getTrackbarPos("done", winNameStr);
		
		if(doneTrackbarCheck > 5) {keypressed = true;}
		
		mywebcam->UpdateWindowMainThread();
		latestImg = mywebcam->GetLastReceivedImageMainThread();
		
		if(keypressed == false && latestImg.empty() == false)
		{
			cv::medianBlur(latestImg, blurredImg, 3);
			GetBinaryImg_CartX(blurredImg, cartBinary);
			
			double cartx, carty;
			GetCartLocationFromSegmentedImg(&cartBinary, true, cartx, carty, nullptr, cartx_track_pix_min, cartx_track_pix_max);
			
			DrawCartTracks(blurredImg, cartx, carty);
			//cv::imshow("latest", latestImg);
			cv::imshow("blurred", blurredImg);
			cv::imshow(winNameStr, cartBinary*255);
			cv::waitKey(3);
		}
		
		MultiPlatformSleep(10);
	}
	cv::destroyAllWindows();
	
	calculated_cart_track_width_in_pixels = (((double)cartx_track_pix_max) - ((double)cartx_track_pix_min));
}


template <typename T> T CLAMP(const T& value, const T& low, const T& high)  {
  return value < low ? low : (value > high ? high : value); 
}

double ColoredPendOrientationFinder::ConvertCartXFromPixelCoords(double raw_pixel_x)
{
	double unclamped = (raw_pixel_x - static_cast<double>(cartx_track_pix_min)) * (2.0 * PRINTER_LINEAR_WIDTH_X / calculated_cart_track_width_in_pixels);
	return CLAMP(unclamped-PRINTER_LINEAR_WIDTH_X, -1.0*PRINTER_LINEAR_WIDTH_X, PRINTER_LINEAR_WIDTH_X);
}

double ColoredPendOrientationFinder::ConvertThetaFromPixelCoords(double raw_theta)
{
	return physmath::differenceBetweenAnglesSigned(raw_theta, angle_when_vertical);
}


void ColoredPendOrientationFinder::DoInitialization()
{
	mywebcam = new WebcamCV(1);
	mywebcam->InitWindow();
	
#if 0
	angle_when_vertical = 0.0;
	calibration_running_total_angle = 0.0;
	calibration_num_angles_saved_hanging = 0.0;
	CalibrateCartFinder();
	CalibratePendulumFinder();
	
	std::ofstream savedCalibration("last_webcam_calibration.txt");
	assert(savedCalibration.is_open() && savedCalibration.good());
	savedCalibration << angle_when_vertical << endl;
	savedCalibration << cartx_track_pix_min << endl;
	savedCalibration << cartx_track_pix_max << endl;
	savedCalibration << HH_l << endl;
	savedCalibration << LL_l << endl;
	savedCalibration << SS_l << endl;
	savedCalibration << HH_h << endl;
	savedCalibration << LL_h << endl;
	savedCalibration << SS_h << endl;
	savedCalibration << HHcar_l << endl;
	savedCalibration << LLcar_l << endl;
	savedCalibration << SScar_l << endl;
	savedCalibration << HHcar_h << endl;
	savedCalibration << LLcar_h << endl;
	savedCalibration << SScar_h << endl;
	savedCalibration.close();
	PrintCalibration();
#else
	std::ifstream savedCalibration("last_webcam_calibration.txt");
	assert(savedCalibration.is_open() && savedCalibration.good());
	savedCalibration >> angle_when_vertical;
	savedCalibration >> cartx_track_pix_min;
	savedCalibration >> cartx_track_pix_max;
	savedCalibration >> HH_l;
	savedCalibration >> LL_l;
	savedCalibration >> SS_l;
	savedCalibration >> HH_h;
	savedCalibration >> LL_h;
	savedCalibration >> SS_h;
	savedCalibration >> HHcar_l;
	savedCalibration >> LLcar_l;
	savedCalibration >> SScar_l;
	savedCalibration >> HHcar_h;
	savedCalibration >> LLcar_h;
	savedCalibration >> SScar_h;
	savedCalibration.close();
	PrintCalibration();
	calculated_cart_track_width_in_pixels = (((double)cartx_track_pix_max) - ((double)cartx_track_pix_min));

	double time_elapsed = 0.0;
	sf::Clock warmupTimer;
	while(time_elapsed < 4.0)
	{
		warmupTimer.restart();
		mywebcam->UpdateWindowMainThread();
		cv::Mat latestImg2 = mywebcam->GetLastReceivedImageMainThread();
		
		cv::Mat blurredImg, pendBinary, cartBinary;
		cv::medianBlur(latestImg2, blurredImg, 3);
		GetBinaryImg_CartX(blurredImg, cartBinary);
		GetBinaryImg_PendAngle(blurredImg, pendBinary);
		
		double cartx, carty, pendtheta;
		GetCartLocationFromSegmentedImg(&cartBinary, false, cartx, carty);
		
		cv::imshow("warming up...", blurredImg);
		GetPendAngleFromSegmentedImg(&cartBinary, &pendBinary, true, cartx, carty, pendtheta);
		cv::waitKey(3);
		MultiPlatformSleep(10);
		
		time_elapsed += warmupTimer.getElapsedTime().asSeconds();
	}
	cv::destroyAllWindows();
#endif

	//mask of whole image
	lastBinaryCart = cv::Mat::ones(mywebcam->GetLastReceivedImageMainThread().size(), CV_8U)*255;
	lastBinaryCart.copyTo(lastBinaryPend);
	
	mywebcam->StartAutoGrabImagesThread();
}






void ColoredPendOrientationFinder::PrintCalibration()
{
	cout << "angle_when_vertical == " << angle_when_vertical << endl;
	cout << "cartx_track_pix_min == " << cartx_track_pix_min << endl;
	cout << "cartx_track_pix_max == " << cartx_track_pix_max << endl;
	cout << "HH_l == " << HH_l << endl;
	cout << "LL_l == " << LL_l << endl;
	cout << "SS_l == " << SS_l << endl;
	cout << "HH_h == " << HH_h << endl;
	cout << "LL_h == " << LL_h << endl;
	cout << "SS_h == " << SS_h << endl;
	cout << "HHcar_l == " << HHcar_l << endl;
	cout << "LLcar_l == " << LLcar_l << endl;
	cout << "SScar_l == " << SScar_l << endl;
	cout << "HHcar_h == " << HHcar_h << endl;
	cout << "LLcar_h == " << LLcar_h << endl;
	cout << "SScar_h == " << SScar_h << endl;
}















