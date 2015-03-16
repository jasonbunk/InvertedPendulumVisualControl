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
#include "../stdafx.h"
#include "ColoredPendOrientationFinder.h"
using std::cout; using std::endl;
#include "Utils/SUtils.h"
#include "Utils/MathUtils.h"
#include "EstimationAndControl/PendCart_State_CVMat_defines.h"
#include "Utils/MultiPlatformSleep.h"
#include "Webcam.h"
#include "TryIncludeJPhysics.h"
#include <fstream>



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


CV_PendCart_Raw_Measurement * ColoredPendOrientationFinder::ProcessImageToMeasurements(cv::Mat * givenMat)
{
	return myImageProcessor.ProcessImageToMeasurements(givenMat, false);
}


void ColoredPendOrientationFinder::CalibratePendulumFinder()
{
	cv::Mat latestImg, blurredImg, processedImg;
	
	std::string winNameStr("calibration");
	cv::namedWindow(winNameStr, cv::WINDOW_AUTOSIZE);
	myImageProcessor.HH_l=0;   myImageProcessor.LL_l=0;   myImageProcessor.SS_l=0;
	myImageProcessor.HH_h=180; myImageProcessor.LL_h=255; myImageProcessor.SS_h=255;
	int doneTrackbarCheck = 0;
	int saveThetaTrackbar = 0;
	int resetThetaTrackbar = 0;
	cv::createTrackbar("HH_l", winNameStr, &myImageProcessor.HH_l, 180);
	cv::createTrackbar("HH_h", winNameStr, &myImageProcessor.HH_l, 180);
	cv::createTrackbar("LL_l", winNameStr, &myImageProcessor.LL_l, 255);
	cv::createTrackbar("LL_h", winNameStr, &myImageProcessor.LL_h, 255);
	cv::createTrackbar("SS_l", winNameStr, &myImageProcessor.SS_l, 255);
	cv::createTrackbar("SS_h", winNameStr, &myImageProcessor.SS_h, 255);
	cv::createTrackbar("savetheta", winNameStr, &saveThetaTrackbar, 10);
	cv::createTrackbar("resettheta", winNameStr, &resetThetaTrackbar, 10);
	cv::createTrackbar("done", winNameStr, &doneTrackbarCheck, 10);
	
	bool keypressed = false;
	while(keypressed == false)
	{
		myImageProcessor.HH_l = cv::getTrackbarPos("HH_l", winNameStr);
		myImageProcessor.HH_h = cv::getTrackbarPos("HH_h", winNameStr);
		myImageProcessor.LL_l = cv::getTrackbarPos("LL_l", winNameStr);
		myImageProcessor.LL_h = cv::getTrackbarPos("LL_h", winNameStr);
		myImageProcessor.SS_l = cv::getTrackbarPos("SS_l", winNameStr);
		myImageProcessor.SS_h = cv::getTrackbarPos("SS_h", winNameStr);
		saveThetaTrackbar = cv::getTrackbarPos("savetheta", winNameStr);
		resetThetaTrackbar = cv::getTrackbarPos("resettheta", winNameStr);
		doneTrackbarCheck = cv::getTrackbarPos("done", winNameStr);
		
		if(resetThetaTrackbar > 5) {
			myImageProcessor.calibration_running_total_angle = 0.0;
			myImageProcessor.calibration_num_angles_saved_hanging = 0.0;
			cv::setTrackbarPos("resettheta", winNameStr, 0);
		}
		
		if(doneTrackbarCheck > 5) {keypressed = true;}
		
		mywebcam->UpdateWindowMainThread();
		latestImg = mywebcam->GetLastReceivedImageMainThread();
		
		if(keypressed == false && latestImg.empty() == false)
		{
			cv::Mat pendBinary, cartBinary;
			cv::medianBlur(latestImg, blurredImg, 3);
			myImageProcessor.GetBinaryImg_CartX(blurredImg, cartBinary);
			myImageProcessor.GetBinaryImg_PendAngle(blurredImg, pendBinary);
			
			double cartx, carty, pendtheta;
			myImageProcessor.GetCartLocationFromSegmentedImg(&cartBinary, false, cartx, carty);
			
			cv::imshow("blurred", blurredImg);
			cv::imshow("latest", latestImg);
			cv::imshow(winNameStr, pendBinary*255);
			cv::waitKey(3);
			myImageProcessor.GetPendAngleFromSegmentedImg(&cartBinary, &pendBinary, true, cartx, carty, pendtheta);
			
			if(saveThetaTrackbar > 5 && isnan(pendtheta)==false) {
				myImageProcessor.calibration_running_total_angle += pendtheta;
				myImageProcessor.calibration_num_angles_saved_hanging += 1.0;
				
				myImageProcessor.angle_when_vertical = (myImageProcessor.calibration_running_total_angle / myImageProcessor.calibration_num_angles_saved_hanging);
				cout<<"angle when inverted (vertical): "<<myImageProcessor.angle_when_vertical<<endl;
			}
			
			cout<<"measurements: cartx: "<<myImageProcessor.ConvertCartXFromPixelCoords(cartx)<<", theta: "<<myImageProcessor.ConvertThetaFromPixelCoords(pendtheta)<<endl;
		}
		
		MultiPlatformSleep(10);
	}
	cv::destroyAllWindows();
	
	myImageProcessor.angle_when_vertical = (myImageProcessor.calibration_running_total_angle / myImageProcessor.calibration_num_angles_saved_hanging);
}


void ColoredPendOrientationFinder::CalibrateCartFinder()
{
	cv::Mat latestImg, blurredImg, cartBinary;
	
	std::string winNameStr("calibration");
	cv::namedWindow(winNameStr, cv::WINDOW_AUTOSIZE);
	myImageProcessor.HHcar_l=0; myImageProcessor.LLcar_l=0; myImageProcessor.SScar_l=0;
	myImageProcessor.HHcar_h=180; myImageProcessor.LLcar_h=255; myImageProcessor.SScar_h=255;
	int doneTrackbarCheck = 0;
	cv::createTrackbar("HHcar_l", winNameStr, &myImageProcessor.HHcar_l, 180);
	cv::createTrackbar("HHcar_h", winNameStr, &myImageProcessor.HHcar_l, 180);
	cv::createTrackbar("LLcar_l", winNameStr, &myImageProcessor.LLcar_l, 255);
	cv::createTrackbar("LLcar_h", winNameStr, &myImageProcessor.LLcar_h, 255);
	cv::createTrackbar("SScar_l", winNameStr, &myImageProcessor.SScar_l, 255);
	cv::createTrackbar("SScar_h", winNameStr, &myImageProcessor.SScar_h, 255);
	cv::createTrackbar("trackmin", winNameStr, &myImageProcessor.cartx_track_pix_min, 255);
	cv::createTrackbar("trackmax", winNameStr, &myImageProcessor.cartx_track_pix_max, 255);
	cv::createTrackbar("done", winNameStr, &doneTrackbarCheck, 10);
	
	bool keypressed = false;
	while(keypressed == false)
	{
		myImageProcessor.HHcar_l = cv::getTrackbarPos("HHcar_l", winNameStr);
		myImageProcessor.HHcar_h = cv::getTrackbarPos("HHcar_h", winNameStr);
		myImageProcessor.LLcar_l = cv::getTrackbarPos("LLcar_l", winNameStr);
		myImageProcessor.LLcar_h = cv::getTrackbarPos("LLcar_h", winNameStr);
		myImageProcessor.SScar_l = cv::getTrackbarPos("SScar_l", winNameStr);
		myImageProcessor.SScar_h = cv::getTrackbarPos("SScar_h", winNameStr);
		myImageProcessor.cartx_track_pix_min = cv::getTrackbarPos("trackmin", winNameStr);
		myImageProcessor.cartx_track_pix_max = cv::getTrackbarPos("trackmax", winNameStr);
		doneTrackbarCheck = cv::getTrackbarPos("done", winNameStr);
		
		if(doneTrackbarCheck > 5) {keypressed = true;}
		
		mywebcam->UpdateWindowMainThread();
		latestImg = mywebcam->GetLastReceivedImageMainThread();
		
		if(keypressed == false && latestImg.empty() == false)
		{
			cv::medianBlur(latestImg, blurredImg, 3);
			myImageProcessor.GetBinaryImg_CartX(blurredImg, cartBinary);
			
			double cartx, carty;
			myImageProcessor.GetCartLocationFromSegmentedImg(&cartBinary, true, cartx, carty, nullptr, myImageProcessor.cartx_track_pix_min, myImageProcessor.cartx_track_pix_max);
			
			myImageProcessor.DrawCartTracks(blurredImg, cartx, carty);
			//cv::imshow("latest", latestImg);
			cv::imshow("blurred", blurredImg);
			cv::imshow(winNameStr, cartBinary*255);
			cv::waitKey(3);
		}
		
		MultiPlatformSleep(10);
	}
	cv::destroyAllWindows();
	
	myImageProcessor.calculated_cart_track_width_in_pixels = (((double)myImageProcessor.cartx_track_pix_max) - ((double)myImageProcessor.cartx_track_pix_min));
}


void ColoredPendOrientationFinder::DoInitialization()
{
	if(mywebcam == nullptr) {
		mywebcam = new WebcamCV(1);
		mywebcam->InitWindow();
	}
	
	if(calibrated == false) {LoadOldCalibration();}
	
	//mask of whole image
	myImageProcessor.lastBinaryCart_Dilated = cv::Mat::ones(mywebcam->GetLastReceivedImageMainThread().size(), CV_8U);
	myImageProcessor.lastBinaryCart_Dilated.copyTo(myImageProcessor.lastBinaryPend_Dilated);
	
	mywebcam->StartAutoGrabImagesThread();
}


void ColoredPendOrientationFinder::LoadOldCalibration() {
	if(mywebcam == nullptr) {
		mywebcam = new WebcamCV(1);
		mywebcam->InitWindow();
	}
	myImageProcessor.LoadCalibrationFromFile("last_webcam_calibration.txt");
	myImageProcessor.PrintCalibration();
	
	double time_elapsed = 0.0;
	myclock warmupTimer;
	while(time_elapsed < 4.0)
	{
		warmupTimer.restart();
		mywebcam->UpdateWindowMainThread();
		cv::Mat latestImg2 = mywebcam->GetLastReceivedImageMainThread();
		
		cv::Mat blurredImg, pendBinary, cartBinary;
		cv::medianBlur(latestImg2, blurredImg, 3);
		myImageProcessor.GetBinaryImg_CartX(blurredImg, cartBinary);
		myImageProcessor.GetBinaryImg_PendAngle(blurredImg, pendBinary);
		
		double cartx, carty, pendtheta;
		myImageProcessor.GetCartLocationFromSegmentedImg(&cartBinary, false, cartx, carty);
		
		if(RenderVisualsUsingSFML) {
			cv::imshow("warming up...", blurredImg);
			myImageProcessor.GetPendAngleFromSegmentedImg(&cartBinary, &pendBinary, true, cartx, carty, pendtheta);
			cv::waitKey(3);
		}
		MultiPlatformSleep(10);
		
		time_elapsed += warmupTimer.getTimeSinceLastMeasurement();
	}
	cv::destroyAllWindows();
	calibrated = true;
}

void ColoredPendOrientationFinder::DoNewCalibration() {
	if(mywebcam == nullptr) {
		mywebcam = new WebcamCV(1);
		mywebcam->InitWindow();
	}
	myImageProcessor.angle_when_vertical = 0.0;
	myImageProcessor.calibration_running_total_angle = 0.0;
	myImageProcessor.calibration_num_angles_saved_hanging = 0.0;
	CalibrateCartFinder();
	CalibratePendulumFinder();
	
	myImageProcessor.SaveCalibrationToFile("last_webcam_calibration.txt");
	
	myImageProcessor.PrintCalibration();
	calibrated = true;
}















