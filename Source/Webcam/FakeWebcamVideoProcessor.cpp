/*
 * Read from a video file, simulating a webcam, possibly in "real time".
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
#include "FakeWebcamVideoProcessor.h"
using std::cout; using std::endl;
#include "../Utils/SUtils.h"
#include "../Utils/MathUtils.h"
#include "EstimationAndControl/PendCart_State_CVMat_defines.h"
#include "../Utils/MultiPlatformSleep.h"
#include "Webcam.h"
#include "TryIncludeJPhysics.h"
#include "PrinterPhys124_PendCart_Params.h"
#include <fstream>

static const double TIME_SCALAR_FOR_VIDEO_READER = (1.0 / 1.0);


void FakeWebcamVideoProcessor::CheckForCapturedImage(double *& possible_returned_delaytime, cv::Mat *& possible_returned_img)
{
	MultiPlatformSleep(1); //since this is called continuously by the RealtimeVideoProcessor's thread
	
	const double fixed_time_delay = (1.0 / 30.0);
	
	double frametime = (videoSimRealtimeTimer.getTimeSinceLastMeasurement())*TIME_SCALAR_FOR_VIDEO_READER;
	timer_for_next_frame += frametime;
	realtimeElapsed += frametime;
	//std::cout<<"video time elapsed: "<<realtimeElapsed<<std::endl;
	
	if(timer_for_next_frame >= fixed_time_delay)
	{
		timer_for_next_frame = 0.0;
		
		possible_returned_img = new cv::Mat();
		(*myVideoFileReader) >> (*possible_returned_img);
		if(possible_returned_img->empty()) {
			delete possible_returned_img;
			possible_returned_img = nullptr;
			cout<<"warning: empty image frame in video file"<<endl;
			return;
		}
		possible_returned_delaytime = new double(0.012);
		
		
		cv::imshow("latestvideoframe", *possible_returned_img);
		cv::waitKey(1);
		/*cout<<"image frame size: "<<(possible_returned_img->size())<<endl;
		cv::waitKey(0);
		cv::destroyAllWindows();*/
	}
	else
	{
		possible_returned_delaytime = nullptr;
		possible_returned_img = nullptr;
	}
}


CV_PendCart_Raw_Measurement * FakeWebcamVideoProcessor::ProcessImageToMeasurements(cv::Mat * givenMat)
{
	return myImageProcessor.ProcessImageToMeasurements(givenMat, true);
}


void FakeWebcamVideoProcessor::DoInitialization()
{
	const std::string videoFilename("../test2/2015-02-28-182543.webm");
	
	const std::string calibrationFilename("../test2/last_webcam_calibration.txt");
	
	myVideoFileReader = new cv::VideoCapture(videoFilename);
	if(myVideoFileReader->isOpened() == false) {
		cout<<"Error: could not open video file \""<<videoFilename<<"\""<<endl;
		exit(0);
	}
	myImageProcessor.LoadCalibrationFromFile(calibrationFilename);
	
	cv::namedWindow("latestvideoframe", cv::WINDOW_AUTOSIZE);
	
    realtimeElapsed = 0.0;
    videoSimRealtimeTimer.restart();
}






