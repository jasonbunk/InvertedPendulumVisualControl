/*
 * Generic class for realtime webcam processing.
 *
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: Apache License Version 2.0, January 2004
 * Copyright (c) 2015 Jason Bunk
 */
#include "RealtimeVideoProcessor.h"

void RealtimeVideoProcessor::InitializeNow()
{
	DoInitialization();
}

void RealtimeVideoProcessor::UpdateAndCheckForMeasurement(double *& possible_returned_delaytime, CV_PendCart_Measurement *& possible_returned_measurement)
{
	possible_returned_delaytime = nullptr;
	possible_returned_measurement = nullptr;
	
	cv::Mat * possible_img = nullptr;
	CheckForCapturedImage(possible_returned_delaytime, possible_img);
	
	if(possible_returned_delaytime != nullptr && possible_img != nullptr)
	{
		imgProcessingTimer.restart();
		
		possible_returned_measurement = ProcessImageToMeasurements(possible_img);
		delete possible_img;
		
		(*possible_returned_delaytime) += imgProcessingTimer.getElapsedTime().asSeconds();
		
		//std::cout<<"video processing delaytime: "<<(*possible_returned_delaytime)<<std::endl;
	}
}




