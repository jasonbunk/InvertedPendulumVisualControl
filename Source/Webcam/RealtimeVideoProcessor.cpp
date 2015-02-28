/*
 * Generic class for realtime webcam processing.
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




