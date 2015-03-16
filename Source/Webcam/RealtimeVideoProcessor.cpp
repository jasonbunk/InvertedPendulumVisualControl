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
	assert(threaded_image_processing == nullptr);
	threaded_image_processing = new std::thread(&RealtimeVideoProcessor::ThreadMainLoopForProcessingIncomingImagesToMeasurements, this);
}


void RealtimeVideoProcessor::ThreadMainLoopForProcessingIncomingImagesToMeasurements()
{
	cv::Mat * possible_img;
	double * possible_returned_delaytime;
	CV_PendCart_Raw_Measurement * possible_returned_measurement;
	
	while(true)
	{
		possible_img = nullptr;
		possible_returned_delaytime = nullptr;
		possible_returned_measurement = nullptr;
		
		CheckForCapturedImage(possible_returned_delaytime, possible_img);
		
		if(possible_returned_delaytime != nullptr && possible_img != nullptr)
		{
			double time_since_last_image_was_processed = imgProcessingTimer.getTimeSinceLastMeasurement();
			
			possible_returned_measurement = ProcessImageToMeasurements(possible_img);
			delete possible_img;
			
			if(possible_returned_measurement != nullptr) {
				possible_returned_measurement->estimated_delay = ((*possible_returned_delaytime) + imgProcessingTimer.getTimeSinceLastMeasurement());
				
				//cout<<"image processing time: "<<(possible_returned_measurement->estimated_delay - (*possible_returned_delaytime))<<endl;
				
				recent_measurements_mutex.lock();
				if(recent_measurements.empty() == false) {
					for(int ii=0; ii<recent_measurements.size(); ii++) {
						recent_measurements[ii]->estimated_delay += time_since_last_image_was_processed;
					}
				}
				recent_measurements.push_back(possible_returned_measurement);
				recent_measurements_mutex.unlock();
			}
		}
	}
}


std::vector<CV_PendCart_Raw_Measurement*>* RealtimeVideoProcessor::UpdateAndCheckForMeasurements()
{
	std::vector<CV_PendCart_Raw_Measurement*>* returned = nullptr;
	
	recent_measurements_mutex.lock();
	if(recent_measurements.empty()==false) {
		returned = new std::vector<CV_PendCart_Raw_Measurement*>();
		for(int ii=0; ii<recent_measurements.size(); ii++) {
			returned->push_back(recent_measurements[ii]); //return them in order (monotonically increasing times)
			
			assert(isnan(returned->back()->estimated_delay)==false);
			assert(isnan(returned->back()->cartx)==false);
			assert(isnan(returned->back()->theta)==false);
		}
		recent_measurements.clear();
	}
	recent_measurements_mutex.unlock();
	
	return returned;
}




