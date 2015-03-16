/*
 * Class to interface with the webcam using OpenCV's webcam interface.
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
#include "Webcam.h"
#include "../Utils/SUtils.h"
#include "../Utils/MultiPlatformSleep.h"
using std::cout; using std::endl;


void WebcamCV::InitWindow()
{
	initCameraParams();
	
    if(img_width_received > 0 && img_height_received > 0)
    {} else {
        std::cout<<"ERROR: WEBCAM "<<to_istring(this_webcam_number)<<" UNABLE TO BE INITIALIZED"<<std::endl;
    }
}

void WebcamCV::UpdateWindowMainThread()
{
    if(img_width_received > 0 && img_height_received > 0)
    {
        the_webcam >> last_received_image;
        
        if(last_received_image.rows > 0 && last_received_image.cols > 0) {
        } else {
            std::cout<<"warning: empty image from webcam "<<to_istring(this_webcam_number)<<"!"<<std::endl;
        }
    }
}


cv::Mat WebcamCV::GetLastReceivedImageMainThread()
{
	cv::Mat retval;
	image_mutex.lock();
	last_received_image.copyTo(retval);
	image_mutex.unlock();
	return retval;
}

cv::Mat* WebcamCV::CheckForNewImageFromThread()
{
	cv::Mat* retval = nullptr;
	
	image_mutex.lock();
	if(haveGrabbedThisImage == false) {
		retval = new cv::Mat();
		last_received_image.copyTo(*retval);
		haveGrabbedThisImage = true;
	}
	image_mutex.unlock();
	
	return retval;
}

void WebcamCV::UpdateWindowCamerasThread()
{
	cv::Mat grabbedImage;
	while(true)
	{
		the_webcam >> grabbedImage;
		image_mutex.lock();
		haveGrabbedThisImage = false;
		grabbedImage.copyTo(last_received_image);
		image_mutex.unlock();
	}
}

void WebcamCV::StartAutoGrabImagesThread()
{
	if(grabbing_images_thread == nullptr) {
		grabbing_images_thread = new std::thread(&WebcamCV::UpdateWindowCamerasThread, this);
	} else {
		cout<<"ERROR: WEBCAM THREAD ALREADY STARTED"<<endl;
		assert(true);
	}
}


void WebcamCV::initCameraParams()
{
	img_width_received = the_webcam.get(CV_CAP_PROP_FRAME_WIDTH);
	img_height_received = the_webcam.get(CV_CAP_PROP_FRAME_HEIGHT);
	double readfps = the_webcam.get(CV_CAP_PROP_FPS);
	cout<<"##### fps of webcam according to itself: "<<readfps<<endl;
	cout<<"##### webcam image dimensions: "<<img_width_received<<"x"<<img_height_received<<endl;
	
#if 1
	the_webcam.set(CV_CAP_PROP_FRAME_WIDTH, 320.0);
	the_webcam.set(CV_CAP_PROP_FRAME_HEIGHT, 240.0);
	the_webcam.set(CV_CAP_PROP_FPS, 30.0);
	
	cout<<"####2 webcam image dimensions: "<<the_webcam.get(CV_CAP_PROP_FRAME_WIDTH)<<"x"<<the_webcam.get(CV_CAP_PROP_FRAME_HEIGHT)<<endl;
	
	//MultiPlatformSleep(2000);
	//the_webcam.set(CV_CAP_PROP_EXPOSURE, -10.0);
#else
	the_webcam.set(CV_CAP_PROP_FRAME_WIDTH, 640.0);
	the_webcam.set(CV_CAP_PROP_FRAME_HEIGHT, 480.0);
	the_webcam.set(CV_CAP_PROP_FPS, 15.0);
#endif
}


