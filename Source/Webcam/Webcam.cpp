/*
 * Class to interface with the webcam using OpenCV's webcam interface.
 *
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: Apache License Version 2.0, January 2004
 * Copyright (c) 2015 Jason Bunk
 */
#include "Webcam.h"
#include "../Utils/SUtils.h"
#include "../Utils/MultiPlatformSleep.h"

void WebcamCV::InitWindow(bool showstuff)
{
    winNameStr = std::string("camera ")+to_istring(this_webcam_number);
	getCameraVals(showstuff);
	
    if(img_width_received > 0 && img_height_received > 0)
    {
		if(showstuff)
		{
			cv::namedWindow(winNameStr, cv::WINDOW_AUTOSIZE);
			std::cout<<"webcam "<<this_webcam_number<<" img size: ("<<img_width_received<<", "<<img_height_received<<")"<<std::endl;
			/*cv::createTrackbar("Brightness", winNameStr, &brightness_slider, 255);
			cv::createTrackbar("Contrast", winNameStr, &contrast_slider, 255);
			cv::createTrackbar("Gain", winNameStr, &gain_slider, 255);
			cv::createTrackbar("Saturation", winNameStr, &saturation_slider, 255);*/
			cv::createTrackbar("Exposure+20", winNameStr, &exposure_slider, 40);
		}
    } else {
        std::cout<<"ERROR: WEBCAM "<<to_istring(this_webcam_number)<<" UNABLE TO BE INITIALIZED"<<std::endl;
    }
}

void WebcamCV::UpdateWindow(bool showstuff)
{
    if(img_width_received > 0 && img_height_received > 0)
    {
		//uncommenting this will make things much faster
        setCameraVals();

        the_webcam >> last_received_image;
        
        if(last_received_image.rows > 0 && last_received_image.cols > 0) {
			if(showstuff) {
				cv::imshow(winNameStr, last_received_image);
			}
        } else {
            std::cout<<"warning: empty image from webcam "<<to_istring(this_webcam_number)<<"!"<<std::endl;
        }
    }
}

void WebcamCV::setCameraVals()
{
	return;
	/*the_webcam.set(CV_CAP_PROP_BRIGHTNESS, brightness_slider);
	the_webcam.set(CV_CAP_PROP_CONTRAST, contrast_slider);
	the_webcam.set(CV_CAP_PROP_GAIN, gain_slider);
	the_webcam.set(CV_CAP_PROP_SATURATION, saturation_slider);*/
	std::cout<<"setting webcam "<<this_webcam_number<<" exposure to: "<<(exposure_slider-20)<<std::endl;
	the_webcam.set(CV_CAP_PROP_EXPOSURE, (exposure_slider-20));
}

void WebcamCV::getCameraVals(bool showingstuff)
{
	img_width_received = the_webcam.get(CV_CAP_PROP_FRAME_WIDTH);
	img_height_received = the_webcam.get(CV_CAP_PROP_FRAME_HEIGHT);
	double readfps = the_webcam.get(CV_CAP_PROP_FPS);
	std::cout<<"##### fps of webcam according to itself: "<<readfps<<std::endl;
	
	if(showingstuff)
	{
		brightness_slider = the_webcam.get(CV_CAP_PROP_BRIGHTNESS);
		contrast_slider = the_webcam.get(CV_CAP_PROP_CONTRAST);
		gain_slider = the_webcam.get(CV_CAP_PROP_GAIN);
		saturation_slider = the_webcam.get(CV_CAP_PROP_SATURATION);
		exposure_slider = the_webcam.get(CV_CAP_PROP_EXPOSURE);
		exposure_slider+=20;
	}
	else
	{
#if 1
		the_webcam.set(CV_CAP_PROP_FRAME_WIDTH, 320.0);
		the_webcam.set(CV_CAP_PROP_FRAME_HEIGHT, 240.0);
		the_webcam.set(CV_CAP_PROP_FPS, 30.0);
		
		//MultiPlatformSleep(2000);
		
		//the_webcam.set(CV_CAP_PROP_EXPOSURE, -10.0);
#else
		the_webcam.set(CV_CAP_PROP_FRAME_WIDTH, 640.0);
		the_webcam.set(CV_CAP_PROP_FRAME_HEIGHT, 480.0);
		the_webcam.set(CV_CAP_PROP_FPS, 15.0);
#endif
	}
}


