#ifndef ___WEBCAM_HPP_______
#define ___WEBCAM_HPP_______

#include <opencv2/opencv.hpp>


class WebcamCV
{
    const int this_webcam_number;
    WebcamCV() : this_webcam_number(9999) {}

    std::string winNameStr;

    cv::VideoCapture the_webcam;
    cv::Mat last_received_image;

    //variables for cam camera
    int brightness_slider;
    int contrast_slider;
    int gain_slider;
    int saturation_slider;
    int exposure_slider; //exposure range is from 0 to -7.

    int img_width_received;
    int img_height_received;

    void setCameraVals();
    void getCameraVals(bool showingstuff);


public:
    void InitWindow(bool showstuff);
    void UpdateWindow(bool showstuff);


    cv::Mat * GetLastReceivedImage() {return &last_received_image;}
    const int GetImageWidth() {return img_width_received;}
    const int GetImageHeight() {return img_height_received;}
	const std::string GetWinNameStr() {return winNameStr;}


    WebcamCV(int camera_number) :
        this_webcam_number(camera_number),
        the_webcam(camera_number),
        brightness_slider(80),
        contrast_slider(80),
        gain_slider(70),
        saturation_slider(80),
        exposure_slider(1),
        img_width_received(-1),
        img_height_received(-1)
        {}
};

#endif // ___WEBCAM_HPP_______
