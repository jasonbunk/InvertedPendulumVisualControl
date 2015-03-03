#include "ImageProcessor.h"
#include "TryIncludeJPhysics.h"
#include "EstimationAndControl/PendCart_State_CVMat_defines.h"
#include "PrinterPhys124_PendCart_Params.h"
#include <fstream>
#include <iostream>
using std::cout; using std::endl;


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


double ImageProcessor::GetPendAngleFromSegmentedImg(cv::Mat * segImgForCart, cv::Mat * segImgForPend,
													bool drawstuff,
													double cart_x, double cart_y, double & returned_theta,
													cv::Mat * returnedDrawnHere /*= nullptr*/) const
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
		cv::waitKey(1);
	}
}


void ImageProcessor::GetCartLocationFromSegmentedImg(cv::Mat * segImgForCart,
											bool drawstuff,
											double & cart_x, double & cart_y,
											cv::Mat * returnedDrawnHere /*= nullptr*/, int cartx_track_pix_min/*=0*/, int cartx_track_pix_max/*=0*/) const
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


void ImageProcessor::DrawCartTracks(cv::Mat & onImg, double cart_x, double cart_y)
{
	cv::line(onImg, cv::Point(cartx_track_pix_min,cart_y-5), cv::Point(cartx_track_pix_min,cart_y+5), CV_RGB(0,0,255), 2);
	cv::line(onImg, cv::Point(cartx_track_pix_max,cart_y-5), cv::Point(cartx_track_pix_max,cart_y+5), CV_RGB(0,0,255), 2);
	
	cv::line(onImg, cv::Point(cart_x,cart_y-5), cv::Point(cart_x,cart_y+5), CV_RGB(0,255,0), 2);
}


void ImageProcessor::GetBinaryImg_PendAngle(cv::Mat & colorImg, cv::Mat & returnedBinary)
{
	cv::cvtColor(colorImg, returnedBinary, CV_BGR2HLS);
	
	cv::Scalar colorsLow(HH_l, LL_l, SS_l);
	cv::Scalar colorsHigh(HH_h, LL_h, SS_h);
	
	cv::inRange(returnedBinary, colorsLow, colorsHigh, returnedBinary);
	
	Erosion(returnedBinary, cv::MORPH_CROSS, 3);
	Dilation(returnedBinary, cv::MORPH_RECT, 3);
}

void ImageProcessor::GetBinaryImg_CartX(cv::Mat & colorImg, cv::Mat & returnedBinary)
{
	cv::cvtColor(colorImg, returnedBinary, CV_BGR2HLS);

	cv::Scalar colorsLow(HHcar_l, LLcar_l, SScar_l);
	cv::Scalar colorsHigh(HHcar_h, LLcar_h, SScar_h);
	
	cv::inRange(returnedBinary, colorsLow, colorsHigh, returnedBinary);
	
	Erosion(returnedBinary, cv::MORPH_CROSS, 3);
	Dilation(returnedBinary, cv::MORPH_RECT, 3);
}


CV_PendCart_Raw_Measurement * ImageProcessor::ProcessImageToMeasurements(cv::Mat * givenMat, bool display /*=false*/)
{
	cv::Mat processingm;
	cv::medianBlur(*givenMat, processingm, 3);
	
	cv::Mat pendBinary, cartBinary;
	GetBinaryImg_CartX(processingm, cartBinary);
	GetBinaryImg_PendAngle(processingm, pendBinary);
	
#if 1
	if(lastBinaryCart_Dilated.size() == cartBinary.size()) {
		assert(cartBinary.channels() == 1 && lastBinaryCart_Dilated.channels() == 1);
		cv::bitwise_and(lastBinaryCart_Dilated, cartBinary, cartBinary);
	}
	else {
		lastBinaryCart_Dilated = cv::Mat::ones(cartBinary.size(), CV_8U);
	}
	if(lastBinaryPend_Dilated.size() == pendBinary.size()) {
		cv::bitwise_and(lastBinaryPend_Dilated, pendBinary, pendBinary);
	} else {
		lastBinaryPend_Dilated = cv::Mat::ones(pendBinary.size(), CV_8U);
	}
	
	int nonzeroPixelsInCartBinary = cv::countNonZero(cartBinary);
	int nonzeroPixelsInPendBinary = cv::countNonZero(pendBinary);
	
	if(nonzeroPixelsInCartBinary < 2) {
		//cartBinary = lastBinaryCart;
	} else {
		cartBinary.copyTo(lastBinaryCart_Dilated);
		Dilation(lastBinaryCart_Dilated, cv::MORPH_RECT, 11);
	}
	if(nonzeroPixelsInPendBinary < 2) {
		//pendBinary = lastBinaryPend;
	} else {
		pendBinary.copyTo(lastBinaryPend_Dilated);
		Dilation(lastBinaryPend_Dilated, cv::MORPH_RECT, 11);
	}
#endif
	
	double cartx, carty, pendtheta;
	GetCartLocationFromSegmentedImg(&cartBinary, false, cartx, carty);
	GetPendAngleFromSegmentedImg(&cartBinary, &pendBinary, display, cartx, carty, pendtheta);
	
	if(isnan(cartx) == false && isnan(pendtheta) == false) {
		CV_PendCart_Raw_Measurement* retval = new CV_PendCart_Raw_Measurement();
		retval->cartx = ConvertCartXFromPixelCoords(cartx);
		retval->theta = ConvertThetaFromPixelCoords(pendtheta);
		return retval;
	}
	return nullptr;
}



template <typename T> T CLAMP(const T& value, const T& low, const T& high)  {
  return value < low ? low : (value > high ? high : value); 
}

double ImageProcessor::ConvertCartXFromPixelCoords(double raw_pixel_x)
{
	double unclamped = (raw_pixel_x - static_cast<double>(cartx_track_pix_min)) * (2.0 * PRINTER_LINEAR_WIDTH_X / calculated_cart_track_width_in_pixels);
	return CLAMP(unclamped-PRINTER_LINEAR_WIDTH_X, -1.0*PRINTER_LINEAR_WIDTH_X, PRINTER_LINEAR_WIDTH_X);
}

double ImageProcessor::ConvertThetaFromPixelCoords(double raw_theta)
{
	return physmath::differenceBetweenAnglesSigned(raw_theta, angle_when_vertical);
}



void ImageProcessor::LoadCalibrationFromFile(std::string filename)
{
	std::ifstream savedCalibration(filename);
	if(savedCalibration.is_open()==false || savedCalibration.good()==false) {
		cout<<"Error: could not open webcam CV calibration file: \""<<filename<<"\""<<endl;
		exit(0);
	}
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
	savedCalibration.close();
	
	calculated_cart_track_width_in_pixels = (((double)cartx_track_pix_max) - ((double)cartx_track_pix_min));
}


void ImageProcessor::SaveCalibrationToFile(std::string filename)
{
	std::ofstream savedCalibration(filename);
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
}


void ImageProcessor::PrintCalibration()
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


