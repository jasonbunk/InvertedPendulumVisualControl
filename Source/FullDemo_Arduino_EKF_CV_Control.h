#ifndef ___FINAL_DCM2_ARDUINO_KALMAN_OPENCV_H____
#define ___FINAL_DCM2_ARDUINO_KALMAN_OPENCV_H____


#include "SimplerGameSimSystem.h"
#include <deque>
#include "Utils/random_normal_dist_using_c.h"
#include <opencv2/opencv.hpp>
#include "EstimationAndControl/Control/Controller_SavedHumanReplay.h"
#include "EstimationAndControl/Control/Controller_Position.h"
#include "EstimationAndControl/Control/Controller_NonlinearSwingup.h"
#include "EstimationAndControl/Control/Controller_LQR.h"

#include "EstimationAndControl/EKF/DCMotor_PendCart_EKF.h"

#include "Webcam/ColoredPendOrientationFinder.h"

#include "Arduino/ArduinoCommsInterface.h"



class Simulation_FinalDCM2ArduinoKalmanCV : public SimplerGameSimSystem
{
	PendulumCartDCM2_Constants my_pcsys_constants;
	
	double simulation_time_elapsed_mytracker;
	
//--------------------------------------------------
	double drawn_but_not_physical__cart_limits;
	double drawn_but_not_physical__cart_limits_inner;
	
	double keyboard_PWM_requested_saved;
	double debugging_last_linearized_control_PWM;
	
	
	dcmotor_pendcart_EKF  my_kalman_filter;
	
	double latest_LQR_control_PWM;
	dcm2_controller_inverted_linearized_LQR   mycontroller_LQR;
	nonlinear_swingup_optimal_controller   mycontroller_nlswing;
	dcm2_pc_position_controller_pid   mycontroller_position;
	dcm2_controller_savedhumanreplay mycontroller_replay;
	
//--------------------------------------------------
	std::ofstream * outFile;
	void FinishUp();
//--------------------------------------------------
	
	ColoredPendOrientationFinder myComputerVisionPendulumFinder;
	
	ArduinoSerialComm arduinoCommunicator;
	
	RNG_rand_r rand_device;
	phys::dcmotor22_pendcart * mypcart;
	
public:
	double SimulatedMeasurePendulumTheta() {return mypcart->positions[0].y;}
	double SimulatedMeasureCartX() {return mypcart->positions[0].x;}
	
	Simulation_FinalDCM2ArduinoKalmanCV() : SimplerGameSimSystem(), mypcart(nullptr), outFile(nullptr) {}
	~Simulation_FinalDCM2ArduinoKalmanCV() {FinishUp();}
	
	virtual double GetGridWidth_ForDrawingPlanes() const {return 0.1;}
	
	virtual void UpdateSystemStuff_OncePerFrame(double frametime);
	
	virtual void RespondToKeyStates();

	virtual void DrawSystemStuff();
	virtual bool FormatTextInfo(char* text_buffer, int line_n);

	virtual void InitBeforeSimStart();
};




#endif
