#ifndef ___FINAL_DCM2_ARDUINO_KALMAN_OPENCV_H____
#define ___FINAL_DCM2_ARDUINO_KALMAN_OPENCV_H____


#include "SimplerGameSimSystem.h"
#include <opencv2/opencv.hpp>
#include "SimUtilsShared.h"

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
	
	savedstatesfordebugging historySaved;
	bool historyStartedRecently;
	bool isRecording;
	
	bool enableSineWave;
	double omegasinfreq;
	double sinewaveenabledstarttime;
	bool enableAnotherBumperSpeedChange;
	
//--------------------------------------------------
	double drawn_but_not_physical__cart_limits;
	double drawn_but_not_physical__cart_limits_inner;
	
	double keyboard_PWM_requested_saved;
	double debugging_last_linearized_control_PWM;
	
	
	CV_PendCart_Measurer my_pendcart_measurer;
	dcmotor_pendcart_EKF  my_kalman_filter;
	
	bool LQR_control_enabled_overriding_joystick;
	
	double last_LQR_control_PWM;
	dcm2_controller_inverted_linearized_LQR   mycontroller_LQR;
	nonlinear_swingup_optimal_controller   mycontroller_nlswing;
	dcm2_pc_position_controller_pid   mycontroller_position;
	dcm2_controller_savedhumanreplay mycontroller_replay;
	
	bool useWebcamForVision; //default: yes
	bool calibrateWebcamVision; //default: no
	ColoredPendOrientationFinder myComputerVisionPendulumFinder;
	
	ArduinoSerialComm arduinoCommunicator;
	
	RNG_rand_r rand_device;
	phys::dcmotor22_pendcart * mypcart;
	
public:
	Simulation_FinalDCM2ArduinoKalmanCV() : SimplerGameSimSystem(), useWebcamForVision(true), calibrateWebcamVision(false), mypcart(nullptr) {}
	~Simulation_FinalDCM2ArduinoKalmanCV() {}
	
	void SetWebcamUse(bool useit) {useWebcamForVision = useit;}
	void TellWebcamVisionToCalibrate(bool calibrate) {calibrateWebcamVision = calibrate;}
	
	virtual void UpdateSystemStuff_OncePerFrame(double frametime);
	
	virtual void RespondToKeyStates();

	virtual double GetGridWidth_ForDrawingPlanes() const {return 0.1;}
	virtual void DrawSystemStuff();
	virtual bool FormatTextInfo(char* text_buffer, int line_n);

	virtual void InitBeforeSimStart();
};




#endif
