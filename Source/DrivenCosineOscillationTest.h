#ifndef ___DRIVEN_COSINE_OSCILLATION_TEST_H____
#define ___DRIVEN_COSINE_OSCILLATION_TEST_H____

#include "SimplerGameSimSystem.h"
#include <opencv2/opencv.hpp>

#include "EstimationAndControl/EKF/DCMotor_PendCart_EKF.h"
#include "Arduino/ArduinoCommsInterface.h"
#include "Webcam/SimulationSnapshotsSimulatingWebcam.h"


class DrivenCosineOscillationTest : public SimplerGameSimSystem
{
	PendulumCartDCM2_Constants my_pcsys_constants;
	
	double simulation_time_elapsed_mytracker;
	double amplitude_of_driving_oscillation;
	double omega_of_driving_oscillation;
	double drivertimeoffset;
	double speedForBumperButtons;
	bool enableAnotherBumperSpeedChange;
	
//--------------------------------------------------
	double drawn_but_not_physical__cart_limits;
	double drawn_but_not_physical__cart_limits_inner;
	
	double last_requested_joy_PWM;
	double keyboard_PWM_requested_saved;
	double debugging_last_linearized_control_PWM;
	
	CV_PendCart_Measurer my_pendcart_measurer;
	dcmotor_pendcart_EKF  my_kalman_filter;
	
	bool control_enabled_overriding_joystick;
	
	ArduinoSerialComm arduinoCommunicator;
	SimulationSnapshotsSimulatingWebcam myComputerVisionPendulumFinder;
	
	RNG_rand_r rand_device;
	phys::dcmotor22_pendcart * mypcart;
	
public:
	double SimulatedMeasurePendulumTheta() {return mypcart->positions[0].y;}
	double SimulatedMeasureCartX() {return mypcart->positions[0].x;}
	
	DrivenCosineOscillationTest() : SimplerGameSimSystem(), mypcart(nullptr) {}
	~DrivenCosineOscillationTest() {}
	
	virtual void UpdateSystemStuff_OncePerFrame(double frametime);
	
	virtual void RespondToKeyStates();

	virtual double GetGridWidth_ForDrawingPlanes() const {return 0.1;}
	virtual void DrawSystemStuff();
	virtual bool FormatTextInfo(char* text_buffer, int line_n);

	virtual void InitBeforeSimStart();
};




#endif
