#ifndef ___PENDULUM_CART_CONSTANTS_PARAMETERS_H______
#define ___PENDULUM_CART_CONSTANTS_PARAMETERS_H______



class PendulumCartDCM2_Constants
{
public:
	//global params
	double g;   //gravitational acceleration (normally +9.81)
	
	//pendulum params
	double m;   //total mass of pendulum
	double Ipc; //rotational inertia of pendulum about its center-of-mass
	double l;   //distance from attachment point to pendulum center-of-mass
	double kp;  //angular drag constant (~force -kp*omega)
	double theta_measurement_noise_stddev;
	double pendulum_process_noise_accelerations_stddev;
	double omega_measurement_noise_stddev;
	
	//cart params
	double MC;   //mass of cart + Imotor/rmotor^2
	double kc;  //linear drag constant (includes both cart and motor friction)
	double cart_x_measurement_noise_stddev;
	double cart_x_process_noise_accelerations_stddev;
	double cart_vel_measurement_noise_stddev;
	
	//motor params
	double uscalar;  //(stalltorque / motorradius) == proportionality factor scaling PWM to a force on the cart
	double sdtau; //time constant tau of the pretend series LR circuit that models the signal delay from laptop to Arduino
	
	//miscellaneous (for control systems?)
	double cart_track_limits__max_hardlimit;
	double cart_track_limits__inner_limit;
	double max_displacement_of_cart_around_COMom_while_pend_swings;
	
	PendulumCartDCM2_Constants() :
		g(0.0),
		m(0.0),
		Ipc(0.0),
		l(0.0),
		kp(0.0),
		MC(0.0),
		kc(0.0),
		uscalar(0.0),
		sdtau(0.0),
		theta_measurement_noise_stddev(0.01),
		pendulum_process_noise_accelerations_stddev(0.01),
		omega_measurement_noise_stddev(0.01),
		cart_x_measurement_noise_stddev(0.01),
		cart_x_process_noise_accelerations_stddev(0.01),
		cart_vel_measurement_noise_stddev(0.01),
		cart_track_limits__max_hardlimit(0.01),
		cart_track_limits__inner_limit(0.01),
		max_displacement_of_cart_around_COMom_while_pend_swings(0.1) {}
};




#endif
