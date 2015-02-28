#ifndef ___OPTIMIZED_NONLINEAR_CONTROLLER_H_____
#define ___OPTIMIZED_NONLINEAR_CONTROLLER_H_____

/*
	Tries to find the optimum time-invariant control solution: u(STATE)
	
		discretization of continuous STATE onto a discrete grid: STATE'
		
		optimizing u(STATE') on that grid
		
		Then, given any continuous point STATE,
		use interpolation to get: u(interpolate(STATE in STATE'))
	
	Since this uses linear interpolation (and simple boundary conditions),
	optimizing on a grid with 3 points per dimension will
	effectively produce a linearized control solution about the middle grid point.
	It may not be the same as the optimal LQR solution,
		where (LQR) is linearized state --> linear control,
		since this would be nonlinear state --> linear control.
		Though if your only interest is linearized about a setpoint then you might as well use LQR.
*/

#include <ostream>
#include "4DStateSpace.h"




/*======================================================================================
	The actual controller, where GetControl() is used when given a point in state space
*/
class NonlinearController_Optimized
{
public:
	enum StateVarsEnum {
		theta = 0,
		omega = 1,
		cartx = 2,
		cartv = 3,
		NumStateVars
	};
protected:
	std::vector<MyStateVariable> vars;
	
	//MyPhaseSpaceGrid4DClass<double> Controller4D;
	MyPhaseSpaceGrid4D_C_Style<double> Controller4D;
public:
	int GetTotalGridPoints() {return Controller4D.GetTotalGridPoints();}
	double* GetGridCptr() {return Controller4D.GetGridCptr();}
	MyPhaseSpaceGrid4D_C_Style<double> & GetMyController4Dcstyle() {return Controller4D;}
	int* GetGridDims() {return Controller4D.GetDims();}
	//-----------------------------------------------------------------------
	
	
	MyPhaseSpaceGrid4DClass<int> * visitedPoints;
	
	
	NonlinearController_Optimized() : visitedPoints(nullptr) {}
	
	
	void SaveMeToFile(std::string filename);
	void InitFromFile(std::string controllerfile, double scale_width_around_setpoint = 1.0);
	void Init(int gridpoints, double scale_width_around_setpoint = 1.0);
	void Init(int* griddims, double scale_width_around_setpoint = 1.0);
	
	void GetGridCornersForOffGridPoint(double THETA, double OMEGA, double CARTX, double CARTV,
											binIndices & bin0, binIndices & bin1,
											binOffGridAlphas & ba);
	
	void EnforceXBoundaryConditions();
	
	double cget_at(int i0, int i1, int i2, int i3) const {return Controller4D.c_at(i0,i1,i2,i3);}
	double & get_at(int i0, int i1, int i2, int i3) {return Controller4D.at(i0,i1,i2,i3);}
	double & get_at(binIndices idx) {return Controller4D.at(idx.thetabin,idx.omegabin,idx.cartxbin,idx.cartvbin);}
	
	double GetControl(double THETA, double OMEGA, double CARTX, double CARTV);
	
	void AddSamplePoint(double CONTROLU, double THETA, double OMEGA, double CARTX, double CARTV);
	
	
	NonlinearController_Optimized& operator-=(const NonlinearController_Optimized& rhs);
	friend NonlinearController_Optimized operator-(NonlinearController_Optimized lhs, // passing first arg by value lets us reuse += operator
													const NonlinearController_Optimized& rhs) {
		return lhs -= rhs; // reuse compound assignment and return the result by value
	}NonlinearController_Optimized& operator+=(const NonlinearController_Optimized& rhs);
	friend NonlinearController_Optimized operator+(NonlinearController_Optimized lhs, // passing first arg by value lets us reuse += operator
													const NonlinearController_Optimized& rhs) {
		return lhs += rhs; // reuse compound assignment and return the result by value
	}
	NonlinearController_Optimized& operator*=(const NonlinearController_Optimized& rhs);
	friend NonlinearController_Optimized operator*(NonlinearController_Optimized lhs, // passing first arg by value lets us reuse += operator
													const NonlinearController_Optimized& rhs) {
		return lhs *= rhs; // reuse compound assignment and return the result by value
	}
	NonlinearController_Optimized& operator/=(const NonlinearController_Optimized& rhs);
	friend NonlinearController_Optimized operator/(NonlinearController_Optimized lhs, // passing first arg by value lets us reuse += operator
													const NonlinearController_Optimized& rhs) {
		return lhs /= rhs; // reuse compound assignment and return the result by value
	}
	
	void ExpandOffGridNANcontrols();
protected:
	double GetNeighborhoodAvg(int i, int j, int k, int m);
};


#endif
