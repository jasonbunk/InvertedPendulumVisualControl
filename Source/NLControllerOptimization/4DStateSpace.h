#ifndef _____4D_STATE_SPACE__H______
#define _____4D_STATE_SPACE__H______

#include <string.h>
#include <vector>
#include <math.h>

/*======================================================================================
	Information about a state space variable (num gridpoints, spatial widths of bins, etc)
*/
class MyStateVariable
{
public:
	double expect_min;
	double expect_max;
	double expect_range_min_to_max;
	double bin_width;
	std::vector<double> gridvalues_lookup; //position (in state space) of each grid point
	
	void SetRange(double EXPECTED_MAX, int NUM_GRIDPOINTS);
};


/*======================================================================================
	grid point indices for the 4 state space variables
*/
class binIndices
{
public:
	int thetabin, omegabin, cartxbin, cartvbin;
	
	binIndices() : thetabin(-1), omegabin(-1), cartxbin(-1), cartvbin(-1) {}
	binIndices(int THETABIN, int OMEGABIN, int CARTXBIN, int CARTVBIN) :
		thetabin(THETABIN), omegabin(OMEGABIN), cartxbin(CARTXBIN), cartvbin(CARTVBIN) {}
	
	binIndices& operator+=(const binIndices& rhs) {
		thetabin += rhs.thetabin;
		omegabin += rhs.omegabin;
		cartxbin += rhs.cartxbin;
		cartvbin += rhs.cartvbin;
		return *this; // return the result by reference
	}
	// friends defined inside class body are inline and are hidden from non-ADL lookup
	friend binIndices operator+(binIndices lhs,       // passing first arg by value lets us reuse += operator
								const binIndices& rhs) {
		return lhs += rhs; // reuse compound assignment and return the result by value
	}
	binIndices& operator*=(const binIndices& rhs) {
		thetabin *= rhs.thetabin;
		omegabin *= rhs.omegabin;
		cartxbin *= rhs.cartxbin;
		cartvbin *= rhs.cartvbin;
		return *this; // return the result by reference
	}
	// friends defined inside class body are inline and are hidden from non-ADL lookup
	friend binIndices operator*(binIndices lhs,       // passing first arg by value lets us reuse += operator
								const binIndices& rhs) {
		return lhs *= rhs; // reuse compound assignment and return the result by value
	}
	friend std::ostream& operator<< (std::ostream &out, const binIndices &bIdxs) {
		out << "(" << bIdxs.thetabin << ", " << bIdxs.omegabin << ", "<<bIdxs.cartxbin << ", " << bIdxs.cartvbin << ")";
		return out;
	}
};

/*======================================================================================
	"alphas" (interpolation values) for the 4 state space variables
*/
class binOffGridAlphas
{
public:
	double atheta, aomega, acartx, acartv;
	
	binOffGridAlphas() : atheta(0.0), aomega(0.0), acartx(0.0), acartv(0.0) {}
	binOffGridAlphas(double ATHETA, double AOMEGA, double ACARTX, double ACARTV)
				: atheta(ATHETA), aomega(AOMEGA), acartx(ACARTX), acartv(ACARTV) {}
	
	double GetLengthFrom0() {return sqrt(atheta*atheta + aomega*aomega + acartx*acartx + acartv*acartv);}
	double GetLength(binIndices binaryIndicesToInvertAlpha);
	
	friend std::ostream& operator<< (std::ostream &out, const binOffGridAlphas &bAlphas) {
		out << "(" << bAlphas.atheta << ", " << bAlphas.aomega << ", "<<bAlphas.acartx << ", " << bAlphas.acartv << ")";
		return out;
	}
};


/*======================================================================================
	Generic 4D state space representation class
*/
template<typename T>
class MyPhaseSpaceGrid4DGeneric
{
public:
	virtual void Init(int constDimsSize4D) = 0;
	virtual void Init(int* dims4D) = 0;
	virtual T c_at(int i0, int i1, int i2, int i3) const = 0;
	virtual T & at(int i0, int i1, int i2, int i3) = 0;
	virtual T & at(binIndices idx) = 0;
	virtual void SetAllTo(T val) = 0;
	virtual int CountNonzero() = 0;
};


/*======================================================================================
	4D state space in C pointer format (to a flattened 1D array representing the full 4D array)
*/
template<typename T>
class MyPhaseSpaceGrid4D_C_Style : public MyPhaseSpaceGrid4DGeneric<T>
{
protected:
	T * grid;
	int totalgridpts;
	int dims[4];
public:
	const T * givenGridOverride; //set during nlopt to current parameter space
	
	
	MyPhaseSpaceGrid4D_C_Style() : grid(nullptr), givenGridOverride(nullptr), totalgridpts(0) {}
	~MyPhaseSpaceGrid4D_C_Style() {if(grid != nullptr){free(grid);} grid = nullptr;}
	
	virtual void Init(int constDimsSize4D) {
		totalgridpts = (constDimsSize4D*constDimsSize4D*constDimsSize4D*constDimsSize4D);
		int bytes_to_malloc = totalgridpts*((int)sizeof(T));
		grid = (T*)malloc(bytes_to_malloc);
		memset(grid, 0, bytes_to_malloc);
	}
	virtual void Init(int* dims4D) {
		dims[0] = dims4D[0];
		dims[1] = dims4D[1];
		dims[2] = dims4D[2];
		dims[3] = dims4D[3];
		totalgridpts = (dims[0]*dims[1]*dims[2]*dims[3]);
		int bytes_to_malloc = totalgridpts*((int)sizeof(T));
		grid = (T*)malloc(bytes_to_malloc);
		memset(grid, 0, bytes_to_malloc);
	}
	
	virtual T c_at(int i0, int i1, int i2, int i3) const {return givenGridOverride != nullptr ? givenGridOverride[i3+dims[3]*(i2+dims[2]*(i1+dims[1]*i0))] : grid[i3+dims[3]*(i2+dims[2]*(i1+dims[1]*i0))];}
	virtual T & at(int i0, int i1, int i2, int i3) {return grid[i3+dims[3]*(i2+dims[2]*(i1+dims[1]*i0))];}
	virtual T & at(binIndices idx) {return grid[idx.cartvbin+dims[3]*(idx.cartxbin+dims[2]*(idx.omegabin+dims[1]*idx.thetabin))];}
									//{return grid[idx.thetabin][idx.omegabin][idx.cartxbin][idx.cartvbin];}
	
	virtual void SetAllTo(T val) {
		for(int nn=0; nn < totalgridpts; nn++) {
			grid[nn] = val;
		}
	}
	virtual int CountNonzero() {
		int retcount = 0;
		for(int nn=0; nn < totalgridpts; nn++) {
			if(grid[nn] != ((T)0)) {retcount++;}
		}
		return retcount;
	}
	
//------------------------------------- not in the generic class
	int GetTotalGridPoints() {return totalgridpts;}
	T* GetGridCptr() {return grid;}
	int* GetDims() {return dims;}
	
	friend std::ostream& operator<< (std::ostream &out, const MyPhaseSpaceGrid4D_C_Style<T> &printMe) {
		if(printMe.givenGridOverride != nullptr) {
			for(int nn=0; nn < printMe.totalgridpts; nn++) {
				out << (printMe.givenGridOverride)[nn] << std::endl;
			}
		} else {
			for(int nn=0; nn < printMe.totalgridpts; nn++) {
				out << printMe.grid[nn] << std::endl;
			}
		}
		return out;
	}
	void ChangeGridToThis(T * newGrid) {
		int bytes_to_malloc = totalgridpts*((int)sizeof(T));
		memcpy(grid, newGrid, bytes_to_malloc);
	}
};


/*======================================================================================
	Template types to be used for 4D phase space implementation that uses C++ std::vector<>
*/
template<typename T> using PhaseSpaceGrid1D =                                        std::vector<T>;
template<typename T> using PhaseSpaceGrid2D =                           std::vector< std::vector<T> >;
template<typename T> using PhaseSpaceGrid3D =              std::vector< std::vector< std::vector<T> > >;
template<typename T> using PhaseSpaceGrid4D = std::vector< std::vector< std::vector< std::vector<T> > > >;


/*======================================================================================
	4D state space in C++ std::vector<> format
*/
template<typename T>
class MyPhaseSpaceGrid4DClass : public MyPhaseSpaceGrid4DGeneric<T>
{
public:
	PhaseSpaceGrid4D<T> grid;
	
	virtual void Init(int constDimsSize4D) {
		grid.resize(constDimsSize4D, PhaseSpaceGrid3D<T>(constDimsSize4D, PhaseSpaceGrid2D<T>(constDimsSize4D, PhaseSpaceGrid1D<T>(constDimsSize4D, (T)0))));
	}
	virtual void Init(int* dims4D) {
		grid.resize(dims4D[0], PhaseSpaceGrid3D<T>(dims4D[1], PhaseSpaceGrid2D<T>(dims4D[2], PhaseSpaceGrid1D<T>(dims4D[3], (T)0))));
	}
	
	virtual T c_at(int i0, int i1, int i2, int i3) const {return grid[i0][i1][i2][i3];}
	virtual T & at(int i0, int i1, int i2, int i3) {return grid[i0][i1][i2][i3];}
	virtual T & at(binIndices idx) {return grid[idx.thetabin][idx.omegabin][idx.cartxbin][idx.cartvbin];}
	
	virtual void SetAllTo(T val) {
		for(int ii=0; ii < grid.size(); ii++) {
			for(int jj=0; jj < grid[0].size(); jj++) {
				for(int kk=0; kk < grid[0][0].size(); kk++) {
					for(int mm=0; mm < grid[0][0][0].size(); mm++) {
						grid[ii][jj][kk][mm] = val;
					}
				}
			}
		}
	}
	virtual int CountNonzero() {
		int retcount = 0;
		for(int ii=0; ii < grid.size(); ii++) {
			for(int jj=0; jj < grid[0].size(); jj++) {
				for(int kk=0; kk < grid[0][0].size(); kk++) {
					for(int mm=0; mm < grid[0][0][0].size(); mm++) {
						if(grid[ii][jj][kk][mm] != ((T)0)) {retcount++;}
					}
				}
			}
		}
		return retcount;
	}
};



#endif
