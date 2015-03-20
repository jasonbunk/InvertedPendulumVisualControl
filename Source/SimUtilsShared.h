#ifndef ___SIM_UTILS_SHARED_H_____
#define ___SIM_UTILS_SHARED_H_____

#include <stdint.h>
#include <opencv2/opencv.hpp>

uint8_t ConvertPWMtoArduinoByte(double PWM);

void DrawKalmanCart(cv::Mat givenState, double cartwidth, double cartheight, double boblength, double bobdiam, unsigned char R, unsigned char G, unsigned char B);


class savedstatesfordebugging
{
	int currStateIdx;
	int maxidx;
	bool writtenBeforeCleared;
	bool recentlyCleared;
	
	int currRealThetaIdx;
	std::vector<float> realthetas;
	std::vector<float> realthetatimes;
	int currRealOmegaIdx;
	std::vector<float> realomegas;
	std::vector<float> realomegatimes;
	int currRealCartxIdx;
	std::vector<float> realcartxs;
	std::vector<float> realcartxtimes;
	int currRealCartvelIdx;
	std::vector<float> realcartvels;
	std::vector<float> realcartveltimes;
	double startTime;
	
	std::vector<float> saved_thetas;
	std::vector<float> saved_omegas;
	std::vector<float> saved_cartxs;
	std::vector<float> saved_cartvels;
	std::vector<float> saved_sdFs;
	std::vector<float> saved_LQR_applied;
	
public:
	void Init(int numToSave);
	void InsertCurrentState(cv::Mat state, double LQR_applied);
	void InsertRealTheta(double theta, double time);
	void InsertRealOmega(double omega, double time);
	void InsertRealCartx(double cartx, double time);
	void InsertRealCartv(double cartv, double time);
	void Clear();
	void SetStartTime(double newStartTime) {startTime = newStartTime; currRealCartvelIdx = currRealCartxIdx = currRealOmegaIdx = currRealThetaIdx = 0;}
	void WriteToFile(std::string estfilename, std::string realthetafilename, std::string realomegafilename, std::string realcartxfilename, std::string realcartvelfilename);
};

#endif
