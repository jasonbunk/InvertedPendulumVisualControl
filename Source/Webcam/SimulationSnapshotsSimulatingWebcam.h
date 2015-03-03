#ifndef ___SIMULATION_SNAPSHOTS_SIMULATING_A_WEBCAM_H_____
#define ___SIMULATION_SNAPSHOTS_SIMULATING_A_WEBCAM_H_____

#include "RealtimeVideoProcessor.h"
#include "EstimationAndControl/StateAndMeasurementClasses.h"

class SimulationSnapshotsSimulatingWebcam : public RealtimeVideoProcessor
{
protected:
	double image_capture_delay_time;
	std::vector<CV_PendCart_Raw_Measurement> RecentMeasurementsBeingDelayed;
	double trueCurrSimTime;
	
	virtual void DoInitialization();
	virtual void CheckForCapturedImage(double *& possible_returned_delaytime, cv::Mat *& possible_returned_img);
	virtual CV_PendCart_Raw_Measurement* ProcessImageToMeasurements(cv::Mat * givenMat);

public:
	void SetTrueSimTime(double latestTrueCurrSimTime) {trueCurrSimTime = latestTrueCurrSimTime;}
	
	void GiveCurrentTrueTimeMeasurement(const CV_PendCart_Raw_Measurement & new_measurements_to_insert);
};




#endif
