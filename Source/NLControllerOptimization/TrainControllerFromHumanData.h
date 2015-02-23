#ifndef ___TRAIN_CONTROLLER_FROM_HUMAN_DATA_H_____
#define ___TRAIN_CONTROLLER_FROM_HUMAN_DATA_H_____

#include "OptimizedNonlinearController.h"



class TrainerFromHumanData
{
protected:
	NonlinearController_Optimized  AveragingController4D_WeightedSumNumerator;
	NonlinearController_Optimized  AveragingController4D_WeightedSumDenominator;
	
public:
	std::string saved_filename;
	std::string base_folder_containing_given_files;
	std::vector<std::string> saved_filenames_from_folder;
	
	NonlinearController_Optimized * Load(int gridpoints_per_axis, bool true_if_humankalman_false_if_nlopt_saved);
};

#endif
