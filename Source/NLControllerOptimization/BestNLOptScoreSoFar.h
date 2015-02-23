#ifndef ___BEST_SCORE_SO_FAR_FOR_NLOPT_H____
#define ___BEST_SCORE_SO_FAR_FOR_NLOPT_H____

#include "OptimizedNonlinearController.h"
#include <mutex>
#include <nlopt.h>
#include <thread>


class BestNLOptScoreSoFar
{
public:
	std::mutex my_mutex;
	std::thread* thethread;
	
	bool force_stop;
	nlopt_opt opt;
	
	MyNLOPTdata my_nloptdata;
	int num_iterations_so_far;
	
	double* my_best_params;
	double my_best_score;
	
	double original_starting_score;
	int which_algorithm;
	
	BestNLOptScoreSoFar() : my_best_params(nullptr), thethread(nullptr), opt(nullptr), force_stop(false) {}
	BestNLOptScoreSoFar(const BestNLOptScoreSoFar & other) : my_nloptdata(other.my_nloptdata), thethread(other.thethread), my_best_params(other.my_best_params), force_stop(other.force_stop), opt(other.opt) {}
	
	void Reset() {
		num_iterations_so_far = 0;
		if(my_best_params != nullptr) {free(my_best_params); my_best_params = nullptr;}
		my_best_score = 1e12;
		original_starting_score = 1e12;
		which_algorithm = 0;
		force_stop = false;
		if(opt != nullptr) {nlopt_destroy(opt);}
		opt = nullptr;
	}
};



#endif
