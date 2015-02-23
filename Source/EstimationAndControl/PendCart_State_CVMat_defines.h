#ifndef ___PEND_CART_STATE_CV_MAT_DEFINITIONS_H__________
#define ___PEND_CART_STATE_CV_MAT_DEFINITIONS_H__________


#ifndef ST_omega
#ifndef ST_theta
#ifndef ST_cartx

#define ST_theta at<double>(0,0)
#define ST_omega at<double>(1,0)
#define ST_cartx at<double>(2,0)
#define ST_cartx_dot at<double>(3,0)

#define POSMEAS__theta at<double>(0,0)
#define POSMEAS__cartx at<double>(1,0)

#define VELMEAS__omega at<double>(0,0)
#define VELMEAS__cartv at<double>(1,0)

#endif // ST_cartx
#endif // ST_theta
#endif // ST_omega


#endif
