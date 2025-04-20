#ifndef MODEL_AERODYNAMICS_H
#define MODEL_AERODYNAMICS_H
#include "application/estimator/estimator_types.h"
#include "application/estimator/model/model_airdata.h"
#include "third_party/rocketlib/include/common.h"

/*
parameters shared across aerodynamics and dynamics
*/
extern const double c_canard;
extern const double cn_alpha;
extern const double c_aero;

/**
 * @brief Computes the aerodynamic forces and torque
 * @param x_state estimator state
 * @param estimator_airdata_t airdata results from model_airdata
 * @param pointer to resulting torque vector
 */
void aerodynamics(const x_state_t *state, const estimator_airdata_t *airdata, vector3d_t *torque);

/**
 * @brief returns CL to expected value slowly, to force convergence in EKF
 * @param double mach_num, pre-computed in model_dynamics
 * @return double CL
 */
double airfoil(double mach_num);

#endif // MODEL_AERODYNAMICS_H