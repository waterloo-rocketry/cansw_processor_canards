#include "application/estimator/model/model_aerodynamics.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include <math.h>
#include <stdlib.h>

/**
 * constants
 */
// aerodynamics
static const double cn_alpha = 80.0; // pitch forcing coeff
static const double c_canard =
    (2 * (4 * 0.0254) * (2.5 * 0.0254)) * (0.203 / 2 + 0.0254); // moment arm * area of canard
// c_aero = area_reference * (length_cp-length_cg), center of pressure(cp): -0.5, center of
// gravity(cg): 0
static const double area_reference = M_PI * pow((0.203 / 2), 2); // cross section of body tube
static const double c_aero = area_reference * (-0.5);

// helper function
// static inline double sign(double x) {
//     return (x) > 0 ? 1 : ((x) < 0 ? -1 : 0);
// } // returns the sign of x; pos: 1, neg: -1, zero: 0

void aerodynamics(const x_state_t *state, const estimator_airdata_t *airdata, vector3d_t *torque) {
    const double p_dyn = airdata->density / 2.0 * pow(math_vector3d_norm(&(state->velocity)), 2);

    double sin_alpha = 0.0, sin_beta = 0.0;
    // angle of attack/sideslip
    if (state->velocity.x >= 0.5) {
        sin_alpha = (state->velocity.z / state->velocity.x);
        sin_beta = -(state->velocity.y / state->velocity.x);
    } else {
        sin_alpha = M_PI / 2;
        sin_beta = -M_PI / 2;
    }

    // torque calculations
    const vector3d_t torque_unit_x = {.array = {1, 0, 0}};
    const vector3d_t torque_sin = {.array = {0, sin_alpha, sin_beta}};

    const vector3d_t torque_canards =
        math_vector3d_scale(state->CL * state->delta * c_canard * p_dyn, &torque_unit_x);
    const vector3d_t torque_aero = math_vector3d_scale(
        p_dyn * c_aero * cn_alpha, &torque_sin
    ); // p_dyn * ( param.c_aero * param.Cn_alpha * [0; sin_alpha; sin_beta])
    *torque = math_vector3d_add(&torque_canards, &torque_aero);
}

double airfoil(double mach_num) {
    // airfoil constants
    double canard_sweep = deg2rad(60); // 60 degrees in radians
    double Cl_alpha = 2 * M_PI * cot(canard_sweep); // estimated coefficient of lift, const with Ma

    // airfoil calc
    double Cl_theory = 0;

    if (mach_num <= 1) {
        Cl_theory = Cl_alpha;
    } else {
        const double cone = acos(1 / mach_num);
        if (cone > canard_sweep) {
            Cl_theory = 4 / sqrt(pow(mach_num, 2) - 1); // 4 / sqrt(mach_num^2 - 1)
        } else {
            const double m = cot(canard_sweep) / cot(cone);
            const double a = m * (0.38 + 2.26 * m - 0.86 * m * m); // m*(0.38+2.26*m-0.86*m^2)
            Cl_theory = 2 * M_PI * M_PI * cot(canard_sweep) /
                        (M_PI + a); // 2*pi^2*cot(param.canard_sweep) / (pi + a)
        }
    }

    return Cl_theory;
}