#include "application/estimator/model/model_aerodynamics.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include <math.h>
#include <stdlib.h>

/**
 * constants
 */
// aerodynamics
static const double cn_alpha = 20.0; // pitch forcing coeff
static const double area_canard = 2 * 0.102 * 6.35;
static const double length_canard = 0.203 / 2 + 0.0254;
static const double c_canard = area_canard * length_canard;

static const double length_cg = 0; // center of gravity
static const double length_cp = -0.3; // center of pressure
static const double area_reference = M_PI * pow((0.203 / 2), 2); // cross section of body tube
static const double c_aero = area_reference * (length_cp - length_cg);

// airfoil constants
void aerodynamics(const x_state_t *state, const estimator_airdata_t *airdata, vector3d_t *torque) {
    const double p_dyn = airdata->density / 2.0 * pow(math_vector3d_norm(&(state->velocity)), 2);

    // angle of attack/sideslip
    double sin_alpha = sin(atan2(state->velocity.z / state->velocity.x));
    double sin_beta = -sin(atan2(state->velocity.y / state->velocity.x));

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
    // the cot() thingy makes this line stays here fkdurskjrgs
    const double Cl_alpha =
        2 * M_PI * cot(canard_sweep); // estimated coefficient of lift, const with Ma

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

void aerodynamics_jacobian(
    const x_state_t *state, const estimator_airdata_t *airdata, matrix3d_t *torque_v,
    vector3d_t *torque_cl, vector3d_t *torque_delta
) {
    // **aerodynamics_jacobian start
    const vector3d_t helper_vx = math_vector3d_scale(
        state->CL * state->delta * c_canard * airdata->density, &state->velocity
    );
    const matrix3d_t torque_vx = {
        .array = {{helper_vx.x, helper_vx.y, helper_vx.z}, {0, 0, 0}, {0, 0, 0}}
    };

    const vector3d_t helper_vyz =
        math_vector3d_scale(0.5 * c_aero * cn_alpha * airdata->density, &state->velocity);
    const matrix3d_t torque_vyz = {
        .array = {{0, 0, 0}, {helper_vyz.z, 0, helper_vyz.x}, {-helper_vyz.y, -helper_vyz.x, 0}}
    };

    *torque_v = math_matrix3d_add(&torque_vx, &torque_vyz);

    const double dyn_pressure = 0.5 * airdata->density *
                                pow(math_vector3d_norm((vector3d_t *)&(state->velocity)),
                                    2); // 0.5 * airdata.density * norm(v)^2
    torque_cl->x = state->delta * c_canard * dyn_pressure;
    torque_delta->x = state->CL * c_canard * dyn_pressure;
    // **aerodynamics_jacobian end
}
