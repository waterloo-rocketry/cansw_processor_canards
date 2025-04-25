#include "projector.h"
#include "application/controller/controller.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include "estimator_types.h"
#include "model/model_airdata.h"
#include "model/quaternion.h"

controller_input_t projector(const x_state_t *est_state) {
    controller_input_t output = {0};
    // roll state:

    // decompose state vector
    const quaternion_t q = est_state->attitude;
    const vector3d_t w = est_state->rates;
    const vector3d_t v = est_state->velocity;
    const double alt = est_state->altitude;
    const double Cl = est_state->CL;
    const double delta = est_state->delta;

    // compute roll angle
    const double phi = quaternion_to_roll(&q);

    // cat roll state
    output.roll_state.roll_angle = phi;
    output.roll_state.roll_rate = w.x;
    output.roll_state.canard_angle = delta;

    // scheduling variables

    // calculate air data
    const double rho = model_airdata(alt).density;
    const double airspeed = math_vector3d_norm(&v);
    const double p_dyn = 0.5 * rho * airspeed * airspeed;

    // cat flight condition
    output.canard_coeff = Cl;
    output.pressure_dynamic = p_dyn;

    return output;
}
