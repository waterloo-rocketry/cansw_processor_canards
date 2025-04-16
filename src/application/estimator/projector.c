#include "projector.h"
#include "application/controller/controller.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include "estimator_types.h"
#include "model/model_airdata.h"
#include "model/quaternion.h"

// computes roll state and scheduling variables for controller
// Output: vector with (1:3) state, (4:5) flight conditions
controller_input_t projector(x_state_t *est_state) {
    controller_input_t output;
    // roll state:

    // decompose state vector
    quaternion_t q = est_state->attitude;
    vector3d_t w = est_state->rates;
    vector3d_t v = est_state->velocity;
    double alt = est_state->altitude;
    double Cl = est_state->CL;
    double delta = est_state->delta;

    // compute roll angle
    double phi = quaternion_to_roll(&q);

    // cat roll state
    output.roll_state.roll_angle = phi;
    output.roll_state.roll_rate = w.x;
    output.roll_state.canard_angle = delta;

    // scheduling variables

    // calculate air data
    double rho = model_airdata(alt).density;
    double airspeed = math_vector3d_norm(&v);
    double p_dyn = 0.5 * rho * airspeed * airspeed;

    // cat flight condition
    output.canard_coeff = Cl;
    output.pressure_dynamic = p_dyn;

    return output;
}