#include "projector.h"
#include "application/controller/controller.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include "estimator_types.h"
#include "model/model_airdata.h"
#include "model/quaternion.h"

controller_input_t projector(const x_state_t *state) {
    controller_input_t output = {0};

    // compute roll angle
    const double phi = quaternion_to_roll(&state->attitude);

    // cat roll state
    output.roll_state.roll_angle = phi;
    output.roll_state.roll_rate = state->rates.x;
    output.roll_state.canard_angle = state->delta;

    // scheduling variables
    // calculate air data
    const double rho = model_airdata(state->altitude).density;
    const double airspeed = math_vector3d_norm(&state->velocity);
    const double p_dyn = 0.5 * rho * airspeed * airspeed;

    // cat flight condition
    output.canard_coeff = state->CL;
    output.pressure_dynamic = p_dyn;

    return output;
}
