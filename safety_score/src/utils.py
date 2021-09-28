
import numpy as np

ACCEL_GRAVITY = 9.81
LATERAL_ACCEL_THRESHOLD = 0.4 * ACCEL_GRAVITY
LATERAL_ACCEL_BASELINE = 0.2 * ACCEL_GRAVITY
LONGITUDINAL_ACCEL_THRESHOLD = 0.3 * ACCEL_GRAVITY
LONGITUDINAL_ACCEL_BASELINE = 0.1 * ACCEL_GRAVITY
UNSAFE_FOLLOWING_THRESHOLD = 1.0
UNSAFE_FOLLOWING_BASELINE = 3.0

FCW_COEFFICIENT = 1.014495
HARD_BRAKING_COEFFICIENT = 1.127294
AGGRESSIVE_TURNING_COEFFICIENT = 1.019630
UNSAFE_FOLLOWING_COEFFICIENT = 1.001444
FORCED_DISENGAGEMENT_COEFFICIENT = 1.317958


def calc_pcf(fcw_value, hb_value, at_value, uf_value, fd_value):

    predicted_collision_frequency = (
        FCW_COEFFICIENT * fcw_value
        + HARD_BRAKING_COEFFICIENT * hb_value
        + AGGRESSIVE_TURNING_COEFFICIENT * at_value
        + UNSAFE_FOLLOWING_COEFFICIENT * uf_value
        + FORCED_DISENGAGEMENT_COEFFICIENT * fd_value
    )

    return predicted_collision_frequency


def calc_safety_score(pcf):

    safety_score_raw = 115.382324 - 22.526504 * pcf
    safety_score = np.clip(safety_score_raw, 0, 100)

    return safety_score
