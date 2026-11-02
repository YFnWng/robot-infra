"""Read-only firmware encoder calibration and physical/logical coordinates."""
import numpy as np

DEGREES_PER_COUNT = 0.045
UNITS_PER_SECOND_PER_RPM = np.asarray([
    (67.319841/20)/60, (.375/5)/60*360, -1.190625/60,
    (67.319841/20)/60, (.375/5)/60*360, (.375/5)/60*360,
], dtype=np.float64)
UNITS_PER_COUNT = np.asarray([
    (67.319841/20)/360, .375/5, -1.190625/360,
    (67.319841/20)/360, .375/5, .375/5,
], dtype=np.float64)*DEGREES_PER_COUNT


def motor_to_logical(value):
    result = np.asarray(value, dtype=np.float64).copy()
    result[..., 0] += result[..., 2]
    result[..., 5] -= result[..., 4]
    return result


def logical_to_motor(value):
    result = np.asarray(value, dtype=np.float64).copy()
    result[..., 0] -= result[..., 2]
    result[..., 5] += result[..., 4]
    return result


def counts_to_logical(counts):
    return motor_to_logical(np.asarray(counts, dtype=np.float64)*UNITS_PER_COUNT)
