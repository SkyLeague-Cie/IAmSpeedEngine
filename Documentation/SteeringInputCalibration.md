# Monotone steering input candidate

This candidate derives from b21dc21 (partial calibration db6dda3 plus the
low-slip settling guard), above the release base18ee51a. It does not alter
the release branch or the low-slip guard.

The commanded wheel-angle fraction is `input * SteeringInputCalibrationScale(input)`.
The former last segment made that product decrease above23/28 input:75%
commanded0.75 but full input commanded0.72. A compiled2049-point probe of the
identical db6/b21 helper recorded183 decreasing positive intervals.

Keep the existing segments through75% unchanged, including quarter-input
output0.275 and half-input output0.54. Above75%, return scale1. This retains
continuity, odd symmetry, bounded output and the configured full-steer angle.
Both tire friction and drive use the same helper. No new coefficient, state,
direction exception or speed curve is introduced.

This is an input-law repair candidate, not acceptance of the combined vehicle
response. The inherited low-slip behavior must be evaluated separately.
Steering/release yaw, forward/lateral speed, radius, support, historical Golds
and repeatability remain required before promotion.
