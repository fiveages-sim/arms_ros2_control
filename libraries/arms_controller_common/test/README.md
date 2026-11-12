# HOLD output regression tests (no hardware)

These tests use in-memory `LoanedCommandInterface` / `LoanedStateInterface`
handles. They do not construct a hardware plugin, controller manager, motor SDK,
MPC solver or robot model. `rclcpp` is used for the lifecycle node's parameters
and logger only. Run in an isolated ROS domain without device passthrough.

In a ROS 2 Jazzy workspace with this repository's dependencies installed:

```sh
colcon build --packages-select arms_controller_common --cmake-args -DBUILD_TESTING=ON
. install/setup.sh
colcon test --packages-select arms_controller_common --event-handlers console_direct+
colcon test-result --verbose
```

When using a standalone CMake install/overlay, ensure its `lib` directory is at
the front of `LD_LIBRARY_PATH`. Otherwise an existing install with the same
SONAME may be loaded instead of the library being tested. Check `ldd` on the
test executable. The test targets have a 30-second timeout.

`test_state_hold` exercises the real common `StateHold` implementation:

- nonzero prior velocity is overwritten on entry and on every update;
- the position reference stays fixed under 1000 cycles of drifting feedback;
- startup mismatch selection occurs once, in `enter()`, before output;
- repeated entry, runtime parameter changes and disabled thresholds;
- position-only and partially populated velocity interfaces;
- non-finite startup feedback does not replace a valid commanded reference;
- velocity clearing is independent of POSITION/MIX mode.

`test_hold_commands` exercises the common output primitive used by
`CtrlComponent::holdLastSentPositions()`, including removal of stale analytic
velocity/acceleration seeds. It does not instantiate an MPC solver or exercise
an entire controller's asynchronous state transition.

## Behavior change

`hold_first_check_position_threshold` retains its startup-selection meaning,
but selection now happens synchronously in `enter()`. If any valid measured
joint exceeds the positive threshold, valid measured positions are captured
once; invalid samples retain the previous commanded position. Otherwise the
last commanded positions are captured. A nonpositive value disables that
startup fallback.

`hold_position_threshold` is now a warning threshold only. Crossing it never
reanchors the HOLD reference. Leaving and entering HOLD again may select a new
reference. Gravity compensation and gain handling are unchanged.

Writing zero velocity feed-forward does not prove that physical motion has
stopped, nor does this patch add a controlled deceleration planner or hardware
stop acknowledgement. No hardware validation is claimed by these tests.
