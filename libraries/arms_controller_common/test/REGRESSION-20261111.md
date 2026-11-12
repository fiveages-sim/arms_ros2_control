# HOLD no-device regression: 2026-11-11

## Versions and scope

- Upstream `main`: `4366e00b1143d795aaea94f73ea020bc497684d1`.
- Tested fix: `84b6a6679d922773fd0961561f6f1ef26f234d54` (the later results-document commit changes no code).
- ROS 2 Jazzy; standalone CMake build of the full `arms_controller_common` library and GTests.
- Locked base image: `sha256:56ac7597b42f9583b2257aba175129fd12b46c9acc95f4548c8ad142551c14eb`.
- Containers had no network or device passthrough, all capabilities dropped, ROS localhost-only.
- Tests use real common StateHold with memory loaned command/state interfaces. No hardware plugin, controller manager, motor SDK, MPC solver or lifecycle activation was run.
- The correct baseline/fixed library path was checked with `ldd` under the same `LD_LIBRARY_PATH` used for CTest. Each CTest target has a 30-second timeout; the complete test command has an external 80-second timeout.

## Results

| Test | Upstream baseline | Patched |
|---|---|---|
| `ClearsAllVelocityCommandsOnEntry` | FAIL | PASS |
| `ClearsNewStaleVelocityEveryCycle` | FAIL | PASS |
| `KeepsCapturedTargetWhileFeedbackDrifts` | FAIL | PASS |
| `StartupFallbackIsResolvedBeforeFirstOutputAndFrozen` | FAIL | PASS |
| `DisabledThresholdsNeverReanchor` | FAIL | PASS |
| `ChangingThresholdsDuringHoldDoesNotReanchor` | FAIL | PASS |
| `ReentrySelectsNewTargetOnce` | FAIL | PASS |
| `PositionOnlyHardwareNeedsNoVelocityInterfaces` | FAIL | PASS |
| `PartialVelocityInterfacesAreCleared` | FAIL | PASS |
| `InvalidMeasuredJointDoesNotPoisonStartupFallback` | FAIL | PASS |
| `MixedModeStillClearsVelocityWithoutGravityObject` | FAIL | PASS |
| `PositionModeClearsVelocityInterfacesIfPresent` | FAIL | PASS |
| `WaitingForPolicyClearsVelocityAndDerivativeSeed` | N/A (new helper) | PASS |
| `ClearsEveryVelocityIncludingUnmatchedOptionalChannels` | N/A (new helper) | PASS |

The 12 shared test cases fail on the upstream baseline and pass with this patch. They exercise the expected HOLD contract across two core defects and boundary cases; they are not 12 independent bug discoveries. Two new-helper tests pass, for 14 passing tests in total. The baseline cannot run the helper tests because that API does not exist there.

The drift test injects changing measured positions for 1000 calls with a 2 ms period argument. The baseline captured reference moves by up to 5.85 rad; the patched reference stays unchanged (strict zero-drift assertion). This is a synthetic stress input, not physical motion or a real-time 500 Hz measurement. Entry uses alternating nonzero velocity seeds; the repeated-update test overwrites all velocity fields with 3.482456 rad/s before each of 100 calls. The patched output is zero after entry and each update.

The additional helper test verifies that stale analytic velocity/acceleration seeds are also cleared. It covers the shared primitive called by CtrlComponent::holdLastSentPositions(), not a complete asynchronous MPC transition.

| Artifact | Baseline | Patched |
|---|---|---|
| Passing / total GTests | 0 / 12 | 14 / 14 |
| CTest command exit code | 8 (expected assertion failures) | 0 |
| Common library SHA256 | `0d6d4b3515feef46c66cf1412c0e9f88b753c038cf12b6472c1313a391a8f383` | `925e1d16ca56ff3e3d8a4c29623700e387509c394208831da4eccf837358cf37` |
| Recorded run ID | `20261111T025320Z-fiveages-MS-7E06-6874f20be2` | `20261111T025414Z-fiveages-MS-7E06-03b438032c` |

## Reproduction and limits

See [README.md](README.md) for normal `colcon build` / `colcon test` instructions and the intentional threshold behavior change. Tests and assertions are part of this PR.

Zero velocity feed-forward is not a hardware stop acknowledgement or proof of physical rest. This patch does not add a deceleration planner or change gravity compensation, gains, MPC costs, communication scheduling or joint limits. No hardware test, full latest-workspace build, or end-to-end controller exit test is claimed by this regression. The earlier full-plugin build on main f41054c is separate historical evidence, not a build of this updated main.
