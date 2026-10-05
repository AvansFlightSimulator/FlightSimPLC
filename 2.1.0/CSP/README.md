# FlightSim CSP commissioning

This source accompanies the existing 2.1.0 CODESYS project. It adds direct
CiA402 CSP without SoftMotion. PP remains the default selection. No program has
been downloaded to the PLC by the integration scripts.

## Configuration

Edit `CSP_Config` in CODESYS (the corresponding readable source is
`CSP_Config.st`). The C++ selection is in FlightSim's `BuildMode.h`.

| Setting | Initial value |
| --- | --- |
| PLC `ControlMode` / C++ `FLIGHTSIM_PLC_CONTROL_MODE` | 1 = PP; select 8 for CSP on both |
| `Commissioned` | FALSE until the checks below are complete |
| Cycle | 0.004 s / EtherCAT task 4 ms |
| Travel | 10..390 mm |
| Maximum velocity | 25 mm/s |
| Maximum acceleration | 50 mm/s² |
| Braking acceleration | 50 mm/s², no greater than maximum acceleration |
| Maximum jerk | 250 mm/s³ |
| Communication timeout | 500 ms |
| Transition timeout | 10 s |
| Standstill indication | measured speed <= 0.05 mm/s for 200 ms |
| C++ `MSFS_FILTER_TIME_CONSTANT_MS` | 120 ms |

These are conservative commissioning values, not validated loaded-platform
tuning. Mode selection is a build/configuration choice: stop the machine before
changing it and rebuilding/downloading. This is not a live HMI mode switch.

The C++ CSP path retains the existing geometry and pre-IK exponential filter,
but sends fractional final positions without feedback-relative steps or rounded
millimetres. Manual actuator positions also retain fractions in CSP. PP and
Unity retain their prior rounding and step/speed behavior.

## Trajectory behavior

`CSP_Trajectory6` is an online (receding-horizon) generator. Every 4 ms CSP cycle
each axis chooses one jerk value from its current commanded position, velocity
and acceleration and integrates it exactly, so P, V and A are always continuous.
A new target only changes the next choice: there is no stop-at-target plan to
finish and no separate brake-to-standstill path for ordinary updates. The jerk
chosen is the one whose next state, braked at full limits (`CSP_StopOffset`),
comes to rest at the target, which is a time-optimal approach that decelerates only
as late as the limits require. It is clamped by MaximumJerk, the smaller of
MaximumAcceleration/BrakingAcceleration, a jerk-aware MaximumVelocity envelope,
and a travel envelope: from every commanded state a full-limit stop ends inside
MinimumPosition..MaximumPosition. Targets are limited to travel; a commanded
position outside travel (not expected) raises FaultCode 7.

The 20 Hz PC targets are linearly interpolated over the measured packet interval
(at most 100 ms), starting from the current interpolated target. This turns the
packet staircase into a continuous target and adds about one packet period of
delay. Targets are tracked by position only; feeding the packet slope forward
reduced lag but amplified packet jitter in simulation.

Reversals need no special path: when a new target lies behind the stopping point,
the axis decelerates at the limits, passes through zero velocity and accelerates
toward the target, with no dwell. `CSP_Data.ReversalDeferred` (trajectory
`Deferred`) is TRUE while any axis must pass its target before reversing, or
while the target moves faster than the limits allow.
Axes are now planned independently rather than with one common duration, so
during strongly limited transients the six actuators do not stay proportionally
synchronized. With small 20 Hz updates this is not expected to be visible.

Operator Stop and communication loss use the same generator with the target set
to the current full-limit stopping point, so braking stays continuous and ends in
a stationary hold. Drive faults, loss of fieldbus readiness or the safety input
still request CiA402 quick stop. `CSP_Brake` and `CSP_Bezier` are no longer called.

Offline, a line-for-line port was simulated: limits held exactly, a steady
8 mm/s target was tracked at 7.99..8.01 mm/s (7.78..8.23 with +/-15 ms packet
jitter), and Stop settled with Busy cleared. With the commissioning limits
(25 mm/s, 50 mm/s^2, 250 mm/s^3) the command lags a moving target by about the
stopping distance at that speed, for example about 2 mm at 10 mm/s. Check the
EtherCAT task execution time: the worst case is about 380 short stop-distance
evaluations per cycle.

Only a complete valid CSP frame refreshes the watchdog. Fragmented frames are
assembled; multiple newline-separated frames are consumed in order; oversized,
malformed, wrong-mode and incomplete frames are rejected. The protocol is the
compact JSON emitted by this C++ build: numeric controlMode=8, six positions,
six zero speeds, newline. It deliberately does not accept reordered fields or
arbitrary third-party JSON. A seqlock transfers the complete six-axis target to
the higher-priority EtherCAT task. Network parsing and feedback assembly stay
in the existing slower task.

## CODESYS / EtherCAT checks before enabling CSP

The project uses the supplied **FlightSim-CMMT-AS.devdesc.xml** profile from
`CSP/device`. On another engineering PC, install it through Device Repository
with its accompanying files. It preserves the Festo hardware identity, PDOs and
libraries, and removes the implicit `BeforeWriteOutputs` callback. The final
`CSP_WriteOutputs` call in the EtherCAT task is now the sole writer: it calls
the original Festo method in PP, or writes the direct PDOs in CSP. The original
vendor device profile remains installed unchanged. Do not replace the project
profile with the original PP profile while CSP is enabled: that would restore
a competing implicit writer.

1. Preserve the existing drive position reference and Festo factor group. Check
   that its user unit is millimetres on all six drives. `CSP_ReadAxis` uses the
   existing `rPositionFactor` and verifies raw position multiplied by that factor
   against the Festo actual-position property. A mismatch prevents entry; do
   not substitute an assumed counts/mm value. Confirm signs and zero/reference.
2. Verify RxPDO 6040:00 WORD, 6060:00 SINT, 607A:00 DINT and TxPDO 6041:00 WORD,
   6061:00 SINT, 6064:00 DINT, 606C:00 DINT on every drive. These already exist
   in the supplied PP mapping. Add **60F4:00 DINT** to each drive's TxPDO and map
   it to `CSP_Data.FollowingErrorRaw[1]` through `[6]`. The original project did
   not map following error cyclically. Preserve the other PP PDO entries.
3. Verify that EtherCAT's bus-cycle task is `EtherCAT_Task`, that all six drives
   use DC Sync0 at 4 ms, and that CMMT interpolation period 60C2:01/02 matches
   4 ms (value 4 with time exponent -3, subject to the installed firmware's
   supported settings). Check EtherCAT OP/DC lock and task execution time/jitter.
   A 2 ms deployment requires changing **all** these periods together with
   `CycleSeconds=0.002`; do not change only the trajectory's timestep.
4. Confirm CMMT following-error supervision, bus watchdog response, PP halt
   deceleration, quick-stop response and holding-brake behavior for the loaded
   vertical platform. CSP entry is now an automatic handover from the working
   PP startup (see "PP startup, CSP runtime" below); the drives stay enabled
   and are not disabled/re-enabled for the mode change.
5. Select CSP on both ends, verify the mappings and low-amplitude movement with
   the machine's established commissioning procedure, then set `Commissioned`
   TRUE. Record RequestedPosition, CommandPosition, ActualPosition, velocity,
   acceleration, following error, mode display and state. Software checks and
   offline builds do not validate real mechanical operation.

`CSP_Data.Axis[1..6]` exposes the requested diagnostics. The feedback packet also
records them. The C++ graph page shows requested/command/actual position,
command/actual velocity, command acceleration and raw drive following error
for CSP sessions. PP recordings remain readable.

The PC's existing sender retransmits its cached target while connected. The
PLC watchdog detects missing valid network frames, not stale MSFS telemetry.

## October 5 startup repair

The owner's active project is FlightSim/build/csp-inspection/final.project.
It already selected mode 8 and Commissioned TRUE; those choices and the edited
operator controls are preserved. Source-template commissioning defaults above
are not the active project's settings.

The output conversion is now the inverse of readback: raw feedback multiplied
by rPositionFactor gives user units; command user units divided by that same
factor gives 0x607A. The former multiply in both directions could command a
large jump on the first active trajectory cycle. No physical scale is changed.
Master and all six slave DC cycle values are now 4000 microseconds, matching
the existing 4 ms task and trajectory timestep. Drive-side 60C2 and physical
OP/DC lock still need verification on hardware.

Watch `CSP_Data.StartupBlockReason`: 0 transitioning/running, 1 commissioning
not enabled, 2 axis not ready, 3 safety input open, 4 waiting for Start,
5 waiting for a valid fresh PC target, 6 waiting for mode 8, 7 fault.
For reason 2, inspect `CSP_Data.Axis[1..6].ReadinessCode`: 0 ready,
1 missing PDO pointer, 2 Festo SDO initialization incomplete, 3 invalid factor,
4 EtherCAT connector not ready, 5 raw/user-unit conversion mismatch.
The existing `TransitionState`, `FaultCode`, `Statusword` and `ModeActual`
remain available. FaultCode 5 means the measured starting position is outside
configured travel; this must be resolved through the established reference/
commissioning procedure, not by bypassing the travel check.

The original project is retained as final.before-startup-fix.project beside
final.project. No PLC download or live motion validation was performed.

## Transition timeout reset correction

FaultCode 1 / TransitionState 20 was reported on hardware. The transition TON
now resets while a fault is latched, on an accepted Reset, and between transition
stages. It no longer carries an expired timer into the recovery attempt. The
first fault cause remains latched rather than being overwritten by later safety
or readiness consequences. Safety, standstill and operator-reset gates remain.

TimeoutState, TimeoutStatusword[1..6] and TimeoutControlword[1..6] retain the
values at the actual timeout, before fault handling requests quick stop.
If state 20 times out again after this correction, record these fields: a live
statusword observed after the fault only describes the subsequent stop state.
This fixes timeout recovery; it does not establish why a drive originally failed
to complete the transition. The pre-repair project is final.before-timeout-fix.project.

## Required Reset -> 200 -> Start sequence

Superseded for ControlMode 8 by "PP startup, CSP runtime" below. The CSP park
move and CSP states 0..40 are retained in source but no longer entered.

On PLC startup, `CSP_Data.ResetRequired=TRUE` and `ReadyToStart=FALSE`.
Press Reset with the safety circuit healthy and all six axes ready. The PLC
enters CSP without an initial target jump, then moves all six axes to 200 using
the existing synchronized jerk-limited planner. Reset during normal motion first
brakes the current trajectory before planning the park move. PC targets are
ignored throughout parking; no PC connection is required for this local move.
This is positioning against the existing encoder reference, not CiA402 mode-6
reference homing or a change of encoder zero.

Completion requires the command trajectory to finish and all six actual
positions to be within HomeTolerance, at standstill, for HomeSettleTime.
The drives then hold the final command at 200. Only a new Start button edge
after completion enables incoming PC targets. Holding Start during parking
does not queue a restart. Stop during parking cancels it, brakes smoothly,
and requires another Reset. Faults and safety loss also revoke park readiness.
Normal Stop after successful parking retains readiness and allows a new Start.

CSP_Config defaults added: HomePosition=200, HomeTolerance=0.5 user units,
HomeSettleTime=T#200MS and HomeTimeout=T#60S. Parking uses the existing conservative
velocity, acceleration and jerk limits. FaultCode 8 indicates parking did not
complete in time. StartupBlockReason 8 means Reset required, 9 means parking.
Existing travel, scale and readiness checks still apply: an encoder position
outside configured travel cannot be corrected by bypassing those checks.

Watch ResetRequired, Homing and ReadyToStart. They are also recorded in CSP
feedback. The PLC green light is gated on successful parking; yellow covers
reset-required/parking, red retains the fault/safety indication. PP behavior is
unchanged. The pre-change project is final.before-home-sequence.project.
Offline builds do not validate the physical move or its tolerance under load.

## PP startup, CSP runtime (ControlMode 8)

Owner decision: PP for startup and Reset, CSP only for real-time motion. Exactly
one owner writes the six drives; `CSP_WriteOutputs` is still the only PDO writer
and selects the Festo PP writer or the direct CSP writer by `CSP_Data.OwnsOutputs`.

1. **PP (main task, `PlatformStartup`).** Reset edge calls the original
   `FlightSimulator.doReset()` (MC_Reset_Festo on faulted axes, then the original
   home move and STOPPED). A Reset pressed while the safety input is open is not
   stored. Start in STOPPED enables PP; the original `goNeutralPosition()` moves
   all six axes to 200 at velocity 25. In STARTED the original PC-target
   `doMovement` is suppressed (`cycle(StartupOnly:=TRUE)`), so no PC data moves PP.
2. **PP startup complete** (`CSP_Data.PPStartupComplete`): Start accepted, PP state
   STARTED, every axis `PPMoveDone` (PP's own `isReady()`: drive Standstill and
   `execute` cleared by Axis.cycle after MC_MoveAbsolute Done; no axis/move error)
   and within HomeTolerance of HomePosition 200, no PP fault, safety OK and
   `AllStationary` (|v| <= 0.05 for 200 ms). Evaluated every cycle; the one-cycle
   Done output is not latched. Handover also requires Commissioned, all
   ModeActual = 1 and positions within travel. `StartupBlockReason`: 10 not
   STARTED, 12 not ready, 13 not at neutral, 14 move error/abort, 11 not
   stationary, 1/6/2 commissioning/mode/travel.
3. **Release.** PP publishes `PPReleased` as its last write and never calls a PP
   motion block again while released.
4. **Capture (EtherCAT task, same cycle as ownership).** All six `ActualRaw` are
   copied to `InitialTargetRaw` and `TargetRaw`, ModeRequested = 8, controlword
   0x000F, then `OwnsOutputs := TRUE`. A failed check gives FaultCode 10.
5. **TransitionState 60.** Hold `InitialTargetRaw` (not live encoder values) until
   all six ModeActual = 8. `ModeBlockingAxis` names the first axis still waiting.
   No axis moves; leaving operation enabled or drifting > HomeTolerance faults (10);
   no confirmation within TransitionTimeout gives FaultCode 1, TimeoutState 60.
6. **61 -> 50.** Trajectory is initialized from the captured positions,
   `ReadyToStart`/CSP_READY is set, and only packets accepted after that point can
   move the platform (Enable from Start, fresh PC target). States 10/20 are not used.

**Reset after handover** (healthy or faulted): TransitionState 70 brakes an
active trajectory to rest, 71 holds position and requests mode 1 on all six
drives. After all six display 1, CSP publishes `CSPReleased` and clears
`OwnsOutputs`; PP clears `PPReleased` and runs its original `doReset()`. CSP no
longer pulses a CiA402 fault reset itself. The fault code that Reset acknowledged
is kept in `LastCSPFaultCode`. If mode 1 is not confirmed, FaultCode 1 with
TimeoutState 71 and `ModeBlockingAxis` remain; the CSP quick-stop fault handling
is otherwise unchanged. Stop during CSP still brakes and holds in CSP; Start
resumes.

PlatformState values: PP_RESET, PP_STOPPED, PP_STARTING, CSP_HANDOVER, CSP_READY,
CSP_RUNNING, FAULT, PP_HANDBACK. Offline compilation only; hardware must still
confirm that the CMMT accepts the in-operation 1 -> 8 and 8 -> 1 mode changes,
and that the Festo PP blocks resume cleanly after CSP owned the drives.
