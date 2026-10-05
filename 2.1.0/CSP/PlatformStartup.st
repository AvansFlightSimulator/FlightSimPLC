PROGRAM PlatformStartup
VAR
    initialized : BOOL;
    ppEnabled : BOOL;
    resetButtonEdge, startButtonEdge : R_TRIG;
    ppStartArmed : BOOL;
    ppState : stateFlightSimulator;
    allPPMode, allInTravel, allAtNeutral : BOOL;
    atNeutral : ARRAY[1..6] OF BOOL;
    i : INT;
END_VAR
(* IMPLEMENTATION *)
// Called only for the PP-startup/CSP-runtime policy. Pure PP main is unchanged.
// Ownership: PP (this task) until PPReleased; CSP (EtherCAT task) while
// CSP_Data.OwnsOutputs; PP again after CSP publishes CSPReleased on Reset.
resetButtonEdge(CLK:=GVL.resetButton OR GVL.resetHMI);
startButtonEdge(CLK:=GVL.startButton OR GVL.startHMI);
CSP_Data.Reset := GVL.resetButton OR GVL.resetHMI;
CSP_Data.StopRequested := GVL.stopButton OR GVL.stopHMI;

IF NOT initialized THEN
    GVL.FlightSimulator.init(
        axis1:=servo_drive_1, axis2:=servo_drive_2, axis3:=servo_drive_3,
        axis4:=servo_drive_4, axis5:=servo_drive_5, axis6:=servo_drive_6,
        lightRed:=GVL.lightRed, lightYellow:=GVL.lightYellow, lightGreen:=GVL.lightGreen,
        enabled:=ppEnabled, safetySystem:=GVL.safetySystem,
        newPositions:=GVL.servoPositions, newVelocities:=GVL.servoVelocities);
    initialized := TRUE;
END_IF

// Reset handback: CSP braked, restored mode 1 on all six drives and no longer
// writes them. Take ownership back and run the ORIGINAL PP reset from here.
IF CSP_Data.PPReleased AND CSP_Data.CSPReleased THEN
    CSP_Data.PPReleased := FALSE;
    ppEnabled := FALSE; ppStartArmed := FALSE;
    FOR i := 1 TO 6 DO CSP_Data.PPMoveDone[i] := FALSE; END_FOR
    // Without safety, FlightSimulator.cycle() immediately selects its error state.
    GVL.FlightSimulator.doReset();
END_IF

// PPReleased is published AFTER the final complete PP call. Once published,
// this task NEVER calls a PP motion block again until CSPReleased. Only the CSP
// task changes OwnsOutputs, at its cycle boundary, after capturing all six
// actual positions.
IF CSP_Data.PPReleased THEN
    IF NOT GVL.safetySystem OR CSP_Data.StopRequested OR CSP_Data.Reset OR CSP_Data.Fault THEN
        ppEnabled := FALSE;
    ELSIF startButtonEdge.Q AND CSP_Data.ReadyToStart THEN
        ppEnabled := TRUE;
    END_IF
    CSP_Data.Enable := ppEnabled AND GVL.safetySystem;
    GVL.lightRed := NOT GVL.safetySystem OR CSP_Data.Fault;
    GVL.lightGreen := NOT GVL.lightRed AND CSP_Data.Active AND CSP_Data.ReadyToStart;
    GVL.lightYellow := NOT GVL.lightRed AND NOT GVL.lightGreen;
    FOR i := 1 TO 6 DO
        GVL.currentPositions[i].Number := LREAL_TO_REAL(CSP_Data.Axis[i].ActualPosition);
        GVL.servoPositionsCurrent[i] := LREAL_TO_REAL(CSP_Data.Axis[i].ActualPosition);
        GVL.servoPositions[i] := LREAL_TO_REAL(CSP_Data.Axis[i].RequestedPosition);
        GVL.servoVelocities[i] := LREAL_TO_REAL(CSP_Data.Axis[i].CommandVelocity);
    END_FOR
    RETURN;
END_IF

// Original PP reset on the button edge. The request is not stored: a Reset
// pressed during an E-stop cannot cause motion when the E-stop is released.
IF resetButtonEdge.Q THEN
    ppEnabled := FALSE; ppStartArmed := FALSE;
    FOR i := 1 TO 6 DO CSP_Data.PPMoveDone[i] := FALSE; END_FOR
    GVL.FlightSimulator.doReset();
END_IF
ppState := GVL.FlightSimulator.PPState();
IF NOT GVL.safetySystem OR CSP_Data.StopRequested THEN
    ppEnabled := FALSE;
ELSIF startButtonEdge.Q AND ppState = stateFlightSimulator.stopped AND NOT CSP_Data.CSPReleased THEN
    ppEnabled := TRUE; ppStartArmed := TRUE;
    FOR i := 1 TO 6 DO CSP_Data.PPMoveDone[i] := FALSE; END_FOR
END_IF
// Never feed PC targets to PP during startup. The original startup method
// selects position 200 and velocity 25 itself; these arrays are only a hold.
FOR i := 1 TO 6 DO GVL.servoPositions[i] := 200; GVL.servoVelocities[i] := 25; END_FOR
GVL.FlightSimulator.cycle(StartupOnly:=TRUE);
ppState := GVL.FlightSimulator.PPState();
CSP_Data.AllPPMoveDone := TRUE;
allPPMode := TRUE; allInTravel := TRUE; allAtNeutral := TRUE;
FOR i := 1 TO 6 DO
    // Evaluated every cycle from PP's own durable state; nothing is latched.
    // STARTED is entered one cycle after goNeutralPosition() sets execute, so
    // a ready axis in STARTED has finished that neutral move (or was aborted;
    // the position check below rejects that case).
    atNeutral[i] := ABS(CSP_Data.Axis[i].ActualPosition - CSP_Config.HomePosition)
        <= CSP_Config.HomeTolerance;
    allAtNeutral := allAtNeutral AND atNeutral[i];
    CSP_Data.PPMoveDone[i] := ppStartArmed AND ppState = stateFlightSimulator.started
        AND GVL.FlightSimulator.axis[i].PPMoveDone() AND atNeutral[i];
    CSP_Data.AllPPMoveDone := CSP_Data.AllPPMoveDone AND CSP_Data.PPMoveDone[i];
    allPPMode := allPPMode AND CSP_Data.Axis[i].ModeActual = CSP_Config.CONTROL_MODE_PP;
    allInTravel := allInTravel AND CSP_Data.Axis[i].ActualPosition >= CSP_Config.MinimumPosition
        AND CSP_Data.Axis[i].ActualPosition <= CSP_Config.MaximumPosition;
    GVL.currentPositions[i].Number := GVL.FlightSimulator.axis[i].getPosition();
    GVL.servoPositionsCurrent[i] := GVL.currentPositions[i].Number;
END_FOR
CSP_Data.Fault := ppState = stateFlightSimulator.error OR GVL.FlightSimulator.inError();
CSP_Data.FaultCode := 0;
IF CSP_Data.Fault THEN CSP_Data.FaultCode := 9; END_IF // inspect original PP/axis error
CSP_Data.Homing := FALSE;
CSP_Data.ReadyToStart := FALSE;
CSP_Data.Enable := ppEnabled AND GVL.safetySystem;
CASE ppState OF
    stateFlightSimulator.error:
        CSP_Data.PlatformState := PlatformControlState.FAULT;
        CSP_Data.ResetRequired := TRUE; ppEnabled := FALSE;
    stateFlightSimulator.resetting:
        CSP_Data.PlatformState := PlatformControlState.PP_RESET;
        CSP_Data.ResetRequired := TRUE;
    stateFlightSimulator.stopped:
        CSP_Data.PlatformState := PlatformControlState.PP_STOPPED;
        CSP_Data.ResetRequired := FALSE;
ELSE
    CSP_Data.PlatformState := PlatformControlState.PP_STARTING;
END_CASE
// PP startup finished: Start accepted, PP in STARTED, every axis ready with its
// neutral move complete (PPMoveDone) and within HomeTolerance of HomePosition
// (= goNeutralPosition's 200), and measured standstill for 200 ms.
CSP_Data.PPStartupComplete := ppStartArmed AND ppEnabled AND GVL.safetySystem
    AND NOT CSP_Data.Fault AND ppState = stateFlightSimulator.started
    AND GVL.FlightSimulator.isReady() AND CSP_Data.AllPPMoveDone AND CSP_Data.AllStationary;
IF CSP_Data.Fault THEN CSP_Data.StartupBlockReason := 7;
ELSIF NOT GVL.safetySystem THEN CSP_Data.StartupBlockReason := 3;
ELSIF NOT ppEnabled OR NOT ppStartArmed THEN CSP_Data.StartupBlockReason := 4;
ELSIF ppState <> stateFlightSimulator.started THEN CSP_Data.StartupBlockReason := 10;
ELSIF NOT GVL.FlightSimulator.isReady() THEN CSP_Data.StartupBlockReason := 12;
ELSIF NOT allAtNeutral THEN CSP_Data.StartupBlockReason := 13;
ELSIF NOT CSP_Data.AllPPMoveDone THEN CSP_Data.StartupBlockReason := 14;
ELSIF NOT CSP_Data.AllStationary THEN CSP_Data.StartupBlockReason := 11;
ELSIF NOT CSP_Config.Commissioned THEN CSP_Data.StartupBlockReason := 1;
ELSIF NOT allPPMode THEN CSP_Data.StartupBlockReason := 6;
ELSIF NOT allInTravel THEN CSP_Data.StartupBlockReason := 2;
ELSE CSP_Data.StartupBlockReason := 0;
END_IF
// The CSP task repeats these checks race-free when it captures the positions.
IF CSP_Data.PPStartupComplete AND CSP_Config.Commissioned AND allPPMode AND allInTravel
    AND NOT CSP_Data.CSPReleased THEN
    CSP_Data.PlatformState := PlatformControlState.CSP_HANDOVER;
    // Last write: cannot be observed until all PP motion calls above finished.
    CSP_Data.PPReleased := TRUE;
END_IF
