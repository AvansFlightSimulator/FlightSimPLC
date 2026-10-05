PROGRAM CSP_Cycle
VAR
    trajectory : CSP_Trajectory6;
    actual, requested, receivedCopy : ARRAY[1..6] OF LREAL;
    generation, generationAfter, acceptedGeneration, tick : UDINT;
    packetMode : SINT;
    packetTick, receivedTickCopy : UDINT;
    newTarget, timedOut, allReady, allStopped, allDisabled, allMode, allEnabled, allSwitchedOn, allPPMode : BOOL;
    resetEdge : R_TRIG;
    resetRequest : BOOL;
    resetConditionsReady, anyDriveFaultReaction : BOOL;
    standstill : TON;
    transition : TON;
    previousTransitionState : UINT;
    homePhase : UINT; // 1 brake, 2 plan park move, 3 wait for measured arrival
    homeTarget : ARRAY[1..6] OF LREAL;
    homeSettled, homeTimer : TON;
    homeAtPosition : BOOL;
    handoverValid : BOOL;
    initialPosition : ARRAY[1..6] OF LREAL; // captured at the PP -> CSP ownership switch
    holdRaw : ARRAY[1..6] OF DINT; // stationary target while restoring PP mode
    expectedMode : SINT;
    i : INT;
    stateWord : WORD;
    distance : LREAL;
END_VAR
(* IMPLEMENTATION *)
IF CSP_Config.ControlMode <> CSP_Config.CONTROL_MODE_CSP THEN RETURN; END_IF
CSP_Data.CycleCounter := CSP_Data.CycleCounter+1;
CSP_ReadAxis(Axis:=servo_drive_1,PlatformAxis:=1);
CSP_ReadAxis(Axis:=servo_drive_2,PlatformAxis:=2);
CSP_ReadAxis(Axis:=servo_drive_3,PlatformAxis:=3);
CSP_ReadAxis(Axis:=servo_drive_4,PlatformAxis:=4);
CSP_ReadAxis(Axis:=servo_drive_5,PlatformAxis:=5);
CSP_ReadAxis(Axis:=servo_drive_6,PlatformAxis:=6);
tick := TIME_TO_UDINT(TIME());
resetEdge(CLK:=CSP_Data.Reset);
IF resetEdge.Q THEN
    resetRequest := TRUE;
END_IF;
newTarget := FALSE;
generation := CSP_Data.Generation;
IF generation MOD 2 = 0 AND generation <> acceptedGeneration THEN
    receivedCopy := CSP_Data.ReceivedPosition;
    packetMode := CSP_Data.ReceivedMode; receivedTickCopy := CSP_Data.ReceivedTick;
    generationAfter := CSP_Data.Generation;
    IF generation = generationAfter AND packetMode = 8 THEN
        acceptedGeneration := generation; newTarget := TRUE;
        requested := receivedCopy; packetTick := receivedTickCopy;

    END_IF
END_IF
timedOut := NOT CSP_Data.Connected OR acceptedGeneration = 0
    OR (tick - packetTick) > TIME_TO_UDINT(CSP_Config.CommunicationTimeout);
allReady := TRUE; allStopped := TRUE; allDisabled := TRUE; allMode := TRUE; allEnabled := TRUE; allSwitchedOn := TRUE;
allPPMode := TRUE;
FOR i := 1 TO 6 DO
    stateWord := CSP_Data.Axis[i].Statusword AND 16#006F;
    CSP_Data.Axis[i].CiA402State := stateWord;
    actual[i] := CSP_Data.Axis[i].ActualPosition;
    allReady := allReady AND CSP_Data.Axis[i].Ready;
    allStopped := allStopped AND ABS(CSP_Data.Axis[i].ActualVelocity) <= CSP_Config.StandstillVelocity;
    allDisabled := allDisabled AND (stateWord = 16#0023 OR stateWord = 16#0021 OR (stateWord AND 16#004F) = 16#0040);
    allMode := allMode AND CSP_Data.Axis[i].ModeActual = 8;
    allPPMode := allPPMode AND CSP_Data.Axis[i].ModeActual = CSP_Config.CONTROL_MODE_PP;
    allEnabled := allEnabled AND stateWord = 16#0027;
    allSwitchedOn := allSwitchedOn AND (stateWord = 16#0023 OR stateWord = 16#0027);
    CSP_Data.Axis[i].CommunicationTimeout := timedOut;
    CSP_Data.Axis[i].PositionLimit := CSP_Data.Axis[i].RequestedPosition < CSP_Config.MinimumPosition
        OR CSP_Data.Axis[i].RequestedPosition > CSP_Config.MaximumPosition;
END_FOR
// Read-only monitoring runs during PP startup; no CSP output ownership yet.
standstill(IN:=allReady AND allStopped, PT:=T#200MS);
CSP_Data.AllStationary := standstill.Q;
CSP_Data.AllCSPModeConfirmed := allMode;
// Mode the drives must display for the current owner; first mismatch is exposed.
IF CSP_Data.OwnsOutputs AND CSP_Data.TransitionState < 70 THEN expectedMode := 8;
ELSE expectedMode := CSP_Config.CONTROL_MODE_PP; END_IF
CSP_Data.ModeBlockingAxis := 0;
FOR i := 6 TO 1 BY -1 DO
    IF CSP_Data.Axis[i].ModeActual <> expectedMode THEN CSP_Data.ModeBlockingAxis := i; END_IF
END_FOR
// PP acknowledged the Reset handback by withdrawing PPReleased.
IF CSP_Data.CSPReleased AND NOT CSP_Data.PPReleased THEN CSP_Data.CSPReleased := FALSE; END_IF
IF NOT CSP_Data.OwnsOutputs THEN
    // PP owns Reset while it owns the drives; do not replay its edges later.
    IF NOT CSP_Data.PPReleased THEN resetRequest := FALSE; END_IF
    IF CSP_Data.PPReleased AND NOT CSP_Data.CSPReleased THEN
        // The slower PP task published this only AFTER its last motion call.
        // Capture all axes before atomically selecting the sole CSP writer.
        handoverValid := CSP_Config.CycleSeconds > 0 AND CSP_Config.MaximumVelocity > 0
            AND CSP_Config.MaximumAcceleration > 0 AND CSP_Config.BrakingAcceleration > 0
            AND CSP_Config.BrakingAcceleration <= CSP_Config.MaximumAcceleration
            AND CSP_Config.MaximumJerk > 0 AND CSP_Config.MinimumPosition < CSP_Config.MaximumPosition;
        FOR i := 1 TO 6 DO
            handoverValid := handoverValid AND actual[i] >= CSP_Config.MinimumPosition
                AND actual[i] <= CSP_Config.MaximumPosition AND CSP_Data.Axis[i].ModeActual = 1;
            // Bumpless transfer: the first CSP target is the measured raw position.
            CSP_Data.InitialTargetRaw[i] := CSP_Data.Axis[i].ActualRaw;
            initialPosition[i] := actual[i]; // = ActualRaw * PositionFactor
            CSP_Data.Axis[i].TargetRaw := CSP_Data.InitialTargetRaw[i];
            CSP_Data.Axis[i].RequestedPosition := initialPosition[i];
            CSP_Data.Axis[i].CommandPosition := initialPosition[i];
            CSP_Data.Axis[i].CommandVelocity := 0;
            CSP_Data.Axis[i].CommandAcceleration := 0;
            CSP_Data.Axis[i].ModeRequested := 8;
            CSP_Data.Axis[i].Controlword := 16#000F;
        END_FOR
        CSP_Data.Active := FALSE; CSP_Data.ReadyToStart := FALSE;
        CSP_Data.Homing := FALSE; homePhase := 0;
        // A Reset edge seen in this capture window is kept: it returns to PP.
        CSP_Data.TransitionState := 60;
        previousTransitionState := 0;
        transition(IN:=FALSE, PT:=CSP_Config.TransitionTimeout);
        CSP_Data.PlatformState := PlatformControlState.CSP_HANDOVER;
        // Stop/Reset here are not faults: CSP holds, and Reset hands back to PP.
        IF NOT allReady OR NOT standstill.Q OR NOT allEnabled OR NOT GVL.safetySystem
            OR NOT CSP_Config.Commissioned OR NOT handoverValid OR CSP_Data.Fault THEN
            CSP_Data.Fault := TRUE; CSP_Data.FaultCode := 10;
            CSP_Data.PlatformState := PlatformControlState.FAULT;
            FOR i := 1 TO 6 DO
                CSP_Data.Axis[i].Controlword := 16#0002;
                CSP_Data.Axis[i].ModeRequested := CSP_Data.Axis[i].ModeActual;
            END_FOR
        END_IF
        CSP_Data.OwnsOutputs := TRUE; // sole ownership switch, in EtherCAT task
    END_IF
    RETURN;
END_IF
homeAtPosition := TRUE;
FOR i := 1 TO 6 DO
    homeTarget[i] := CSP_Config.HomePosition;
    homeAtPosition := homeAtPosition AND ABS(actual[i]-CSP_Config.HomePosition) <= CSP_Config.HomeTolerance;
END_FOR
homeSettled(IN:=CSP_Data.Homing AND homePhase = 3 AND NOT trajectory.Busy
    AND homeAtPosition AND allReady AND allStopped, PT:=CSP_Config.HomeSettleTime);
homeTimer(IN:=CSP_Data.Homing AND NOT CSP_Data.Fault, PT:=CSP_Config.HomeTimeout);
IF homeTimer.Q AND NOT CSP_Data.Fault THEN
    CSP_Data.Fault := TRUE; CSP_Data.FaultCode := 8;
END_IF
CSP_Data.StartupBlockReason := 0;
IF CSP_Data.Fault THEN CSP_Data.StartupBlockReason := 7;
ELSIF NOT CSP_Config.Commissioned THEN CSP_Data.StartupBlockReason := 1;
ELSIF NOT allReady THEN CSP_Data.StartupBlockReason := 2;
ELSIF NOT GVL.safetySystem THEN CSP_Data.StartupBlockReason := 3;
ELSIF CSP_Data.Homing THEN CSP_Data.StartupBlockReason := 9;
ELSIF CSP_Data.ResetRequired THEN CSP_Data.StartupBlockReason := 8;
ELSIF NOT CSP_Data.Enable THEN CSP_Data.StartupBlockReason := 4;
ELSIF timedOut THEN CSP_Data.StartupBlockReason := 5;
ELSIF NOT allMode THEN CSP_Data.StartupBlockReason := 6;
END_IF
// Each transition gets a fresh timeout. A latched fault must stop/reset
// the TON; otherwise Reset -> state 20 immediately faults again on old Q.
transition(IN:=NOT CSP_Data.Fault
    AND CSP_Data.TransitionState = previousTransitionState
    AND ((CSP_Data.TransitionState > 0 AND CSP_Data.TransitionState < 40)
        OR (CSP_Data.TransitionState = 40 AND CSP_Data.Homing)
        OR CSP_Data.TransitionState = 60 OR CSP_Data.TransitionState >= 70),
    PT:=CSP_Config.TransitionTimeout);
previousTransitionState := CSP_Data.TransitionState;
IF transition.Q AND NOT CSP_Data.Fault THEN
    CSP_Data.TimeoutState := CSP_Data.TransitionState;
    FOR i := 1 TO 6 DO
        CSP_Data.TimeoutStatusword[i] := CSP_Data.Axis[i].Statusword;
        CSP_Data.TimeoutControlword[i] := CSP_Data.Axis[i].Controlword;
    END_FOR
    CSP_Data.Fault := TRUE; CSP_Data.FaultCode := 1;
END_IF
// Keep the first cause latched; later consequences must not overwrite it.
IF NOT CSP_Data.Fault AND CSP_Data.OwnsOutputs AND (NOT allReady OR NOT GVL.safetySystem) THEN
    CSP_Data.Fault := TRUE; CSP_Data.FaultCode := 2;
END_IF
IF CSP_Data.Fault THEN
    CSP_Data.PlatformState := PlatformControlState.FAULT;
    CSP_Data.Active := FALSE;
    CSP_Data.ResetRequired := TRUE; CSP_Data.ReadyToStart := FALSE;
    CSP_Data.Homing := FALSE; homePhase := 0;
    anyDriveFaultReaction := FALSE;
    FOR i := 1 TO 6 DO
        CSP_Data.Axis[i].Controlword := 16#0002;
        anyDriveFaultReaction := anyDriveFaultReaction
            OR (CSP_Data.Axis[i].Statusword AND 16#004F) = 16#000F;
    END_FOR
    resetConditionsReady := standstill.Q AND GVL.safetySystem AND allReady
        AND NOT CSP_Data.Enable AND NOT CSP_Data.StopRequested AND CSP_Config.Commissioned;
    // Reset returns the drives to the original PP reset (MC_Reset_Festo etc.).
    // CSP issues no CiA402 fault-reset pulse itself; a drive in Fault stays
    // quick-stopped/disabled while its mode display is restored to 1.
    IF resetRequest AND resetConditionsReady AND NOT anyDriveFaultReaction THEN
        transition(IN:=FALSE, PT:=CSP_Config.TransitionTimeout);
        previousTransitionState := 0;
        CSP_Data.LastCSPFaultCode := CSP_Data.FaultCode;
        CSP_Data.Fault := FALSE; CSP_Data.FaultCode := 0;
        CSP_Data.TransitionState := 70;
        CSP_Data.PlatformState := PlatformControlState.PP_HANDBACK;
        resetRequest := FALSE;
    END_IF
    RETURN;
END_IF
// Reset belongs to the original PP state machine. A Reset during CSP handover
// or operation first brakes the existing command continuously (state 70), then
// restores mode 1 (state 71) and releases the drives to PP. States 0..40 and
// the CSP park move are not entered under this PP-startup policy.
IF resetRequest AND CSP_Data.TransitionState >= 50 AND CSP_Data.TransitionState < 70 THEN
    CSP_Data.ResetRequired := TRUE; CSP_Data.ReadyToStart := FALSE;
    CSP_Data.TransitionState := 70;
    resetRequest := FALSE;
END_IF
IF CSP_Data.StopRequested AND CSP_Data.Homing THEN
    CSP_Data.Homing := FALSE; homePhase := 0;
    CSP_Data.ResetRequired := TRUE; CSP_Data.ReadyToStart := FALSE;
END_IF
IF CSP_Data.Homing AND (CSP_Config.HomePosition < CSP_Config.MinimumPosition
    OR CSP_Config.HomePosition > CSP_Config.MaximumPosition OR CSP_Config.HomeTolerance <= 0
    OR CSP_Config.HomeTimeout <= T#0MS OR CSP_Config.HomeSettleTime <= T#0MS) THEN
    CSP_Data.Homing := FALSE; CSP_Data.Fault := TRUE; CSP_Data.FaultCode := 3; RETURN;
END_IF
CASE CSP_Data.TransitionState OF
60:
    // PP already enabled and positioned every axis. Do not disable/re-enable.
    // Hold the captured positions while changing mode; no PC targets are consumed.
    // A drive that leaves operation-enabled or moves away aborts the handover.
    handoverValid := allEnabled;
    FOR i := 1 TO 6 DO
        handoverValid := handoverValid
            AND ABS(actual[i]-initialPosition[i]) <= CSP_Config.HomeTolerance;
    END_FOR
    IF NOT handoverValid THEN
        CSP_Data.Fault := TRUE; CSP_Data.FaultCode := 10;
        FOR i := 1 TO 6 DO CSP_Data.Axis[i].Controlword := 16#0002; END_FOR
        RETURN;
    END_IF
    FOR i := 1 TO 6 DO
        CSP_Data.Axis[i].TargetRaw := CSP_Data.InitialTargetRaw[i];
        CSP_Data.Axis[i].CommandPosition := initialPosition[i];
        CSP_Data.Axis[i].RequestedPosition := initialPosition[i];
        CSP_Data.Axis[i].ModeRequested := 8;
        CSP_Data.Axis[i].Controlword := 16#000F;
    END_FOR
    // All-six barrier: the platform stays in handover until every display is 8.
    IF allMode THEN
        trajectory(Initialize:=TRUE, Actual:=initialPosition);
        CSP_Data.Active := TRUE; CSP_Data.ReadyToStart := TRUE;
        CSP_Data.ResetRequired := FALSE;
        CSP_Data.PlatformState := PlatformControlState.CSP_READY;
        CSP_Data.TransitionState := 61;
        // Hold for another whole cycle; earlier/cached network targets ignored.
    END_IF
61:
    IF NOT allMode OR NOT allEnabled THEN
        CSP_Data.Fault := TRUE; CSP_Data.FaultCode := 6;
        FOR i := 1 TO 6 DO CSP_Data.Axis[i].Controlword := 16#0002; END_FOR
        RETURN;
    END_IF
    CSP_Data.TransitionState := 50;
    // No trajectory update in this cycle. Only a later valid packet can move.
0:
    // Remain reset-required until the operator explicitly requests the park move.
    IF CSP_Data.Homing AND CSP_Config.Commissioned AND allReady AND GVL.safetySystem THEN
        IF CSP_Config.CycleSeconds <= 0 OR CSP_Config.MaximumVelocity <= 0
            OR CSP_Config.MaximumAcceleration <= 0 OR CSP_Config.BrakingAcceleration <= 0
            OR CSP_Config.BrakingAcceleration > CSP_Config.MaximumAcceleration
            OR CSP_Config.MaximumJerk <= 0 OR CSP_Config.MinimumPosition >= CSP_Config.MaximumPosition THEN
            CSP_Data.Fault := TRUE; CSP_Data.FaultCode := 3; RETURN;
        END_IF
        FOR i := 1 TO 6 DO
            CSP_Data.Axis[i].TargetRaw := CSP_Data.Axis[i].ActualRaw;
            CSP_Data.Axis[i].ModeRequested := CSP_Data.Axis[i].ModeActual;
            IF CSP_Data.Axis[i].CiA402State = 16#0027 THEN CSP_Data.Axis[i].Controlword := 16#010F;
            ELSE CSP_Data.Axis[i].Controlword := 16#0006; END_IF
        END_FOR
        CSP_Data.OwnsOutputs := TRUE; CSP_Data.TransitionState := 10;
    END_IF
10:
    // HALT in the current PP mode, retaining drive-configured PP deceleration.
    // Never switch mode or discard the moving setpoint before actual standstill.
    FOR i := 1 TO 6 DO
        IF CSP_Data.Axis[i].CiA402State = 16#0027 THEN
            CSP_Data.Axis[i].Controlword := 16#010F;
        ELSE CSP_Data.Axis[i].Controlword := 16#0006;
        END_IF
    END_FOR
    IF standstill.Q THEN CSP_Data.TransitionState := 20; END_IF
20:
    FOR i := 1 TO 6 DO
        CSP_Data.Axis[i].Controlword := 16#0007; // disable operation, retain voltage
        IF CSP_Data.Axis[i].CiA402State = 16#0007 THEN
            CSP_Data.Axis[i].Controlword := 0; // leave quick-stop-active via disable voltage
        END_IF
        CSP_Data.Axis[i].TargetRaw := CSP_Data.Axis[i].ActualRaw;
    END_FOR
    IF allDisabled THEN CSP_Data.TransitionState := 30; END_IF
30,40:
    FOR i := 1 TO 6 DO
        IF actual[i] < CSP_Config.MinimumPosition OR actual[i] > CSP_Config.MaximumPosition THEN
            CSP_Data.Fault := TRUE; CSP_Data.FaultCode := 5; RETURN;
        END_IF
    END_FOR
    FOR i := 1 TO 6 DO
        CSP_Data.Axis[i].TargetRaw := CSP_Data.Axis[i].ActualRaw;
        CSP_Data.Axis[i].ModeRequested := 8;
        stateWord := CSP_Data.Axis[i].CiA402State;
        CASE stateWord OF
            16#0040,16#0060: CSP_Data.Axis[i].Controlword := 16#0006;
            16#0021: CSP_Data.Axis[i].Controlword := 16#0007;
            16#0023,16#0027:
                CSP_Data.Axis[i].Controlword := 16#0007;
                // All-six mode barrier before ANY axis enables operation.
                IF allMode AND allSwitchedOn AND CSP_Data.Homing AND NOT CSP_Data.StopRequested THEN
                    CSP_Data.Axis[i].Controlword := 16#000F;
                END_IF
        ELSE
            CSP_Data.Axis[i].Controlword := 0;
            IF (stateWord AND 16#004F) = 16#0008 THEN CSP_Data.Fault := TRUE; CSP_Data.FaultCode := 4; END_IF
        END_CASE
    END_FOR
    IF allMode THEN CSP_Data.TransitionState := 40; END_IF
    IF allMode AND allEnabled AND CSP_Data.Homing AND NOT CSP_Data.StopRequested THEN
        FOR i := 1 TO 6 DO
            IF actual[i] < CSP_Config.MinimumPosition OR actual[i] > CSP_Config.MaximumPosition THEN
                CSP_Data.Fault := TRUE; CSP_Data.FaultCode := 5; RETURN;
            END_IF
        END_FOR
        trajectory(Initialize:=TRUE, Actual:=actual);
        FOR i := 1 TO 6 DO
            CSP_Data.Axis[i].CommandPosition := actual[i];
            CSP_Data.Axis[i].CommandVelocity := 0;
            CSP_Data.Axis[i].CommandAcceleration := 0;
        END_FOR
        CSP_Data.TransitionState := 50; CSP_Data.Active := TRUE;
        // Local park motion runs first; PC commands remain gated until a later Start.
    END_IF
50:
    IF NOT allMode OR NOT allEnabled THEN
        CSP_Data.Fault := TRUE; CSP_Data.FaultCode := 6; RETURN;
    END_IF
    IF CSP_Data.Homing THEN
        FOR i := 1 TO 6 DO CSP_Data.Axis[i].RequestedPosition := homeTarget[i]; END_FOR
        CASE homePhase OF
        1:
            trajectory(Initialize:=FALSE, NewTarget:=FALSE, Stop:=TRUE);
            IF NOT trajectory.Busy AND standstill.Q THEN homePhase := 2; END_IF
        2:
            trajectory(Initialize:=FALSE, NewTarget:=TRUE, Stop:=FALSE, Requested:=homeTarget);
            homePhase := 3;
        3:
            trajectory(Initialize:=FALSE, NewTarget:=FALSE, Stop:=FALSE);
            IF homeSettled.Q AND NOT trajectory.Busy THEN
                CSP_Data.Homing := FALSE; homePhase := 0;
                CSP_Data.ResetRequired := FALSE; CSP_Data.ReadyToStart := TRUE;
            END_IF
        END_CASE
    ELSE
        // No queued PC move may run during reset-required or park hold.
        trajectory(Initialize:=FALSE,
            NewTarget:=newTarget AND CSP_Data.ReadyToStart AND CSP_Data.Enable,
            Stop:=timedOut OR NOT CSP_Data.Enable OR NOT CSP_Data.ReadyToStart
                OR CSP_Data.StopRequested, Requested:=requested);
        IF newTarget AND CSP_Data.ReadyToStart AND CSP_Data.Enable THEN
            FOR i := 1 TO 6 DO CSP_Data.Axis[i].RequestedPosition := requested[i]; END_FOR
        END_IF
    END_IF
    IF trajectory.Fault THEN CSP_Data.Fault := TRUE; CSP_Data.FaultCode := 7; RETURN; END_IF
    IF CSP_Data.Homing OR CSP_Data.ResetRequired THEN
        CSP_Data.PlatformState := PlatformControlState.CSP_RECOVERY;
    ELSIF CSP_Data.Enable AND NOT timedOut AND (newTarget OR CSP_Data.PlatformState = PlatformControlState.CSP_RUNNING) THEN
        CSP_Data.PlatformState := PlatformControlState.CSP_RUNNING;
    ELSE
        CSP_Data.PlatformState := PlatformControlState.CSP_READY;
    END_IF
    // Festo factor is user units per raw count: output is the inverse of readback.
    CSP_Data.ReversalDeferred := trajectory.Deferred;
    FOR i := 1 TO 6 DO
        CSP_Data.Axis[i].CommandPosition := trajectory.P[i];
        CSP_Data.Axis[i].CommandVelocity := trajectory.V[i];
        CSP_Data.Axis[i].CommandAcceleration := trajectory.A[i];
        CSP_Data.Axis[i].TargetRaw := LREAL_TO_DINT(trajectory.P[i]/CSP_Data.Axis[i].PositionFactor);
        CSP_Data.Axis[i].Controlword := 16#000F; // only while operation-enabled in CSP
        IF trajectory.V[i] >= 0 THEN distance := CSP_Config.MaximumPosition-trajectory.P[i];
        ELSE distance := trajectory.P[i]-CSP_Config.MinimumPosition; END_IF
        CSP_Data.Axis[i].VelocityAllowed := MIN(CSP_Config.MaximumVelocity,
            SQRT(2.0*CSP_Config.BrakingAcceleration*MAX(0.0,distance)));
    END_FOR
70:
    // Reset handback: bring any active CSP command to rest in mode 8 first.
    CSP_Data.PlatformState := PlatformControlState.PP_HANDBACK;
    IF CSP_Data.Active THEN
        IF NOT allMode OR NOT allEnabled THEN
            CSP_Data.Fault := TRUE; CSP_Data.FaultCode := 6; RETURN;
        END_IF
        trajectory(Initialize:=FALSE, NewTarget:=FALSE, Stop:=TRUE);
        IF trajectory.Fault THEN CSP_Data.Fault := TRUE; CSP_Data.FaultCode := 7; RETURN; END_IF
        FOR i := 1 TO 6 DO
            CSP_Data.Axis[i].CommandPosition := trajectory.P[i];
            CSP_Data.Axis[i].CommandVelocity := trajectory.V[i];
            CSP_Data.Axis[i].CommandAcceleration := trajectory.A[i];
            CSP_Data.Axis[i].TargetRaw := LREAL_TO_DINT(trajectory.P[i]/CSP_Data.Axis[i].PositionFactor);
            CSP_Data.Axis[i].ModeRequested := 8;
            CSP_Data.Axis[i].Controlword := 16#000F;
        END_FOR
        IF NOT trajectory.Busy AND standstill.Q THEN
            FOR i := 1 TO 6 DO holdRaw[i] := CSP_Data.Axis[i].TargetRaw; END_FOR
            CSP_Data.Active := FALSE; CSP_Data.TransitionState := 71;
        END_IF
    ELSE
        // Handover hold or after quick stop: keep enabled axes at their current
        // CSP target; disabled/faulted axes get no enable and no fault reset.
        FOR i := 1 TO 6 DO
            IF CSP_Data.Axis[i].CiA402State = 16#0027 THEN
                holdRaw[i] := CSP_Data.Axis[i].TargetRaw;
            ELSE
                holdRaw[i] := CSP_Data.Axis[i].ActualRaw;
            END_IF
        END_FOR
        CSP_Data.TransitionState := 71;
    END_IF
71:
    // Request PP (mode 1) on all six drives while holding position. In PP mode a
    // target without a new-setpoint bit starts no motion.
    CSP_Data.PlatformState := PlatformControlState.PP_HANDBACK;
    FOR i := 1 TO 6 DO
        CSP_Data.Axis[i].TargetRaw := holdRaw[i];
        CSP_Data.Axis[i].ModeRequested := CSP_Config.CONTROL_MODE_PP;
        IF CSP_Data.Axis[i].CiA402State = 16#0027 THEN CSP_Data.Axis[i].Controlword := 16#000F;
        ELSE CSP_Data.Axis[i].Controlword := 16#0002; END_IF
    END_FOR
    // All six displays must confirm mode 1 (ModeBlockingAxis names a laggard);
    // otherwise the transition timeout raises FaultCode 1 with TimeoutState 71.
    IF allPPMode THEN
        // Mode 1 was already in the PDO image in earlier cycles. From this
        // cycle's CSP_WriteOutputs call the original Festo writer is the sole writer.
        CSP_Data.Active := FALSE; CSP_Data.ReadyToStart := FALSE;
        CSP_Data.ResetRequired := TRUE; CSP_Data.Homing := FALSE; homePhase := 0;
        CSP_Data.TransitionState := 0;
        CSP_Data.PlatformState := PlatformControlState.PP_RESET;
        resetRequest := FALSE;
        CSP_Data.CSPReleased := TRUE;
        CSP_Data.OwnsOutputs := FALSE;
    END_IF
END_CASE
