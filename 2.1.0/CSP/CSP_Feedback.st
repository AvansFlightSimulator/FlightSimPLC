FUNCTION CSP_Feedback : STRING(4095)
VAR
    output : STRING(4095);
    fragment : STRING(255);
    i, field : INT;
    ok : BOOL;
    cycle : UDINT;
    snapshot : ARRAY[1..6] OF CSP_AxisData;
END_VAR
(* IMPLEMENTATION *)
cycle := CSP_Data.CycleCounter;
snapshot := CSP_Data.Axis;
IF cycle <> CSP_Data.CycleCounter THEN CSP_Feedback := ''; RETURN; END_IF
FOR i := 1 TO 6 DO
    IF NOT snapshot[i].Ready THEN CSP_Feedback := ''; RETURN; END_IF
END_FOR
output := '{"currentPositions":['; ok := TRUE;
FOR i := 1 TO 6 DO
    IF i > 1 THEN ok := AppendDiagnosticText(',',output) AND ok; END_IF
    ok := AppendDiagnosticText(LREAL_TO_STRING(snapshot[i].ActualPosition),output) AND ok;
END_FOR
ok := AppendDiagnosticText('],"diagnostics":{"version":2,"controlMode":8,"positionUnit":"mm","velocityUnit":"mm/s","registerUnit":"raw CiA402","targetSource":"CSP PDO command","followingErrorSource":"0x60F4 PDO","plcTickMs":',output) AND ok;
ok := AppendDiagnosticText(UDINT_TO_STRING(TIME_TO_UDINT(TIME())),output) AND ok;
FOR field := 1 TO 14 DO
    CASE field OF
        1: fragment := ',"receivedPositions":[';
        2: fragment := ',"receivedSpeeds":[';
        3: fragment := ',"actualPositions":[';
        4: fragment := ',"actualVelocities":[';
        5: fragment := ',"driveTargetPositions":[';
        6: fragment := ',"followingErrors":[';
        7: fragment := ',"commandPositions":[';
        8: fragment := ',"commandVelocities":[';
        9: fragment := ',"commandAccelerations":[';
        10: fragment := ',"modeRequested":[';
        11: fragment := ',"modeActual":[';
        12: fragment := ',"cia402State":[';
        13: fragment := ',"positionLimit":[';
        14: fragment := ',"communicationTimeout":[';
    END_CASE
    ok := AppendDiagnosticText(fragment,output) AND ok;
    FOR i := 1 TO 6 DO
        IF i > 1 THEN ok := AppendDiagnosticText(',',output) AND ok; END_IF
        CASE field OF
            1: fragment := LREAL_TO_STRING(snapshot[i].RequestedPosition);
            2: fragment := '0';
            3: fragment := LREAL_TO_STRING(snapshot[i].ActualPosition);
            4: fragment := LREAL_TO_STRING(snapshot[i].ActualVelocity);
            5: fragment := DINT_TO_STRING(snapshot[i].TargetRaw);
            6: fragment := DINT_TO_STRING(snapshot[i].FollowingErrorRaw);
            7: fragment := LREAL_TO_STRING(snapshot[i].CommandPosition);
            8: fragment := LREAL_TO_STRING(snapshot[i].CommandVelocity);
            9: fragment := LREAL_TO_STRING(snapshot[i].CommandAcceleration);
            10: fragment := SINT_TO_STRING(snapshot[i].ModeRequested);
            11: fragment := SINT_TO_STRING(snapshot[i].ModeActual);
            12: fragment := UINT_TO_STRING(WORD_TO_UINT(snapshot[i].CiA402State));
            13: fragment := INT_TO_STRING(BOOL_TO_INT(snapshot[i].PositionLimit));
            14: fragment := INT_TO_STRING(BOOL_TO_INT(snapshot[i].CommunicationTimeout));
        END_CASE
        IF field >= 7 AND field <= 9 AND NOT CSP_Data.Active THEN fragment := 'null'; END_IF
        ok := AppendDiagnosticText(fragment,output) AND ok;
    END_FOR
    ok := AppendDiagnosticText(']',output) AND ok;
END_FOR
ok := AppendDiagnosticText(',"resetRequired":',output) AND ok;
ok := AppendDiagnosticText(INT_TO_STRING(BOOL_TO_INT(CSP_Data.ResetRequired)),output) AND ok;
ok := AppendDiagnosticText(',"homing":',output) AND ok;
ok := AppendDiagnosticText(INT_TO_STRING(BOOL_TO_INT(CSP_Data.Homing)),output) AND ok;
ok := AppendDiagnosticText(',"readyToStart":',output) AND ok;
ok := AppendDiagnosticText(INT_TO_STRING(BOOL_TO_INT(CSP_Data.ReadyToStart)),output) AND ok;
ok := AppendDiagnosticText('}}$N',output) AND ok;
// Build the packet from the coherent copy, outside the EtherCAT task.
IF ok THEN CSP_Feedback := output; ELSE CSP_Feedback := ''; END_IF
