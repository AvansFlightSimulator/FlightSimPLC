FUNCTION_BLOCK CSP_Receive
VAR_INPUT
    Connected : BOOL;
    Connection : CAA.HANDLE;
END_VAR
VAR_OUTPUT
    RejectedFrames, AcceptedFrames : UDINT;
    Error : BOOL;
END_VAR
VAR
    reader : NBS.TCP_Read;
    buffer : ARRAY[0..510] OF BYTE;
    count : UDINT;
    frame : STRING(511);
    length, i : INT;
    discarding : BOOL;
    positions : ARRAY[1..6] OF LREAL;
    // Dashboard commands hold the HMI flag long enough for the main task to see it.
    startHold, stopHold, resetHold : TON;
    startActive, stopActive, resetActive : BOOL;
    commandAccepted : BOOL;
END_VAR
VAR CONSTANT
    CommandPulse : TIME := T#200MS;
END_VAR
(* IMPLEMENTATION *)
// Runs before the connection check so a flag cannot stay set after a disconnect.
// Each flag is cleared only at the end of its own pulse, so the PLC HMI can
// still write these variables between dashboard commands.
startHold(IN:=startActive, PT:=CommandPulse);
stopHold(IN:=stopActive, PT:=CommandPulse);
resetHold(IN:=resetActive, PT:=CommandPulse);
IF startHold.Q THEN GVL.startHMI := FALSE; startActive := FALSE; END_IF
IF stopHold.Q THEN GVL.stopHMI := FALSE; stopActive := FALSE; END_IF
IF resetHold.Q THEN GVL.resetHMI := FALSE; resetActive := FALSE; END_IF
CSP_Data.Connected := Connected;
Error := FALSE;
IF NOT Connected THEN
    reader(xEnable:=FALSE); length := 0; discarding := FALSE; frame := ''; RETURN;
END_IF
reader(xEnable:=TRUE,hConnection:=Connection,szSize:=SIZEOF(buffer),pData:=ADR(buffer),szCount=>count);
IF reader.xReady AND count > 0 THEN
    FOR i := 0 TO UDINT_TO_INT(count)-1 DO
        IF buffer[i] = 10 THEN
            frame[length] := 0;
            // Operator commands use the existing HMI flags and must match exactly.
            // GVL.safetySystem is the hardware safety relay and is never written here.
            commandAccepted := NOT discarding AND length <= 254;
            IF commandAccepted THEN
                IF frame = '{"command":"start"}' THEN GVL.startHMI := TRUE; startActive := TRUE;
                ELSIF frame = '{"command":"stop"}' THEN GVL.stopHMI := TRUE; stopActive := TRUE;
                ELSIF frame = '{"command":"reset"}' THEN GVL.resetHMI := TRUE; resetActive := TRUE;
                ELSIF frame = '{"command":"estop"}' THEN GVL.softwareEStop := TRUE;
                ELSE commandAccepted := FALSE;
                END_IF
            END_IF
            IF commandAccepted THEN
                CSP_Data.CommandsAccepted := CSP_Data.CommandsAccepted+1;
            ELSIF NOT discarding AND length <= 254 AND CSP_ParseFrame(Frame:=frame,Positions:=positions) THEN
                CSP_Data.Generation := CSP_Data.Generation+1;
                CSP_Data.ReceivedPosition := positions; CSP_Data.ReceivedMode := 8;
                CSP_Data.ReceivedTick := TIME_TO_UDINT(TIME());
                CSP_Data.Generation := CSP_Data.Generation+1;
                AcceptedFrames := AcceptedFrames+1;
            ELSE RejectedFrames := RejectedFrames+1;
            END_IF
            length := 0; discarding := FALSE;
        ELSIF NOT discarding THEN
            IF length < 510 THEN frame[length] := buffer[i]; length := length+1;
            ELSE discarding := TRUE;
            END_IF
        END_IF
    END_FOR
END_IF
IF reader.xError THEN CSP_Data.Connected := FALSE; Error := TRUE; END_IF
