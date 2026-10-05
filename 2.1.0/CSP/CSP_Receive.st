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
END_VAR
(* IMPLEMENTATION *)
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
            IF NOT discarding AND length <= 254 AND CSP_ParseFrame(Frame:=frame,Positions:=positions) THEN
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
