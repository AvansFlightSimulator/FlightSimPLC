FUNCTION CSP_ParseFrame : BOOL
VAR_INPUT
    Frame : STRING(511);
END_VAR
VAR_IN_OUT
    Positions : ARRAY[1..6] OF LREAL;
END_VAR
VAR
    parsed : ARRAY[1..6] OF LREAL;
    cursor, i : INT;
    speed : LREAL;
    prefix : STRING := '{"controlMode":8,"positions":[';
    middle : STRING := ',"speeds":[';
END_VAR
(* IMPLEMENTATION *)
// Deliberately accept the exact compact schema emitted by BuildCommandPayload.
// Unknown fields/modes, missing/extra axes and legacy PP packets cannot move CSP.
CSP_ParseFrame := FALSE;
IF LEFT(Frame,LEN(prefix)) <> prefix THEN RETURN; END_IF
cursor := LEN(prefix);
FOR i := 1 TO 6 DO
    IF NOT CSP_ReadNumber(Text:=Frame,Cursor:=cursor,Number:=parsed[i]) THEN RETURN; END_IF
    IF i < 6 THEN
        IF Frame[cursor] <> 44 THEN RETURN; END_IF
    ELSE IF Frame[cursor] <> 93 THEN RETURN; END_IF
    END_IF
    cursor := cursor+1;
END_FOR
IF MID(Frame,LEN(middle),cursor+1) <> middle THEN RETURN; END_IF
cursor := cursor+LEN(middle);
FOR i := 1 TO 6 DO
    IF NOT CSP_ReadNumber(Text:=Frame,Cursor:=cursor,Number:=speed) OR speed <> 0 THEN RETURN; END_IF
    IF i < 6 THEN
        IF Frame[cursor] <> 44 THEN RETURN; END_IF
    ELSE IF Frame[cursor] <> 93 THEN RETURN; END_IF
    END_IF
    cursor := cursor+1;
END_FOR
IF Frame[cursor] <> 125 OR cursor+1 <> LEN(Frame) THEN RETURN; END_IF
Positions := parsed; CSP_ParseFrame := TRUE;
