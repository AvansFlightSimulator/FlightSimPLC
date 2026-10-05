FUNCTION CSP_ReadNumber : BOOL
VAR_INPUT
    Text : STRING(511);
END_VAR
VAR_IN_OUT
    Cursor : INT;
    Number : LREAL;
END_VAR
VAR
    first, digits : INT;
    token : STRING(48);
END_VAR
(* IMPLEMENTATION *)
// JSON number grammar, bounded token length. Reject malformed numbers instead
// of relying on STRING_TO_LREAL's permissive prefix conversion.
CSP_ReadNumber := FALSE; first := Cursor;
IF Text[Cursor] = 45 THEN Cursor := Cursor+1; END_IF
digits := 0;
IF Text[Cursor] = 48 THEN Cursor := Cursor+1; digits := 1;
ELSE
    WHILE Cursor <= LEN(Text) AND Text[Cursor] >= 48 AND Text[Cursor] <= 57 DO
        Cursor := Cursor+1; digits := digits+1;
    END_WHILE
END_IF
IF digits = 0 THEN RETURN; END_IF
IF Text[Cursor] = 46 THEN
    Cursor := Cursor+1; digits := 0;
    WHILE Cursor <= LEN(Text) AND Text[Cursor] >= 48 AND Text[Cursor] <= 57 DO
        Cursor := Cursor+1; digits := digits+1;
    END_WHILE
    IF digits = 0 THEN RETURN; END_IF
END_IF
IF Text[Cursor] = 101 OR Text[Cursor] = 69 THEN
    Cursor := Cursor+1;
    IF Text[Cursor] = 43 OR Text[Cursor] = 45 THEN Cursor := Cursor+1; END_IF
    digits := 0;
    WHILE Cursor <= LEN(Text) AND Text[Cursor] >= 48 AND Text[Cursor] <= 57 DO
        Cursor := Cursor+1; digits := digits+1;
    END_WHILE
    // C++ float exponents fit two digits. Bound before conversion so hostile
    // huge exponents cannot overflow the PLC's STRING_TO_LREAL conversion.
    IF digits = 0 OR digits > 2 THEN RETURN; END_IF
END_IF
IF Cursor-first > 48 THEN RETURN; END_IF
token := MID(Text, Cursor-first, first+1);
Number := STRING_TO_LREAL(token);
// All accepted numbers are finite and bounded; no NaN or Infinity refreshes watchdog.
CSP_ReadNumber := Number = Number AND ABS(Number) <= 1000000.0;
