FUNCTION CSP_Bezier : LREAL
VAR_INPUT
    Points : ARRAY[0..5] OF LREAL;
    Degree : INT;
    U : LREAL;
END_VAR
VAR
    work : ARRAY[0..5] OF LREAL;
    i, k : INT;
END_VAR
(* IMPLEMENTATION *)
// de Casteljau evaluation: bounded interpolation, including near endpoints.
work := Points;
FOR k := Degree TO 1 BY -1 DO
    FOR i := 0 TO k - 1 DO
        work[i] := (1.0 - U) * work[i] + U * work[i+1];
    END_FOR
END_FOR
CSP_Bezier := work[0];
