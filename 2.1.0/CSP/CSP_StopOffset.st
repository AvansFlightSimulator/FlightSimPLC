FUNCTION CSP_StopOffset : LREAL
VAR_INPUT
    V, A : LREAL;
    MaxAcceleration, MaxJerk : LREAL;
END_VAR
VAR
    t : ARRAY[1..3] OF LREAL;
    sign, speed, accel, peak, h, jk, sp, sv, sa : LREAL;
    k : INT;
END_VAR
(* IMPLEMENTATION *)
// Signed displacement of the minimum-time jerk-limited stop (V = A = 0) from
// (V, A): jerk -J, optional constant -peak, jerk +J (mirrored by sign). The braking
// direction is the sign of the velocity left after unwinding A at full jerk, so
// every state is valid, including one that must cross zero velocity first.
// The result increases monotonically with V and A.
CSP_StopOffset := 0;
IF MaxJerk <= 0 OR MaxAcceleration <= 0 THEN RETURN; END_IF
sign := 1.0;
IF V + A*ABS(A)/(2.0*MaxJerk) < 0 THEN sign := -1.0; END_IF
speed := sign*V; accel := sign*A;
IF ABS(speed) < 1E-12 AND ABS(accel) < 1E-12 THEN RETURN; END_IF
peak := MIN(MaxAcceleration, SQRT(MAX(0.0, MaxJerk*speed + accel*accel/2.0)));
t[1] := MAX(0.0, (accel + peak)/MaxJerk);
t[2] := 0;
IF peak > 0 THEN
    t[2] := MAX(0.0, (speed + (accel*accel - 2.0*peak*peak)/(2.0*MaxJerk))/peak);
END_IF
t[3] := peak/MaxJerk;
sp := 0; sv := speed; sa := accel;
FOR k := 1 TO 3 DO
    CASE k OF
        1: jk := -MaxJerk;
        2: jk := 0;
    ELSE
        jk := MaxJerk;
    END_CASE
    h := t[k];
    sp := sp + sv*h + sa*h*h/2.0 + jk*h*h*h/6.0;
    sv := sv + sa*h + jk*h*h/2.0;
    sa := sa + jk*h;
END_FOR
CSP_StopOffset := sign*sp;
