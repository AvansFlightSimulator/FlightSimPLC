FUNCTION_BLOCK CSP_Brake
VAR_INPUT
    Start : BOOL;
    Position, Velocity, Acceleration : LREAL;
    StepSeconds, MaxAcceleration, MaxJerk : LREAL;
END_VAR
VAR_OUTPUT
    P, V, A, StopPosition : LREAL;
    Done, Valid : BOOL;
END_VAR
VAR
    t : ARRAY[1..3] OF LREAL;
    j : ARRAY[1..3] OF LREAL;
    phase : INT;
    sign, speed, accel, peak, h, remaining : LREAL;
    sp, sv, sa : LREAL;
    k : INT;
END_VAR
(* IMPLEMENTATION *)
IF Start THEN
    P := Position; V := Velocity; A := Acceleration;
    Done := FALSE; Valid := FALSE; phase := 1;
    IF MaxJerk <= 0 OR MaxAcceleration <= 0 THEN RETURN; END_IF
    sign := 1.0;
    IF Velocity < 0 THEN sign := -1.0; END_IF
    speed := sign * Velocity; accel := sign * Acceleration;
    IF ABS(Velocity) < 1E-10 AND ABS(Acceleration) < 1E-10 THEN
        StopPosition := P; Done := TRUE; Valid := TRUE; RETURN;
    END_IF
    // Exact jerk-aware minimum stopping profile: -J, optional -A, +J.
    // A valid monotone trajectory must be able to unwind its braking acceleration
    // before velocity changes sign. Do not repair an invalid state by clamping V.
    IF accel < 0 AND speed + 1E-9 < accel * accel / (2.0 * MaxJerk) THEN RETURN; END_IF
    peak := MIN(MaxAcceleration, SQRT(MaxJerk * speed + accel * accel / 2.0));
    IF peak <= 0 THEN RETURN; END_IF
    t[1] := MAX(0.0, (accel + peak) / MaxJerk);
    t[2] := MAX(0.0, (speed + (accel*accel - 2.0*peak*peak)/(2.0*MaxJerk)) / peak);
    t[3] := peak / MaxJerk;
    j[1] := -sign * MaxJerk; j[2] := 0; j[3] := sign * MaxJerk;
    sp := P; sv := V; sa := A;
    FOR k := 1 TO 3 DO
        h := t[k];
        sp := sp + sv*h + sa*h*h/2.0 + j[k]*h*h*h/6.0;
        sv := sv + sa*h + j[k]*h*h/2.0;
        sa := sa + j[k]*h;
    END_FOR
    StopPosition := sp; Valid := TRUE;
    RETURN;
END_IF
IF Done OR NOT Valid THEN RETURN; END_IF
remaining := StepSeconds;
// Split a PLC interval at jerk-phase boundaries; never integrate beyond a stop.
FOR k := 1 TO 3 DO
    IF phase <= 3 THEN
        h := MIN(remaining, t[phase]);
        P := P + V*h + A*h*h/2.0 + j[phase]*h*h*h/6.0;
        V := V + A*h + j[phase]*h*h/2.0;
        A := A + j[phase]*h;
        t[phase] := t[phase] - h; remaining := remaining - h;
        IF t[phase] <= 1E-12 THEN phase := phase + 1; END_IF
    END_IF
END_FOR
IF phase > 3 THEN
    // Analytical terminal state, not a high-speed position clamp.
    P := StopPosition; V := 0; A := 0; Done := TRUE;
END_IF
