FUNCTION_BLOCK CSP_Trajectory6
VAR_INPUT
    Initialize, NewTarget, Stop : BOOL;
    Actual, Requested : ARRAY[1..6] OF LREAL;
END_VAR
VAR_OUTPUT
    P, V, A : ARRAY[1..6] OF LREAL;
    // Busy: not yet at rest on the latest target. Braking: Stop in progress.
    // Deferred: some axis passes its target before reversing (unavoidable with
    // the current V/A and the acceleration/jerk limits).
    Busy, Braking, Deferred, Fault : BOOL;
END_VAR
VAR CONSTANT
    MaximumRampSeconds : LREAL := 0.1; // >= PC send period (nominal 50 ms)
    Iterations : INT := 20; // bisection steps: 2*MaximumJerk / 2^20 resolution
END_VAR
VAR
    goal, rampStart, rampEnd : ARRAY[1..6] OF LREAL;
    rampElapsed, rampDuration, sinceTarget : LREAL;
    stopping : BOOL;
    cycleTime, maxV, maxA, maxJ : LREAL;
    jHigh, jLow, jTry, lo, hi, p1, v1, a1, rest : LREAL;
    i, k : INT;
    ok : BOOL;
END_VAR
(* IMPLEMENTATION *)
// Online, receding-horizon generator. Every CSP cycle each axis independently
// chooses one jerk value from its CURRENT commanded P/V/A, so P, V and A are
// always continuous and a new target simply changes the next choice; there is
// no stop-at-target plan to finish and no separate brake-to-standstill path.
// The chosen jerk is the one whose resulting state, braked at full limits,
// comes to rest closest to the target (a time-optimal approach). It is limited
// by MaximumJerk, MaximumAcceleration/BrakingAcceleration, a jerk-aware
// MaximumVelocity envelope, and a travel envelope: from every commanded state
// a full-limit stop ends inside MinimumPosition..MaximumPosition.
// The 20 Hz targets are linearly interpolated over the measured packet
// interval, starting from the current interpolated target (one packet delay).
cycleTime := CSP_Config.CycleSeconds;
maxV := CSP_Config.MaximumVelocity;
maxA := MIN(CSP_Config.MaximumAcceleration, CSP_Config.BrakingAcceleration);
maxJ := CSP_Config.MaximumJerk;
IF Initialize THEN
    P := Actual; goal := Actual; rampStart := Actual; rampEnd := Actual;
    FOR i := 1 TO 6 DO V[i] := 0; A[i] := 0; END_FOR
    rampDuration := MAX(cycleTime, 1E-3); rampElapsed := rampDuration;
    sinceTarget := MaximumRampSeconds; stopping := FALSE;
    Busy := FALSE; Braking := FALSE; Deferred := FALSE;
    Fault := cycleTime <= 0 OR maxV <= 0 OR maxA <= 0 OR maxJ <= 0
        OR CSP_Config.MinimumPosition >= CSP_Config.MaximumPosition;
    RETURN;
END_IF
IF Fault THEN RETURN; END_IF

IF Stop THEN
    IF NOT stopping THEN
        // Target = where a full-limit stop from the current state ends.
        FOR i := 1 TO 6 DO
            goal[i] := P[i] + CSP_StopOffset(V:=V[i], A:=A[i], MaxAcceleration:=maxA, MaxJerk:=maxJ);
        END_FOR
        rampStart := goal; rampEnd := goal; rampElapsed := rampDuration;
        stopping := TRUE;
    END_IF
ELSE
    stopping := FALSE;
    IF NewTarget THEN
        // Retarget immediately from the current interpolated target: no jump.
        FOR i := 1 TO 6 DO
            rampStart[i] := goal[i];
            rampEnd[i] := LIMIT(CSP_Config.MinimumPosition, Requested[i], CSP_Config.MaximumPosition);
        END_FOR
        rampDuration := LIMIT(2.0*cycleTime, sinceTarget, MaximumRampSeconds);
        rampElapsed := 0; sinceTarget := 0;
    END_IF
END_IF
sinceTarget := MIN(sinceTarget + cycleTime, MaximumRampSeconds);
rampElapsed := MIN(rampDuration, rampElapsed + cycleTime);

Busy := FALSE; Deferred := FALSE;
FOR i := 1 TO 6 DO
    IF rampElapsed >= rampDuration THEN
        goal[i] := rampEnd[i]; // exact: lets Busy clear at rest
    ELSE
        goal[i] := rampStart[i] + (rampEnd[i] - rampStart[i])*rampElapsed/rampDuration;
    END_IF
    IF rampElapsed >= rampDuration AND ABS(A[i]) <= maxJ*cycleTime AND ABS(V[i]) <= maxJ*cycleTime*cycleTime
        AND ABS(P[i] - goal[i]) <= maxJ*cycleTime*cycleTime*cycleTime THEN
        // Terminal capture: the remaining stop is shorter than one cycle. The
        // change is within one cycle of full jerk; it prevents a +/- limit cycle.
        P[i] := goal[i]; V[i] := 0; A[i] := 0;
    ELSE
        jHigh := MIN(maxJ, (maxA - A[i])/cycleTime);
        jLow := MAX(-maxJ, (-maxA - A[i])/cycleTime);
        // Upper envelope: velocity peak after unwinding A, and stop below MaximumPosition.
        jTry := jHigh;
        p1 := P[i] + V[i]*cycleTime + A[i]*cycleTime*cycleTime/2.0 + jTry*cycleTime*cycleTime*cycleTime/6.0;
        v1 := V[i] + A[i]*cycleTime + jTry*cycleTime*cycleTime/2.0;
        a1 := A[i] + jTry*cycleTime;
        ok := v1 + MAX(a1, 0.0)*MAX(a1, 0.0)/(2.0*maxJ) <= maxV
            AND p1 + CSP_StopOffset(V:=v1, A:=a1, MaxAcceleration:=maxA, MaxJerk:=maxJ)
            <= CSP_Config.MaximumPosition;
        IF NOT ok THEN
            lo := jLow; hi := jHigh;
            FOR k := 1 TO Iterations DO
                jTry := (lo + hi)/2.0;
                p1 := P[i] + V[i]*cycleTime + A[i]*cycleTime*cycleTime/2.0 + jTry*cycleTime*cycleTime*cycleTime/6.0;
                v1 := V[i] + A[i]*cycleTime + jTry*cycleTime*cycleTime/2.0;
                a1 := A[i] + jTry*cycleTime;
                ok := v1 + MAX(a1, 0.0)*MAX(a1, 0.0)/(2.0*maxJ) <= maxV
                    AND p1 + CSP_StopOffset(V:=v1, A:=a1, MaxAcceleration:=maxA, MaxJerk:=maxJ)
                        <= CSP_Config.MaximumPosition;
                IF ok THEN lo := jTry; ELSE hi := jTry; END_IF
            END_FOR
            jHigh := lo;
        END_IF
        // Lower envelope: velocity trough and stop above MinimumPosition.
        jTry := jLow;
        p1 := P[i] + V[i]*cycleTime + A[i]*cycleTime*cycleTime/2.0 + jTry*cycleTime*cycleTime*cycleTime/6.0;
        v1 := V[i] + A[i]*cycleTime + jTry*cycleTime*cycleTime/2.0;
        a1 := A[i] + jTry*cycleTime;
        ok := v1 - MIN(a1, 0.0)*MIN(a1, 0.0)/(2.0*maxJ) >= -maxV
            AND p1 + CSP_StopOffset(V:=v1, A:=a1, MaxAcceleration:=maxA, MaxJerk:=maxJ)
            >= CSP_Config.MinimumPosition;
        IF NOT ok THEN
            lo := jLow; hi := jHigh;
            FOR k := 1 TO Iterations DO
                jTry := (lo + hi)/2.0;
                p1 := P[i] + V[i]*cycleTime + A[i]*cycleTime*cycleTime/2.0 + jTry*cycleTime*cycleTime*cycleTime/6.0;
                v1 := V[i] + A[i]*cycleTime + jTry*cycleTime*cycleTime/2.0;
                a1 := A[i] + jTry*cycleTime;
                ok := v1 - MIN(a1, 0.0)*MIN(a1, 0.0)/(2.0*maxJ) >= -maxV
                    AND p1 + CSP_StopOffset(V:=v1, A:=a1, MaxAcceleration:=maxA, MaxJerk:=maxJ)
                        >= CSP_Config.MinimumPosition;
                IF ok THEN hi := jTry; ELSE lo := jTry; END_IF
            END_FOR
            jLow := hi;
        END_IF
        IF jLow > jHigh THEN
            // Not reachable from a state this block produced; reduce |A| only.
            jTry := LIMIT(-maxJ, -A[i]/cycleTime, maxJ);
        ELSE
            // Rest point after this cycle rises monotonically with jerk: choose
            // the jerk whose rest point is the target, clamped to the envelope.
            jTry := jHigh;
            p1 := P[i] + V[i]*cycleTime + A[i]*cycleTime*cycleTime/2.0 + jTry*cycleTime*cycleTime*cycleTime/6.0;
            v1 := V[i] + A[i]*cycleTime + jTry*cycleTime*cycleTime/2.0;
            a1 := A[i] + jTry*cycleTime;
            rest := p1 + CSP_StopOffset(V:=v1, A:=a1, MaxAcceleration:=maxA, MaxJerk:=maxJ);
            IF rest > goal[i] THEN
                jTry := jLow;
                p1 := P[i] + V[i]*cycleTime + A[i]*cycleTime*cycleTime/2.0 + jTry*cycleTime*cycleTime*cycleTime/6.0;
                v1 := V[i] + A[i]*cycleTime + jTry*cycleTime*cycleTime/2.0;
                a1 := A[i] + jTry*cycleTime;
                rest := p1 + CSP_StopOffset(V:=v1, A:=a1, MaxAcceleration:=maxA, MaxJerk:=maxJ);
                IF rest < goal[i] THEN
                    lo := jLow; hi := jHigh;
                    FOR k := 1 TO Iterations DO
                        jTry := (lo + hi)/2.0;
                        p1 := P[i] + V[i]*cycleTime + A[i]*cycleTime*cycleTime/2.0 + jTry*cycleTime*cycleTime*cycleTime/6.0;
                        v1 := V[i] + A[i]*cycleTime + jTry*cycleTime*cycleTime/2.0;
                        a1 := A[i] + jTry*cycleTime;
                        rest := p1 + CSP_StopOffset(V:=v1, A:=a1, MaxAcceleration:=maxA, MaxJerk:=maxJ);
                        IF rest > goal[i] THEN hi := jTry; ELSE lo := jTry; END_IF
                    END_FOR
                    jTry := lo;
                END_IF
            END_IF
        END_IF
        // Exact cubic integration over one cycle.
        P[i] := P[i] + V[i]*cycleTime + A[i]*cycleTime*cycleTime/2.0 + jTry*cycleTime*cycleTime*cycleTime/6.0;
        V[i] := V[i] + A[i]*cycleTime + jTry*cycleTime*cycleTime/2.0;
        A[i] := A[i] + jTry*cycleTime;
    END_IF
    IF P[i] < CSP_Config.MinimumPosition - 1E-6 OR P[i] > CSP_Config.MaximumPosition + 1E-6 THEN
        Fault := TRUE;
    END_IF
    rest := P[i] + CSP_StopOffset(V:=V[i], A:=A[i], MaxAcceleration:=maxA, MaxJerk:=maxJ);
    Deferred := Deferred OR (V[i]*(goal[i] - rest) < 0 AND ABS(goal[i] - rest) > 1E-3);
    Busy := Busy OR V[i] <> 0 OR A[i] <> 0 OR P[i] <> rampEnd[i];
END_FOR
Braking := stopping AND Busy;
