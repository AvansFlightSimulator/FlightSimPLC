FUNCTION CSP_ReadAxis : BOOL
VAR_INPUT
    Axis : REFERENCE TO AXIS_REF_CiA402_ETC_CMMT_AS;
    PlatformAxis : INT;
END_VAR
(* IMPLEMENTATION *)
IF PlatformAxis < 1 OR PlatformAxis > 6 THEN RETURN; END_IF
CSP_Data.Axis[PlatformAxis].ReadinessCode := 1;

IF Axis.pIoStatusword = 0
    OR Axis.pIoModesOfOperationDisplay = 0
    OR Axis.pIoPositionActualValue = 0
    OR Axis.pIoVelocityActualValue = 0
    OR Axis.pIoControlword = 0
    OR Axis.pIoModesOfOperation = 0
    OR Axis.pIoTargetPosition = 0 THEN

    CSP_Data.Axis[PlatformAxis].Ready := FALSE;
    RETURN;
END_IF

CSP_Data.Axis[PlatformAxis].Statusword := Axis.pIoStatusword^;
CSP_Data.Axis[PlatformAxis].ModeActual := Axis.pIoModesOfOperationDisplay^;
CSP_Data.Axis[PlatformAxis].ActualRaw := Axis.pIoPositionActualValue^;
CSP_Data.Axis[PlatformAxis].ActualPosition := Axis.ActualPosition;
CSP_Data.Axis[PlatformAxis].ActualVelocity := Axis.ActualVelocity;

// Reuse Festo conversion factors.
CSP_Data.Axis[PlatformAxis].PositionFactor := Axis.rPositionFactor;
CSP_Data.Axis[PlatformAxis].VelocityFactor := Axis.rVelocityFactor;

CSP_Data.Axis[PlatformAxis].Ready :=
    Axis.SDOsInitialized
    AND Axis.rPositionFactor > 0
    AND (Axis.SlaveConnectorFlags AND 16#3F0) = 16#F0;

// Verify raw position conversion against Festo's calculated position.
IF Axis.rPositionFactor > 0 THEN

    CSP_Data.Axis[PlatformAxis].Ready :=
        CSP_Data.Axis[PlatformAxis].Ready
        AND ABS(
            DINT_TO_LREAL(Axis.pIoPositionActualValue^) * Axis.rPositionFactor
            - Axis.ActualPosition
        ) <= 0.01;

    CSP_Data.Axis[PlatformAxis].ActualPosition :=
        DINT_TO_LREAL(Axis.pIoPositionActualValue^) * Axis.rPositionFactor;

    CSP_Data.Axis[PlatformAxis].FollowingError :=
        DINT_TO_LREAL(CSP_Data.FollowingErrorRaw[PlatformAxis]) * Axis.rPositionFactor;

END_IF

CSP_Data.Axis[PlatformAxis].FollowingErrorRaw :=
    CSP_Data.FollowingErrorRaw[PlatformAxis];

// Keep the blocking condition visible even before TCP feedback is valid.
IF NOT Axis.SDOsInitialized THEN CSP_Data.Axis[PlatformAxis].ReadinessCode := 2;
ELSIF NOT (Axis.rPositionFactor > 0) THEN CSP_Data.Axis[PlatformAxis].ReadinessCode := 3;
ELSIF (Axis.SlaveConnectorFlags AND 16#3F0) <> 16#F0 THEN CSP_Data.Axis[PlatformAxis].ReadinessCode := 4;
ELSIF NOT CSP_Data.Axis[PlatformAxis].Ready THEN CSP_Data.Axis[PlatformAxis].ReadinessCode := 5;
ELSE CSP_Data.Axis[PlatformAxis].ReadinessCode := 0;
END_IF
CSP_ReadAxis := CSP_Data.Axis[PlatformAxis].Ready;
