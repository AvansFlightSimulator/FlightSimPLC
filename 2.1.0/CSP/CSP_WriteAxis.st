FUNCTION CSP_WriteAxis : BOOL
VAR_INPUT
    Axis : REFERENCE TO AXIS_REF_CiA402_ETC_CMMT_AS;
    PlatformAxis : INT;
END_VAR
(* IMPLEMENTATION *)
IF PlatformAxis < 1 OR PlatformAxis > 6 OR NOT CSP_Data.OwnsOutputs THEN
    Axis.BeforeWriteOutputs();
    IF PlatformAxis >= 1 AND PlatformAxis <= 6 AND Axis.pIoModesOfOperation <> 0
        AND Axis.pIoControlword <> 0 AND Axis.pIoTargetPosition <> 0 THEN
        CSP_Data.Axis[PlatformAxis].ModeRequested := Axis.pIoModesOfOperation^;
        CSP_Data.Axis[PlatformAxis].Controlword := Axis.pIoControlword^;
        CSP_Data.Axis[PlatformAxis].TargetRaw := Axis.pIoTargetPosition^;
    END_IF
ELSE
    // These are the SAME PDO pointers initialized by the existing Festo device.
    // Do not run its PP writer and then race it from another IEC task.
    IF Axis.pIoControlword <> 0 AND Axis.pIoModesOfOperation <> 0 AND Axis.pIoTargetPosition <> 0 THEN
        Axis.pIoTargetPosition^ := CSP_Data.Axis[PlatformAxis].TargetRaw;
        Axis.pIoModesOfOperation^ := CSP_Data.Axis[PlatformAxis].ModeRequested;
        Axis.pIoControlword^ := CSP_Data.Axis[PlatformAxis].Controlword;
        IF Axis.pIoVelocityOffset <> 0 THEN Axis.pIoVelocityOffset^ := 0; END_IF
        IF Axis.pIoTorqueOffset <> 0 THEN Axis.pIoTorqueOffset^ := 0; END_IF
    END_IF
END_IF
