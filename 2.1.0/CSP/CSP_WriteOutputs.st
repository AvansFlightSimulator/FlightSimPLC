PROGRAM CSP_WriteOutputs
(* IMPLEMENTATION *)
// Sole final writer, called after CSP_Cycle in the EtherCAT task.
CSP_WriteAxis(Axis:=servo_drive_1,PlatformAxis:=1);
CSP_WriteAxis(Axis:=servo_drive_2,PlatformAxis:=2);
CSP_WriteAxis(Axis:=servo_drive_3,PlatformAxis:=3);
CSP_WriteAxis(Axis:=servo_drive_4,PlatformAxis:=4);
CSP_WriteAxis(Axis:=servo_drive_5,PlatformAxis:=5);
CSP_WriteAxis(Axis:=servo_drive_6,PlatformAxis:=6);
