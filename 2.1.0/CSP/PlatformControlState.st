{attribute 'qualified_only'}
{attribute 'strict'}
TYPE PlatformControlState :
(
    PP_RESET := 0,
    PP_STOPPED := 1,
    PP_STARTING := 2,
    CSP_HANDOVER := 3,
    CSP_READY := 4,
    CSP_RUNNING := 5,
    CSP_RECOVERY := 6,
    FAULT := 7,
    PP_HANDBACK := 8 // Reset: CSP brakes and restores mode 1, then PP resumes
);
END_TYPE
