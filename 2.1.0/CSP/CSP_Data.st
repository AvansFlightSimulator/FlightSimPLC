{attribute 'qualified_only'}
VAR_GLOBAL
    Axis : ARRAY[1..6] OF CSP_AxisData;
    CycleCounter : UDINT;
    // TCP writes odd generation, complete target, then even generation. The
    // higher-priority motion task only accepts matching even generations.
    Generation : UDINT;
    ReceivedPosition : ARRAY[1..6] OF LREAL;
    ReceivedMode : SINT;
    ReceivedTick : UDINT;
    Connected : BOOL;
    Enable : BOOL;
    Reset : BOOL;
    StopRequested : BOOL;
    ResetRequired : BOOL := TRUE;
    Homing : BOOL; // park at HomePosition using the existing encoder reference
    ReadyToStart : BOOL;
    OwnsOutputs : BOOL; // FALSE: vendor PP writer; TRUE: direct CSP writer
    PlatformState : PlatformControlState := PlatformControlState.PP_RESET;
    PPReleased : BOOL; // published by PP task only after its last complete call
    // Published by the CSP task only after it restored mode 1 and stopped writing.
    // PP clears PPReleased in response; CSP then clears this flag.
    CSPReleased : BOOL;
    PPMoveDone : ARRAY[1..6] OF BOOL;
    AllPPMoveDone, AllStationary, AllCSPModeConfirmed : BOOL;
    // The exact PP-side handover condition, evaluated in the PP task.
    PPStartupComplete : BOOL;
    // First axis whose mode display differs from the requested handover mode
    // (8 during CSP_HANDOVER, 1 during PP_HANDBACK); 0 = all confirmed.
    ModeBlockingAxis : INT;
    InitialTargetRaw : ARRAY[1..6] OF DINT;
    // FaultCode that Reset acknowledged before ownership returned to PP.
    LastCSPFaultCode : UINT;
    Active : BOOL;
    TransitionState : UINT;
    Fault : BOOL;
    FaultCode : UINT;
    // Frozen at timeout, before quick stop changes the live drive states.
    TimeoutState : UINT;
    TimeoutStatusword : ARRAY[1..6] OF WORD;
    TimeoutControlword : ARRAY[1..6] OF WORD;
    // 0 running/transitioning; 1 commissioning; 2 axis readiness; 3 safety;
    // 4 operator Start; 5 fresh PC target; 6 drive mode; 7 fault (see FaultCode).
    // While PP owns the drives: 10 PP not in STARTED; 11 not stationary;
    // 12 PP move still executing / no drive Standstill (FlightSimulator.isReady);
    // 13 an axis is outside HomeTolerance of HomePosition; 14 MC_MoveAbsolute
    // Error/CommandAborted. PPMoveDone[1..6] shows the axis.
    StartupBlockReason : UINT;
    ReversalDeferred : BOOL;
    // Map 0x60F4:00 (DINT) from each drive here; not computed PC error.
    FollowingErrorRaw : ARRAY[1..6] OF DINT;
END_VAR
