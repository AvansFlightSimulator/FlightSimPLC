{attribute 'qualified_only'}
VAR_GLOBAL CONSTANT
    CONTROL_MODE_PP : SINT := 1;
    CONTROL_MODE_CSP : SINT := 8;
    // Match BuildMode.h. PP remains the initial commissioning selection.
    ControlMode : SINT := 1;
    // Also set EtherCAT_Task, master cycle and slave DC Sync0 to this interval.
    CycleSeconds : LREAL := 0.004;
    MinimumPosition : LREAL := 10.0;
    MaximumPosition : LREAL := 390.0;
    MaximumVelocity : LREAL := 25.0; // mm/s; commissioning starting values
    MaximumAcceleration : LREAL := 50.0; // mm/s^2
    BrakingAcceleration : LREAL := 50.0; // <= MaximumAcceleration
    MaximumJerk : LREAL := 250.0; // mm/s^3
    CommunicationTimeout : TIME := T#500MS;
    TransitionTimeout : TIME := T#10S;
    StandstillVelocity : LREAL := 0.05;
    HomePosition : LREAL := 200.0;
    HomeTolerance : LREAL := 0.5; // measured user units, all six axes
    HomeSettleTime : TIME := T#200MS;
    HomeTimeout : TIME := T#60S;
    // Set TRUE only after the eight required PDOs, DC, interpolation period,
    // actual task timing and all six existing position conversions are checked.
    Commissioned : BOOL := FALSE;
END_VAR
