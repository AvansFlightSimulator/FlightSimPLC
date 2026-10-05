TYPE CSP_AxisData :
STRUCT
    RequestedPosition : LREAL;
    CommandPosition : LREAL;
    CommandVelocity : LREAL;
    CommandAcceleration : LREAL;
    ActualPosition : LREAL;
    ActualVelocity : LREAL;
    FollowingError : LREAL;
    PositionFactor : LREAL;
    VelocityFactor : LREAL;
    ActualRaw : DINT;
    TargetRaw : DINT;
    FollowingErrorRaw : DINT;
    ModeRequested : SINT;
    ModeActual : SINT;
    Statusword : WORD;
    Controlword : WORD;
    CiA402State : WORD;
    PositionLimit : BOOL;
    CommunicationTimeout : BOOL;
    Ready : BOOL;
    // 0 ready; 1 PDO pointer; 2 SDO init; 3 factor; 4 bus; 5 conversion mismatch.
    ReadinessCode : UINT;
    VelocityAllowed : LREAL;
END_STRUCT
END_TYPE
