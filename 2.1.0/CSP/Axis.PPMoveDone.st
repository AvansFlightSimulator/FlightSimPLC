METHOD PPMoveDone : BOOL
(* IMPLEMENTATION *)
// Level-based PP completion, as recorded by the original Axis.cycle():
// doMovement() sets execute, and cycle() clears it only after MC_MoveAbsolute
// Done (or CommandAborted/axis error). isReady() = drive Standstill AND NOT
// execute. The transient Done output itself is not used. The caller must also
// check the reached position, which excludes a move cleared by abort.
PPMoveDone := THIS^.isReady() AND NOT THIS^.error
    AND NOT THIS^.MC_MoveAbsolute.Error AND NOT THIS^.MC_MoveAbsolute.CommandAborted;
