METHOD cycle : BOOL
VAR_INPUT
    StartupOnly : BOOL := FALSE; // PP startup for automatic CSP handover
END_VAR
VAR
	// Counter to cycle through axis.
	iCounter: UINT;
END_VAR

(* IMPLEMENTATION *)
// If safety system has been triggered/broken go to error.
IF NOT THIS^.safetySystem THEN
	THIS^.state := stateFlightSimulator.error;
	THIS^.safetyStop();
END_IF

// If any Axis is in error stop all axis.
IF THIS^.inError() AND NOT (THIS^.state = stateFlightSimulator.error) THEN
	IF NOT (THIS^.state = stateFlightSimulator.resetting) THEN
		THIS^.state := stateFlightSimulator.error;
		THIS^.safetyStop();
	END_IF
END_IF

// Reset signal variables.
THIS^.signal.red.enable := FALSE;
THIS^.signal.yellow.enable := FALSE;
THIS^.signal.green.enable := FALSE;
THIS^.signal.red.blink := FALSE;
THIS^.signal.yellow.blink := FALSE;
THIS^.signal.green.blink := FALSE;

// Status case switch.
CASE THIS^.state OF
	stateFlightSimulator.stopped:
		// In STOPPED state:
		// Signal needs to be continuously yellow.
		// Drives needs to be disabled.
		// When enabled go to STARTING state.
		
		// Set signal.
		THIS^.signal.yellow.enable := TRUE;
		
		// Disable drives.
		IF THIS^.isEnabled() THEN
			FOR iCounter := 1 TO 6 BY 1 DO
				Axis[iCounter].enabled := FALSE;
			END_FOR
		END_IF
		
		// If flightsim is enabled go to STARTING state.
		IF THIS^.enabled THEN
			THIS^.state := stateFlightSimulator.starting;
		END_IF
		
	stateFlightSimulator.starting:
		// In STARTING state:
		// Signal needs to be blinking green.
		// Drives needs to be enabled.
		// Once enabled go to neutral position.
		// Once in neutral position go to STARTED state.
	
		// Set signal.
		THIS^.signal.green.enable := TRUE;
		THIS^.signal.green.blink := TRUE;
		
		// Start Axises.
		IF NOT THIS^.isEnabled() THEN
			FOR iCounter := 1 TO 6 BY 1 DO
				Axis[iCounter].enabled := TRUE;
				GVL.servoPositions[iCounter] := 200;
			END_FOR
		END_IF
		
		IF THIS^.isEnabled() AND THIS^.isReady() THEN
			IF NOT bToggle THEN
				// Move pistons to neutral position.
				THIS^.goNeutralPosition();
				bToggle := TRUE;
			ELSE
				// When position has been reached go to started state.
				bToggle := FALSE;
				THIS^.state := stateFlightSimulator.started;
			END_IF	
		END_IF
		
	stateFlightSimulator.started:
		// In STARTED state:
		// Signal needs to be continuously green.
		// When at least one of the drives goes to disabled go to ERROR state.
		// Do movement.
		// When not enabled go to STOPPING state.
	
		// Set signal.
		THIS^.signal.green.enable := TRUE;
		
		// If drives are disabled somehow go to error.
		IF NOT THIS^.isEnabled() THEN
			THIS^.state := stateFlightSimulator.error;
		END_IF
		
		// Do movement.
		// PP startup holds its completed neutral move until the CSP task takes over.
        IF NOT StartupOnly THEN
            THIS^.doMovement(THIS^.newPositions, THIS^.newVelocities);
        END_IF
		
		// If flightsim is disabled go to STOPPING state.
		IF NOT THIS^.enabled THEN
			THIS^.state := stateFlightSimulator.stopping;
		END_IF
		
	stateFlightSimulator.stopping:
		// In STOPPING state:
		// Signal needs to be blinking green.
		// Drives needs to be enabled.
		// Once enabled go to home position.
		// Once in home position go to STOPPED state.
	
		// Set signal.
		THIS^.signal.green.enable := TRUE;
		THIS^.signal.green.blink := TRUE;
		
		// Enable drives if not enabled yet.
		IF NOT THIS^.isEnabled() THEN
			FOR iCounter := 1 TO 6 BY 1 DO
				Axis[iCounter].enabled := TRUE;
			END_FOR
		END_IF
		
		IF THIS^.isEnabled() THEN
			IF NOT bToggle THEN
				// Move pistons to home position.
				THIS^.goHomePosition();
				bToggle := TRUE;
			ELSE
				IF THIS^.isReady() THEN
					// When position has been reached go to stopped state.
					bToggle := FALSE;
					THIS^.state := stateFlightSimulator.stopped;
				END_IF
			END_IF
		END_IF
		
	stateFlightSimulator.error:
		// In ERROR state:
		// Signal needs to be blinking red.
		
		// Set signal.
		THIS^.signal.red.enable := TRUE;
		THIS^.signal.red.blink := TRUE;
		
	stateFlightSimulator.resetting:
		// In RESETTING state:
		// Signal needs to be blinking yellow.
		// If in error drives needs to be reset.
		// Else go to home and go to STOPPED state.
	
		// Set signal.
		THIS^.signal.yellow.enable := TRUE;
		THIS^.signal.yellow.blink := TRUE;
		
		IF THIS^.inError() THEN
			// Reset drives.
			FOR iCounter := 1 TO 6 BY 1 DO
				Axis[iCounter].doReset();
			END_FOR
		ELSE
			// Enable drives if not enabled yet.
			IF NOT THIS^.isEnabled() THEN
				FOR iCounter := 1 TO 6 BY 1 DO
					Axis[iCounter].enabled := TRUE;
				END_FOR
			END_IF
			
			IF THIS^.isEnabled() THEN
				IF NOT bToggle THEN
					// Move pistons to home position.
					THIS^.goHomePosition();
					bToggle := TRUE;
				ELSE
					IF THIS^.isReady() THEN
						// When position has been reached go to stopped state.
						bToggle := FALSE;
						THIS^.state := stateFlightSimulator.stopped;
					END_IF
				END_IF
			END_IF
		END_IF
END_CASE

// Cycle Axis classes.
FOR iCounter := 1 TO 6 BY 1 DO
	THIS^.axis[iCounter].cycle();
END_FOR
// Cycle signal class.
THIS^.signal.cycle();
