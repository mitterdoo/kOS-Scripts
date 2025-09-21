runpath("justin/jst_orbital.ks") .
function maintain_target_apoapsis
{
	parameter TARGET .

	lock throttle to 0 .
	unlock steering .
	sas ON.
	wait 0.001.
	set sasmode to "PROGRADE" .
	until ship:q = 0
	{
		if ship:apoapsis < TARGET
		{
			lock throttle to 0.1 .
		}
		else
		{
			lock throttle to 0 .
		}
	}
}

// returns deltav used (in a vacuum)
function generic_launch_and_circularize
{
	parameter TARGET_APOAPSIS .

	local start_deltav is ship:deltav:vacuum .

	generic_launch(TARGET_APOAPSIS) .
	maintain_target_apoapsis(TARGET_APOAPSIS) .
	lock throttle to 0 .
	set circ_node to get_circular_orbit_node() .
	add circ_node .
	execute_node() .

	return start_deltav - ship:deltav:vacuum .

}

function execute_node
{
	if not HASNODE
	{
		return 1 .
	}

	unlock steering .
	sas ON.
	set sasmode to "MANEUVER".

	local lock max_accel to ship:maxthrust/ship:mass. // in deltav per second (or m/s^2)
	local burn_time is NEXTNODE:deltav:mag/max_accel.

	wait until NEXTNODE:eta <= (burn_time/2).

	local done is false.
	local initial_deltav is NEXTNODE:deltav.

	local pid is pidloop(0.2, 0, 0.1, -1, 1) .
	set pid:setpoint to 0 .
	lock throttle to -pid:update(time:seconds, NEXTNODE:deltav:mag) .
	clearscreen .
	until vdot(initial_deltav, NEXTNODE:deltav) < 0
	{
		// local x is -pid:update(time:seconds, NEXTNODE:deltav:mag) .
		// lock throttle to x .
		print("pid_value:   " + round(pid:output, 2) + "    ") at (0, 0) .
		print("pid:change:  " + round(pid:changerate, 3) + "    ") at (0, 1) .
		print("pid:iterm:   " + round(pid:iterm,3) + "    ") at (0, 2) .
		print("pid:dterm:   " + round(pid:dterm,3) + "    ") at (0, 3) .
		print("pid:setpoint:" + round(pid:setpoint, 1) + "    ") at (0, 4) .
		print("pid:input:   " + round(pid:input, 1) + "    ") at (0, 5) .
		print("pid:error:   " + round(pid:error, 2) + "    ") at (0, 6) .
		print("> burn_time: " + round(NEXTNODE:deltav:mag / max_accel, 2) + "    ") at (0, 7) .
		wait 0.001 .
	}
	lock throttle to 0 .
	// lock throttle to 1.
	// wait until NEXTNODE:deltav:mag < 50.
	// lock throttle to NEXTNODE:deltav:mag / 50 + 0.01.
	// wait until vdot(initial_deltav, NEXTNODE:deltav) < 0.
	// lock throttle to 0.

	remove NEXTNODE.
	return 0 .
}
