runpath("justin/jst_orbital.ks") .
// may need to include the below external library to correctly calculate burn duration for maneuvers
// for now i'm just doing the naive way, calculating for current stage only and not accounting for mass loss
// (the library requires the user to manually insert kOS part tags on fuel ducts for asparagus staging)
// runpath("quetschke-kOSutil/sinfo.ks") .
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
	local lock burn_time to NEXTNODE:deltav:mag/max_accel.

	wait until NEXTNODE:eta <= (burn_time/2).

	local done is false.
	local initial_deltav is NEXTNODE:deltav.

	// when 1 second left of burn time, slow down throttle until we're doing 0.1 m/s^2 of thrust exactly
	local lock target_throttle to burn_time^0.8.
	lock throttle to max(0.1 / max_accel, target_throttle) .

	wait until vdot(initial_deltav, NEXTNODE:deltav) < 0 .
	lock throttle to 0 .

	remove NEXTNODE.
	return 0 .
}
