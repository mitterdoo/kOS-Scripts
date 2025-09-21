// ---- Helpers: calculate density at given height
function air_density 
{
    parameter p_alt is ship:altitude .
    // pressure (atm -> pa):
    local p_atm is ship:body:atm:altitudepressure(p_alt) .
    local p_pa  is p_atm * constant:atmtokpa * 1000 .
    // temperature (k):
    local t     is ship:body:atm:alttemp(p_alt) .
    if (not ship:body:atm:exists) or (p_atm <= 0) or (t <= 0) 
    {
        return 0.
    }.
    // molar mass (kg/mol) and universal gas constant (j/mol/k):
    local m     is ship:body:atm:molarmass .
    local ru    is constant:idealgas .
    // density ρ = p*m / (r*t). if above atmo, p_atm=0 → ρ=0:
    local rho   is (p_pa * m) / (ru * t) .
    return rho.
}.


function follow_guidance_vector
{
	if not addons:tr:available
	{
		print "Trajectories mod not available" .
		return .
	}
	if not addons:tr:hastarget or not addons:tr:hasimpact
	{
		print "Must have an impact and target trajectory." .
		return .
	}

	clearscreen .
	print "Following guidance vector..." .
	sas off .
	lock throttle to 0 .
	unlock throttle .
	lock steering to addons:tr:correctedvec .
	until alt:radar < 3000
	{
		local lock rho to air_density(ship:altitude) .
		local lock vel to ship:velocity:surface:mag .
		local lock q_value to 0.5 * rho * vel^2 .
		print "q: " + round(q_value, 0) at (0, 1) .

		wait 0.2 .
	}
	// parachutes?
	unlock steering .
	stage .
}

function lerp
{
	parameter a .
	parameter b .
	parameter frac .

	return a + (b - a) * frac .
}

function ilerp
{
	parameter a .
	parameter b .
	parameter value .

	return (value - a) / (b - a) .
}

function generic_land
{
	if not addons:tr:available
	{
		print "Trajectories mod not available" .
		return .
	}
	if not addons:tr:hasimpact
	{
		print "Must have an impact trajectory." .
		return .
	}
	clearscreen.

	local TARGET_COAST_SPEED is 2 . // m/s
	local TARGET_LAND_LEAD_TIME is 2 . // seconds to try and aim for above the ground

	local lock impact_time to addons:tr:timetillimpact .
	local lock impact_altitude to addons:tr:impactpos:terrainheight .
	local lock orbit_atm to ship:body:atm .
	local lock impact_pressure to orbit_atm:altitudepressure(impact_altitude) .
	local lock max_accel to ship:maxthrustat(impact_pressure)/ship:mass .
	local lock burn_time to (ship:velocity:surface:mag - TARGET_COAST_SPEED)/max_accel .
	until burn_time >= impact_time
	{
		print("IMPACT TIME: " + round(impact_time, 1) + "s     ") at (0, 0).
	 	print("BURN TIME:   " + round(burn_time, 1) + "s     ") at (0, 1).
		wait 0 .
	
	}
	
	print("TIME TO KILL VELOCITY") .
	gear on .

	// unlock steering .
	// sas on .
	// wait 0.001 .
	// set sasmode to "RETROGRADE" .
	sas off .

	// local pid is pidloop(-0.45, -0.035, 0) .
	
	local pid is pidloop(0.2, 1, 0, -1, 1) .
	
	local cycles_pid_was_stable is 0 .

	// lerp our target velocity from our current speed (at this moment) down to the targeted landing speed
	// frac is 0 when target should be the speed we just measured
	// frac is 1 when target should be landing speed
	// might need to make the lerp fraction nonlinear though.
	local ship_bounds is ship:bounds . // we COULD just do it the intended way of getting the gear part and doing that...but that's too much work
	local lock ship_radar to ship_bounds:bottomaltradar .
	local altitude_start is ship_radar .
	local speed_start is ship:velocity:surface:mag .
	until ship:status = "LANDED" // or not addons:tr:hasimpact
	{
		local altitude_finish is TARGET_LAND_LEAD_TIME * TARGET_COAST_SPEED .
		local altitude_frac is min(1, max(0, ilerp(altitude_start, altitude_finish, ship_radar))) .
		set altitude_frac to altitude_frac ^ 1.3.
		local target_vel is lerp(speed_start, TARGET_COAST_SPEED, altitude_frac) .

		// also need to cancel the horizontal component
	 	local ship_velocity_horizontal is ship:velocity:surface + ship:up():vector * -ship:verticalspeed .
		local ship_lnav_target is -ship:velocity:surface . // retrograde

		// scale the horizontal antivector down to nothing as we approach zero velocity, so we don't get into something unstable
		local antihoriz_minvel is TARGET_COAST_SPEED * 20 .
		local antihoriz_maxvel is TARGET_COAST_SPEED * 50 .
		local antihoriz_base is 0 .
		local antihoriz_frac is min(1, max(0, ilerp(antihoriz_minvel, antihoriz_maxvel, ship:velocity:surface:mag))) .
		local antihoriz_scale is antihoriz_frac * antihoriz_base .

		set ship_lnav_target to ship_lnav_target - ship_velocity_horizontal * antihoriz_scale. // this effectively repeats the horizontal component. if the multiplier was 0, it would be regular retrograde.

		lock steering to ship_lnav_target .
		
		print("----====----") at (0, 3) .

		set pid:setpoint to target_vel .
		local pid_value is pid:update(time:seconds, ship:velocity:surface:mag) .
		print("target_vel:     " + round(target_vel, 3) + "     ") at (0, 4).
		print("actual vel:     " + round(ship:velocity:surface:mag, 3) + "     ") at (0, 5) .
		print("error:          " + round(target_vel - ship:velocity:surface:mag, 3) + "     ") at (0, 6).
		print("pid_value:      " + round(pid_value, 2) + "     ") at (0, 7).
		print("pid changerate: " + round(pid:changerate, 5) + "        ") at (0, 8) .
		print("radar:          " + round(ship_radar, 3) + "     ") at (0, 9) .
		print("altitude_frac:  " + round(altitude_frac, 3) + "    ") at (0, 10).
		print("antihoriz_frac: " + round(antihoriz_frac, 3) + "    ") at (0, 11).
		print("antihoriz_scale:" + round(antihoriz_scale, 3) + "    ") at (0, 12) .

		local throttle_dot is vdot(ship_lnav_target:normalized, ship:facing:vector:normalized) .
		local throttle_dot_min is 180 - 5 .
		local throttle_dot_max is 180 - 15 .
		local throttle_frac is min(1, max(0, ilerp(throttle_dot_max, throttle_dot_min, throttle_dot*180))) .
		print("throttle_dot:   " + round(throttle_dot, 2) + "    ") at (0, 13) .
		print("throttle_frac:  " + round(throttle_frac, 3) + "    ") at (0, 14) .
		lock throttle to -pid_value * throttle_frac .
		

		wait 0.001 .
	}

	print("the eagle has landed.") .

	// TODO when finished landing, lock SAS or steering to the normal of the terrain below, in case of uneven terrain
	// stop for now
}

