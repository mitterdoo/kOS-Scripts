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

function display_impact_and_burn_times
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

	local TARGET_COAST_SPEED is 10 .

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

	lock throttle to 1 .
	unlock steering .
	sas on .
	wait 0.001 .
	set sasmode to "RETROGRADE" .

	// local pid is pidloop(-0.45, -0.035, 0) .
	
	local pid is pidloop(5, 0, 0) .
	local cycles_pid_was_stable is 0 .

	until cycles_pid_was_stable > 50
	{
		// figure out deltav needed to get to target vel
		// find out how much thrust we're capable of, in m/s^2 (max_accel)
		
		// m/s
		local delta_v is ship:velocity:surface:mag - TARGET_COAST_SPEED .

		if max_accel = 0
		{
			print("max accel is 0") .
			break .
		}
		// max_accel is m/s^2
		print("----====----") at (0, 3).
		print("burn_time: " + round(burn_time, 2) + "     ") at (0, 4).

		set pid:setpoint to 0 .
		local pid_value is -pid:update(time:seconds, burn_time) .
		if abs(pid:changerate) < 0.001
		{
			set cycles_pid_was_stable to cycles_pid_was_stable + 1 .
		}
		else
		{
			set cycles_pid_was_stable to 0 .
		}
		print("pid_value: " + round(pid_value, 2) + "     ") at (0, 5).
		print("pid changerate: " + round(pid:changerate, 5) + "        ") at (0, 6) .
		lock throttle to pid_value .

		wait 0.001 .
	}
	print("stable. now waiting for impact again") .
	lock throttle to 0 .

	set TARGET_COAST_SPEED to 6 .

	until burn_time * .65 >= impact_time
	{
		print("IMPACT TIME: " + round(impact_time, 1) + "s     ") at (0, 1).
	 	print("BURN TIME:   " + round(burn_time, 1) + "s     ") at (0, 2).
		wait 0 .
	}


	until ship:status = "LANDED"
	{
		// figure out deltav needed to get to target vel
		// find out how much thrust we're capable of, in m/s^2 (max_accel)
		
		// m/s
		local delta_v is ship:velocity:surface:mag - TARGET_COAST_SPEED .

		if max_accel = 0
		{
			print("max accel is 0") .
			break .
		}
		// max_accel is m/s^2
		print("----====----") at (0, 3).
		print("burn_time: " + round(burn_time, 2) + "     ") at (0, 4).

		set pid:setpoint to 0 .
		local pid_value is -pid:update(time:seconds, burn_time) .
		print("pid_value: " + round(pid_value, 2) + "     ") at (0, 5).
		print("pid changerate: " + round(pid:changerate, 5) + "        ") at (0, 6) .
		lock throttle to pid_value .

		wait 0.001 .
	}
	print("done") .

	lock throttle to 0 .
	unlock throttle .

	
	// until ship:status = "LANDED"
	// {
	// 	local impact_time is addons:tr:timetillimpact .
	// 	print(addons:tr:impactvel) at (0, 10) .
	// 	local impact_velocity is addons:tr:impactvel:mag .
	// 	local impact_altitude is addons:tr:impactpos:terrainheight .
	// 	local orbit_atmosphere is ship:body:atm .
	// 	local impact_pressure is orbit_atmosphere:altitudepressure(impact_altitude) .
	// 	local max_accel is ship:maxthrustat(impact_pressure)/ship:mass.
	// 	local burn_time is ship:velocity:surface:mag/max_accel.

	// 	print("IMPACT TIME: " + round(impact_time, 1) + "s     ") at (0, 0).
	// 	print("BURN TIME:   " + round(burn_time, 1) + "s     ") at (0, 1).
	// 	print("IMPACT ? VEL: " + round(impact_velocity, 1) + "m/s     ") at (0, 3).
	// 	print("CUR SURFACE VEL:    " + round(ship:velocity:surface:mag, 1) + "m/s     ") at (0, 4).
	// 	
	// 	
	// }


}

display_impact_and_burn_times() .
