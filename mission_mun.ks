
// MISSION PHASES
// 0		Launchpad
// 1		Burn to raise A

runpath("mylib.ks") .
runpath("land.ks") .

function main
{
	// Phase LAUNCHPAD -> ENTER_ORBIT
	// WHEN stage has actual deltav
	print("Activate a stage with a deltaV > 0 to launch.") .
	wait until stage:deltav:current > 0 .

	when (stage:deltav:current < 0.1 and ship:stagenum > 3) then
	{
		stage .
		return true .
	}
	local launch_deltav_spent is generic_launch_and_circularize(80000) .
	print("Launch complete. used " + round(launch_deltav_spent, 1) + "m/s dV") .
	
	// Phase ENTER_ORBIT -> FOLLOW_MANEUVER_NODES
	// maneuver nodes are an exercise left for the reader (:

	
	// Phase MUN_LAND -> MUN_DEPART_ORBIT
	// WHEN GEAR is off

}
//main() .
when (stage:deltav:current < 0.1) then
{
	stage .
}
set circ_node to get_circular_orbit_node() .
add circ_node .
execute_node() .
