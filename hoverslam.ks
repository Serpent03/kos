clearscreen.

//coordinates

set kscLaunchPad to latlng(-0.0972092543643722, -74.557706433623).
lock shipLatLong to ship:GEOPOSITION.


//lowest part

list parts in partList.
set lp to 0.//lowest part height
set hp to 0.//hightest part height
for p in partList{
    set cp to facing:vector * p:position.
    if cp < lp 
	    set lp to cp.
    else if cp > hp	
	    set hp to cp.
}
set radarOffset to hp - lp.

// engine perf

list engines in engList.
for eng in engList {lock engIsp to eng:ISP.}


// calculations
set runmode to 1.
lock trueRadar to alt:radar - radarOffset.			// Offset radar to get distance from gear to ground
lock g to constant:g * ship:body:mass / ship:body:radius^2.		// Gravity (m/s^2, g0 = GM/r^2)
lock maxDecel to (ship:availablethrust / ship:mass) - g.	// Maximum deceleration possible (m/s^2)
lock stopDist to ship:verticalspeed^2 / (2 * maxDecel).		// The distance the burn will require
lock idealThrottle to stopDist / trueRadar * 1.1.			// Throttle required for perfect hoverslam
lock impactTime to trueRadar / abs(ship:verticalspeed).		// Time until impact, used for landing gear


until runmode = 0 {

	set runmode to 1.1.	// declare initial descent.
	

	if runmode = 1.1 { 
		if trueRadar > 75000{	// check if orbital
			print "Not in atmosphere yet." at (2,3).
			clearScreen.
		}
		if trueRadar < 45000 and ship:verticalspeed < -2 {	// set trajectory conditions
			RCS on.
			BRAKES on.
			set runmode to 1.2.
			clearScreen.
		}
	}
	if runmode = 1.2 and ship:verticalspeed < -2 {	// declare mid-descent.
		SAS off.
		LOCK steering to srfRetrograde.
		print "Configuring for precise hoverslam." at (2, 3).
		wait until trueRadar < stopDist.
			print "Performing hoverslam." at (2, 4).
			set oldDv to ship:deltaV:ASL.
			lock throttle to idealThrottle.
			set runmode to 1.3.

	}
	if runmode = 1.3 {	// declare final-descent.
		when impactTime < 4 then {
			GEAR on.
			clearScreen.
		}
		wait until ship:verticalspeed < 0.001.
		print "Hoverslam completed" at (2,3).
		clearScreen.
		set ship:control:pilotmainthrottle to 0.
		lock steering to UP.
		rcs off.
		wait 1.
		unlock steering.
		SAS on.
		wait 0.5.
		print "deltaV expended: " + round(oldDv - ship:deltaV:ASL) + "m/s" at (2, 7).
		print "ISP0: " + engIsp + "s" at (2,9).
		print "Decoupling autopilot.." at (2,3).
		wait 2.
		set runmode to 0.
	}

	wait 0.002.

}

	



