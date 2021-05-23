clearscreen.

//coordinates

// well not exactly cursive but the curved font style = I like very much

// now to make a PID loop of the expected coordinates v/s trajectory coordinates. Maybe something can be done with the trajectories mod (?)

//set kscLaunchPad to latlng(-0.0972092543643722, -74.557706433623).
//lock shipLatLong to ship:GEOPOSITION.	// working on delta


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

// Alternate lowest part
local function actualHeight {
	local bounds_box is ship:bounds.
	return boounds_box:bottomaltradar.
}
// calculations
set runmode to 1.
lock trueRadar to addons:tr:impactpos:distance.	// Offset radar to get distance from gear to ground ->> deprecated
lock g to constant:g * ship:body:mass / ship:body:radius^2.		// Gravity (m/s^2, g0 = GM/r^2)
lock maxDecel to (ship:availablethrust / ship:mass) - g.	// Maximum deceleration possible (m/s^2)
lock stopDist to ship:verticalspeed^2 / (2 * maxDecel).		// The distance the burn will require
lock idealThrottle to stopDist / trueRadar * 1.2.// * 0.95.			// Throttle required for perfect hoverslam
lock impactTime to trueRadar / abs(ship:verticalspeed).		// Time until impact, used for landing gear


// runmode configurations
// runmodes block 10 to 20 ascent
// runmodes block 20 to 30 descent

until runmode = 0 {

	

	set runmode to 10.	// initial -> check if ascending.
		
	if runmode = 10 and ship:verticalspeed > 10 {
		print "Currently ascending" at (2,3).
		until stage:deltaV:current <= 650.
			set ship:control:pilotmainthrottle to 0.
			print "Reached hoverslam fuel criteria. Decouple.." at (2, 5).
			set runmode to 11.
	}

	if runmode = 11 or ship:verticalspeed < -5{
		wait until ship:verticalspeed < - 50.
			clearScreen.
			SAS off.
			set runmode to 20.
	}

	if runmode = 20 {
		if trueRadar > 25000 and trueRadar < 120000{	// check if entering atmosphere
			lock steering to srfRetrograde.
			BRAKES on.
		}
		if trueRadar < 25000{	// set trajectory conditions
			lock steering to srfRetrograde.
			RCS on.
			BRAKES on.
			lock trueRadar to actualHeight().
			set runmode to 21.
		}
	}
	if runmode = 21{	// declare mid-descent.

		LOCK steering to srfRetrograde.
		wait until trueRadar < stopDist.	
			lock throttle to idealThrottle.
			set runmode to 22.

	}
	if runmode = 22 {	// declare final-descent.
		when impactTime < 4 then {
			GEAR on.
		}

		when impactTime < 2 then { 
			lock steering to up. 	// prevent any fall
		}

		WAIT UNTIL ship:status = "landed".
			print "Hoverslam completed" at (2,3).
			set ship:control:pilotmainthrottle to 0.
			rcs off.
			wait 1.
			unlock steering.
			clearScreen.
			set runmode to 0.
	}
	
	wait 0.05.

}

	



