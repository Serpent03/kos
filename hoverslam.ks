clearscreen.

//coordinates
run navballLIB.
set kscLaunchPad to latlng(-0.0972092543643722, -74.557706433623).
addons:tr:settarget(kscLaunchPad).

// well not exactly cursive but the curved font style = I like very much

// now to make a PID loop of the expected coordinates v/s trajectory coordinates. Maybe something can be done with the trajectories mod (?)
//lock shipLatLong to ship:GEOPOSITION.	// working on delta


//  lowest part
local function actualHeight {
	local bounds_box is ship:bounds.
	return bounds_box:bottomaltradar.
}

//Trajectory calculations

function getBearingFromAtoB {
    // B is target
    // A is impact pos from trajectories.
    
    set deltaLNG to kscLaunchPad:lng - addons:tr:impactpos:lng.
    set x to cos(kscLaunchPad:lat) * sin (deltaLNG).
    set y to cos(addons:tr:impactpos:lat) * sin(kscLaunchPad:lat) - sin (addons:tr:impactpos:lat) * cos(kscLaunchPad:lat) * cos(deltaLNG).

    return arcTan2(x,y).

}

function getDelta {
    lock delta to (sqrt((((addons:tr:impactpos:lat - kscLaunchPad:lat)^2)*10448.6454) + (((addons:tr:impactpos:lng - kscLaunchPad:lng)^2)*8272.80113)))*10000.
    return round(delta).
}

function getTwr {
	return g * (ship:mass/ship:availableThrust).
}

function descentAOA_X {
	return min(10, max(-10, (getBearingFromAtoB()))).
}


function descentAOA_Y {
	return -min(10, max(-10, ((addons:tr:impactpos:lat - kscLaunchPad:lat)*10448.6454)*10000)).
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
		
	if runmode = 10 {
		
		wait until stage:deltaV:ASL <= 700. 
			set ship:control:pilotmainthrottle to 0.
			set runmode to 11.
	}

	if runmode = 11 or eta:apoapsis <> 0{
		wait until ship:verticalspeed < 100.
		set runmode to 19.
	}

	if runmode = 19 {// adjust trajectory
		set steeringmanager:rollcontrolanglerange to -1.
		RCS on. SAS off.
		set error to getDelta().
		set bearing to getBearingFromAtoB().
		lock angularMag to ship:angularVel:mag.
		lock steering to heading(bearing, 30, 0).
		wait 2.
		if error >= 10000{
			if angularMag < 0.009 {
				lock throttle to min(4.5 * getTwr(), sqrt(error)/900 * getTwr()).
			}
		}
		if error <= 20000{
			set runmode to 20.
			set steeringmanager:rollcontrolanglerange to 1.
		}
	}

	if runmode = 20 {
		print 20.
		lock steering to srfRetrograde.
		lock trueRadar to actualHeight().
		lock throttle to 0.
		BRAKES on.
		RCS on.
		set runmode to 21.
	}

	if runmode = 21{	// guidance.
		until getDelta() < 200 {lock steering to R(descentAOA_Y(), descentAOA_X(), 0) * srfRetrograde.}
		lock steering to srfRetrograde.
		print 21.
		if trueRadar <= stopDist. {
			set runmode to 22.
			lock throttle to idealThrottle.
			print 22.	
		}
	}

	if runmode = 22 {	// declare final-descent.
	
		when impactTime < 6 then {
			GEAR on.
		}

		//when trueRadar < .3 then { //prevent falling.
		//	lock steering to up.
		//}

		wait until ship:verticalspeed > -0.001.
			set ship:control:pilotmainthrottle to 0.
			rcs off.
			wait 1.
			unlock steering.
			clearScreen.
			set runmode to 0.
	}
	
	wait 0.001.

}