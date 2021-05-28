clearscreen.
//set target to "spacex_barge1". // -> downrange barge target vessel.

//coordinates
//set targetHoverslam to latlng(target:geoposition:lat, target:geoposition:lng). // -> downrange barge coordinates. Barge non-functional
set targetHoverslam to latlng(-0.0972092543643722, -74.557706433623).	// -> RTLS return to launch site coordinates
addons:tr:settarget(targetHoverslam).

// now to make a PID loop of the expected coordinates v/s trajectory coordinates. Maybe something can be done with the trajectories mod (?)
//lock shipLatLong to ship:GEOPOSITION.	// working on delta


//  lowest part
local function actualHeight {	// get actual height from KSP collision box system
	local bounds_box is ship:bounds.
	return bounds_box:bottomaltradar - 0.5.	// add half a meter for safety
}

//Trajectory calculations

function getBearingFromAtoB {	// get vector to heading(magnetic) between the predicted impact point and targeted impact point
    // B is target
    // A is impact pos from trajectories.
    
    set deltaLNG to targetHoverslam:lng - addons:tr:impactpos:lng.
    set x to cos(targetHoverslam:lat) * sin (deltaLNG).
    set y to cos(addons:tr:impactpos:lat) * sin(targetHoverslam:lat) - sin (addons:tr:impactpos:lat) * cos(targetHoverslam:lat) * cos(deltaLNG).

    return arcTan2(x,y).

}

function getDelta {	// distance between predicted trajectory impact point and target trajectory impact point
    lock delta to (sqrt((((addons:tr:impactpos:lat - targetHoverslam:lat)^2)*10448.6454) + (((addons:tr:impactpos:lng - targetHoverslam:lng)^2)*8272.80113)))*10000.
    return round(delta).
}

function getTwr {	// get TWR for first correctional burn
	return g * (ship:mass/ship:availableThrust).
}

function descentAOA_X {	// TDAG EW - trajectory discrepancy avoidance guidance East <-> West.
	return min((masterAOA/2), max(-(masterAOA/2), (getBearingFromAtoB()))).
}


function descentAOA_Y {	// TDAG NS -trajectory discrepancy avoidance guidance for North <-> South.
	return -min(masterAOA, max(-masterAOA, ((addons:tr:impactpos:lat - targetHoverslam:lat)*10448.6454))).
}

// calculations
set runmode to 1.
lock trueRadar to alt:radar.	// Offset radar to get distance from gear to ground ->> deprecated
lock g to constant:g * ship:body:mass / ship:body:radius^2.		// Gravity (m/s^2, g0 = GM/r^2)
lock maxDecel to (ship:availablethrust / ship:mass) - g.	// Maximum deceleration possible (m/s^2)
lock stopDist to ship:verticalspeed^2 / (2 * maxDecel).		// The distance the burn will require
lock idealThrottle to stopDist / trueRadar * 1.2.// * 0.95.			// Throttle required for perfect hoverslam
lock impactTime to trueRadar / abs(ship:verticalspeed).		// Time until impact, used for landing gear
set masterAOA to 20.

// runmode configurations
// runmodes block 10 to 20 ascent
// runmodes block 20 to 30 descent

until runmode = 0 {

	set runmode to 10.	
		
	if runmode = 10 { // initial -> check if ascending.
		
		wait until stage:deltaV:current <= 800. 
			set ship:control:pilotmainthrottle to 0.
			set runmode to 11.
	}

	if runmode = 11 or eta:apoapsis <> 0{	// if ascending, wait until apoapsis is less than 10 seconds away.
		wait until ship:verticalspeed < 100.
		set runmode to 19.
	}

	if runmode = 19 {// adjust trajectory
		set steeringmanager:rollcontrolanglerange to -1.
		RCS on. SAS off.
		lock error to getDelta().
		lock bearing to getBearingFromAtoB().
		lock angularMag to ship:angularVel:mag.
		lock steering to heading(bearing, 30, 0).
		wait 2.
		if error >= 5000{	// initial trajectory configuration. Some error (upto 0.5km) is expected.
			if angularMag < 0.009 {
				lock throttle to min(4.5 * getTwr(), sqrt(error)/900 * getTwr()).
			}
		}
		if error <= 40000{
			set runmode to 20.
		}
	}

	if runmode = 20 {	// configure for ballistic guidance
		set steeringmanager:rollcontrolanglerange to 1.
		print 20.
		lock steering to srfRetrograde.
		lock trueRadar to actualHeight().
		lock throttle to 0.
		BRAKES on.
		RCS on.
		set runmode to 21.
	}

	if runmode = 21{	// guidance.
		lock steering to R(descentAOA_Y(), descentAOA_X(), 0) * srfRetrograde.
		print 21.
		wait until trueRadar <= stopDist. // when engine burn starts
			set runmode to 22.
			lock steering to -R(descentAOA_Y(), descentAOA_X(), 0) * srfRetrograde.	// point ship in the inverse offset direction once motor burns.
			lock throttle to idealThrottle.
			print 22.
	}

	if runmode = 22 {	// declare final-descent.
	
		when impactTime < 5 then {
			GEAR on.
			set masterAOA to 10.
		}
		
		when trueRadar < .7 then {.
			lock steering to up.
		}

		//when trueRadar < .3 then { //prevent falling.
		//	lock steering to up.
		//}

		wait until ship:verticalspeed > -0.01.
			lock throttle to 0.
			lock steering to up.
			clearScreen.
			set runmode to 0.
			wait 2.
	}
	
	wait 0.001.

}