clearscreen.

//	coordinates
set FIWLT to latlng(-0.167535441286375, -68.545775219406). // -> downrange barge coordinates. "Funny, it worked last time.."
set kscPad to latlng(-0.0972092543643722, -74.557706433623).	// -> RTLS return to launch site coordinates
set padA to latlng(-0.185992338973754, -74.5185665785612).	// ignore padA and padB, or set these to your own custom SpaceX pad A and pad B for recovery. These coordinates are custom to my system :P
set padB to latlng(-0.153670012718547, -74.5215027684729).

set targetHoverslam to padA.	// set which hoverslam target to use.
addons:tr:settarget(targetHoverslam).


// calculations
set runmode to 1.
lock trueRadar to addons:tr:impactpos:distance.	// Offset radar to get distance from gear to ground ->> deprecated
lock g to constant:g * ship:body:mass / ship:body:radius^2.		// Gravity (m/s^2, g0 = GM/r^2)
lock maxDecel to (ship:availablethrust / ship:mass) - g.	// Maximum deceleration possible (m/s^2)
lock stopDist to ship:verticalspeed^2 / (2 * maxDecel).		// The distance the burn will require
lock idealThrottle to stopDist / trueRadar * 1.4.// * 0.95.			// Throttle required for perfect hoverslam
lock impactTime to trueRadar / abs(ship:verticalspeed).		// Time until impact, used for landing gear
lock errorDistance to haversineDistance(addons:tr:impactpos, targetHoverslam).
lock twoDimErrorDistance to getDelta(addons:tr:impactpos, targetHoverslam).
lock masterAOA to sqrt(twoDimErrorDistance)/5.

// ---{ CONSTANTS }---

//  lowest part
local function actualHeight {	// get actual height from KSP collision box system
	local bounds_box is ship:bounds.
	return bounds_box:bottomaltradar.	// add 1 + 1/2 meter for safety
}

// engine list

list ENGINES in engineList.

// ---{ CALCULATIONS }---

//Trajectory calculations

declare local function getBearingFromAtoB {	// get vector to heading(magnetic) between the predicted impact point and targeted impact point
    // B is target
    // A is impact pos from trajectories.
    if ADDONS:TR:HASIMPACT {

		local deltaLNG is targetHoverslam:lng - addons:tr:impactpos:lng.
		local x is cos(targetHoverslam:lat) * sin (deltaLNG).
		local y is cos(addons:tr:impactpos:lat) * sin(targetHoverslam:lat) - sin (addons:tr:impactpos:lat) * cos(targetHoverslam:lat) * cos(deltaLNG).

		return arcTan2(x,y).
	}

	else {
		return 0.
	}
}

declare local function getDelta {	// distance between predicted trajectory impact point and target trajectory impact point

	if addons:tr:hasimpact {// using the formula for distance between two points = sqrt((ax - bx)^2 + (ay - by)^2). multiply by 10448 and 8272 to convert to meters
		local parameter geoPointA.	
		local parameter geoPointB.
		
		// get the distance. Accurate by about 95-96% due to Kerbin not being a 2D geometry. Haversine distance returns 100% accurate.

		local delta is (sqrt((((geoPointA:lat - geoPointB:lat)^2)*10448.6454) + (((geoPointA:lng - geoPointB:lng)^2)*8272.80113)))*1000.
		return delta.
	}
	else {
		return 0.
	}
}

declare local function haversineDistance {

	if addons:tr:hasimpact {

		local parameter geoPointA.
		local parameter geoPointB.

		//convert to radians for haversine function
		local geoPointALat is geoPointA:lat * (constant:pi / 180).
		local geoPointALng is geoPointA:lng * (constant:pi / 180).
		local geoPointBLat is geoPointB:lat * (constant:pi / 180).
		local geoPointBLng is geoPointB:lng * (constant:pi / 180).


		//calculate the first haversine function
		local dLat is geoPointBLat - geoPointALat.
		local dLon is geoPointBLng - geoPointALng.

		local havLatitude is sin(dLat/2)^2.
		local havLongitude is sin(dLon/2)^2.

		//distance from A to B. Travel from A -> B.

		local distanceStepOne is havLatitude + cos(geoPointALat) * cos(geoPointBLat) * havLongitude.
		local distanceStepTwo is arcSin(sqrt(distanceStepOne)).
		local distance is distanceStepTwo * ship:body:radius * 2.
		
		return distance.
	}
	else {
		return 0.
	}
}

declare local function getTwr {	// get TWR for first correctional burn
	return g * (ship:mass/ship:availableThrust).
}

declare local function descentAOA_X {	// TDAG EW - trajectory discrepancy avoidance guidance East <-> West.
	return min((masterAOA), max(-(masterAOA), (getBearingFromAtoB()))).
}


declare local function descentAOA_Y {	// TDAG NS -trajectory discrepancy avoidance guidance for North <-> South.
	return -min(masterAOA, max(-masterAOA, ((addons:tr:impactpos:lat - targetHoverslam:lat)*10448.6454))).
}

declare local function printfunc {
    print "AUTOMATIC LANDER TOUCHDOWN SYSTEM(ALTS)" at (2,5).
    print "=======================" at (2,7).
    print "Height above terrain: " + round(trueRadar, 1) + "m      " at (2,9).
    print "Vertical velocity: " + round(abs(ship:verticalspeed),1) + "m/s      " at (2,11).
    print "Horizontal velocity: " + round(ship:groundspeed,1) + "m/s      " at (2,13).
    print "DeltaV remaining: " + round(stage:deltaV:current,1) + "m/s      " at (2,15).
    print "Error in distance: " + round(errorDistance,1) + "m      " at (2,17).
    print "Runmode: " + (runmode) + "       " at (2,19).
    print "=======================" at (2,21).
}

declare local function activeEngine {
    for eng in engineList {
        if eng:ignition {
            return eng.
        }
    }
}

declare local function getLowerStageDeltaV {
	// dv = exhaustVelocity(isp * g0) * ln(mf/mi)

	local exhaustVelocity is activeEngine():isp * g.
	local deltaV is exhaustVelocity * ln(64200/25000).
	
	return deltaV.
}

//flight control configuration. reset this back when we reduce AoA.
set steeringManager:pitchpid:kd to 5.
set steeringManager:yawpid:kd to 5.
set steeringmanager:pitchts to 10.
set steeringmanager:yawts to 10.

// runmode configurations
// runmodes block 10 to 20 ascent
// runmodes block 20 to 30 descent

// ---{ MAIN FUNCTION }---


print getLowerStageDeltaV().
set runmode to 10.

until runmode = 0 {	

	printfunc().	
		
	if runmode = 10 { // initial -> check if ascending.
			
			set ship:control:pilotmainthrottle to 0.
			set runmode to 11.
	}

	if runmode = 11 or eta:apoapsis <> 0{	// if ascending, wait until apoapsis is less than 10 seconds away.
		wait until ship:verticalspeed < 200.
		set runmode to 19.
	}

	if runmode = 19 {// adjust trajectory
		set steeringmanager:rollcontrolanglerange to -1.
		RCS on. SAS off.
		lock steering to heading(getBearingFromAtoB(), 10).
		set errorD to errorDistance.
		until errorD <= 500 {
			if ship:angularVel:mag < 0.009 {
				lock throttle to min(4.5 * getTwr(), sqrt(errorDistance)/100 * getTwr()).
			}
		}
		set runmode to 20.
	}

	if runmode = 20 {	// configure for ballistic guidance
		toggle AG3.
		set steeringmanager:rollcontrolanglerange to 1.
		lock trueRadar to actualHeight().
		lock throttle to 0.
		lock steering to srfRetrograde.
		BRAKES on.
		RCS on.
		set runmode to 21.
	}

	if runmode = 21{	// ballistic guidance.
		lock steering to R(descentAOA_Y(), descentAOA_X(), 0) * srfRetrograde. // offset from retrograde to adjust ship trajectory
		if trueRadar <= stopDist{ // when engine burn starts
			lock steering to -R(descentAOA_Y(), descentAOA_X(), 0) * srfRetrograde.	// point ship in the inverse offset direction once motor burns.
			lock throttle to idealThrottle.
			set runmode to 22.
		}
	}

	if runmode = 22 {	// declare final-descent.


		set steeringManager:pitchpid:kd to 2.
		set steeringManager:yawpid:kd to 2.
		set steeringmanager:pitchts to 1.
		set steeringmanager:yawts to 1.
		set steeringmanager:rollts to 1.
		
		when impactTime < 3 then  {
			lock masterAOA to 6.	// reduce offset limit to avoid over-correction
			GEAR on.			
		}

		//when impactTime < .9 then  {
		//	lock steering to srfRetrograde.
		//}
		
		when ship:groundspeed < 1 then {
			lock steering to up.
		}

		wait until ship:verticalspeed > -0.01.
			lock throttle to 0.
			clearScreen.
			set runmode to 0.
			wait 2.
			BRAKES off.
	}	
	wait 0.15.

}