// ALTS - Automatic Lander Touchdown System

clearScreen.

// target
set targetHoverslam to latlng(-0.0972092543643722, -74.557706433623).
addons:tr:settarget(targetHoverslam).

// initial variables

set runmode to 1.
lock trueRadar to actualHeight().
lock g0 to constant:g * ship:body:mass/ship:body:radius^2.
lock shipAcc to (ship:availableThrust/ship:mass) - g0.
lock decelHeight to (ship:verticalspeed^2/(2 * shipAcc)) * 4.
lock throtVal to decelHeight/trueRadar.
lock errorDistance to haversineDistance(addons:tr:impactpos, targetHoverslam).

// calculation function references 

declare local function actualHeight {	// get actual height from KSP collision box system
	local bounds_box is ship:bounds.
	return bounds_box:bottomaltradar.
}

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

declare local function descentAOA_X {	// TDAG EW - trajectory discrepancy avoidance guidance East <-> West.
    if addons:tr:hasimpact {
	    return min((sqrt(errorDistance)), max(-(sqrt(errorDistance)), (getBearingFromAtoB()))).
    }
    else {
        return 0.
    }
}


declare local function descentAOA_Y {	// TDAG NS -trajectory discrepancy avoidance guidance for North <-> South.
    if addons:tr:hasimpact {
	    return -min(sqrt(errorDistance)*2, max(-sqrt(errorDistance)*2, ((addons:tr:impactpos:lat - targetHoverslam:lat)*10448.6454))).
    }
    else {
        return 0.
    }
}

declare local function getTwr {	// get TWR for first correctional burn
	return g0 * (ship:mass/ship:availableThrust).
}

declare local function printfunc {
    print "AUTOMATIC LANDER TOUCHDOWN SYSTEM(ALTS)" at (2,5).
    print "=======================" at (2,7).
    print "Height above terrain: " + round(trueRadar, 1) + "m      " at (2,9).
    print "Vertical velocity: " + round(abs(ship:verticalspeed),1) + "m/s      " at (2,11).
    print "Horizontal velocity: " + round(ship:groundspeed,1) + "m/s      " at (2,13).
    print "DeltaV remaining: " + round(stage:deltaV:current,1) + "m/s      " at (2,15).
    print "Error in distance: " + round(haversineDistance(addons:tr:impactpos, targetHoverslam),1) + "m      " at (2,17).
    print "=======================" at (2,21).
    print sqrt(haversineDistance(addons:tr:impactpos, targetHoverslam))/10 at (2,23).
}

// Actual code

until runmode = 0 {
    printfunc().

    if runmode = 1 {
        lock throttle to 4 * getTwr().
        lock steering to srfRetrograde.
        print "Status: IVRP - Initial Velocity Reduction Phase          " at (2, 19).        
        when abs(ship:verticalspeed) < 50 and ship:groundspeed < 50 then {
            set runmode to 2.
        }
    }

    if runmode  = 2 {
        lock steering to -R(descentAOA_Y(), descentAOA_X(), 0) * UP.
        print "Status: MVRP - Midterm Velocity Redsuction Phase          " at (2, 19).
        lock throttle to sqrt(errorDistance)/7 * getTwr().
        set runmode to 3.
    }

    if runmode = 3 and trueRadar < decelHeight {

        print "Status: FDGP - Final Descent Guidance Phase              " at (2, 19).
        lock throttle to throtVal.
        
        when ship:verticalspeed > -2 then {
            lock steering to srfRetrograde.
        }
        when ship:verticalspeed > -0.01 then{
            lock throttle to 0.
            set runmode to 0.
        }
    }

    wait 0.15.
}