// ALTS - Automatic Lander Touchdown System

clearScreen.
SAS off.
RCS on.
GEAR off.

// target
set desiredWaypoint to "KSC".
set waypointList to allWaypoints(). 

declare local function currentActiveWaypoint {
    for i in range (0, waypointList:length) {
        if waypointList[i]:name = desiredWaypoint {
            return i.
        }
    }
    return "No waypoint found! Make sure you enter the correct waypoint name.".
}

set targetHoverslam to waypointList[currentActiveWaypoint()]:geoposition.
addons:tr:settarget(targetHoverslam).

// initial variables

set runmode to 1.
lock trueRadar to actualHeight().
lock g0 to constant:g * ship:body:mass/ship:body:radius^2.
lock shipAcc to (ship:availableThrust/ship:mass) - g0.
lock decelHeight to (ship:verticalspeed^2/(2 * shipAcc)) * 5.
lock throtVal to decelHeight/trueRadar.
lock errorDistance to haversineDistance(targetHoverslam).

// calculation function references 


declare local function actualHeight {	// get actual height from KSP collision box system
	local bounds_box is ship:bounds.
	return bounds_box:bottomaltradar-0.3.
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

		local geoPointA is addons:tr:impactpos.
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
	return g0 * (ship:mass/ship:availableThrust).
}

declare local function hoverThrottle {
    local fg is ship:mass * g0.
    local angleToNormal is vAng(ship:facing:forevector, ship:up:forevector).
    local throttleConfig is fg / cos(angleToNormal) / ship:availableThrust.
    return throttleConfig + (-ship:verticalspeed).
}

declare local function printfunc {
    print "Autonomous Navigation &Upsampled Sanding(ANUS)" at (2,5).
    print "=======================" at (2,7).
    print "Height above terrain: " + round(trueRadar, 1) + "m      " at (2,9).
    print "Vertical velocity: " + round(abs(ship:verticalspeed),1) + "m/s      " at (2,11).
    print "Horizontal velocity: " + round(ship:groundspeed,1) + "m/s      " at (2,13).
    print "DeltaV remaining: " + round(stage:deltaV:current,1) + "m/s      " at (2,15).
    print "Error in distance: " + round(errorDistance,1) + "m      " at (2,17).
    print "Error vector: " + round(getBearingFromAtoB(),1) + "°      " at (2,19).
    print "=======================" at (2,23).
}

// Actual code

//until runmode = 2 {
//
//    printfunc().
//
//    lock throttle to 4 * getTwr().
//    lock steering to srfRetrograde.
//    print "Status: IVRP - Initial Velocity Reduction Phase          " at (2, 21).        
//    when abs(ship:verticalspeed) < 50 and ship:groundspeed < 50 then {
//        set runmode to 2.
//    }
//}
//
//until runmode = 3{
//    printfunc().
//
//    lock steering to heading(getBearingFromAtoB(), max(50, 90-(sqrt(errorDistance)*2)), 0).
//    print "Status: MTGP - Midterm Trajectory Guidance Phase          " at (2, 21).
//    lock throttle to hoverThrottle(). //max(.7, sqrt(errorDistance)/10) * getTwr().
//    if errorDistance < 100 {
//        set runmode to 3.
//    }
//
//}
//
//until runmode = 0 {
//    printfunc().
//
//    GEAR on.
//
//    print "Status: FDGP - Final Descent Guidance Phase              " at (2, 21).
//    lock throttle to  sqrt(errorDistance)/3 * getTwr().
//    lock steering to heading(getBearingFromAtoB(), max(30, 90-(sqrt(errorDistance)*2)), 0).
//
//    when trueRadar < decelHeight then{
//        lock throttle to throtVal.
//    }
//
//    when trueRadar <= 1.5 then {
//        lock steering to up.
//    }
//
//    when trueRadar <= 0 then{
//        set runmode to 0.
//        lock throttle to 0.
//        set ship:control:pilotmainthrottle to 0.
//        SAS on.
//    }
//}

set ship:control:pilotmainthrottle to 1.
until runmode = 0 {

   

    if runmode = 1 {
        printfunc().

        lock throttle to 4 * getTwr().
        lock steering to srfRetrograde.
        print "Status: IVRP - Initial Velocity Reduction Phase          " at (2, 21).        
        if abs(ship:verticalspeed) < 50 and ship:groundspeed < 50 {
            set runmode to 1.3.
        }
    }

    if runmode = 1.3 {
        printfunc().
        if trueRadar > 1000 {
            set runmode to 2.
        }
        else {
            print "Status: ACP - Altitude Correction Phase           " at (2, 21).
            lock steering to up.
            lock throttle to 1.
        }
    }

    if runmode = 1.7 {
        lock steering to srfRetrograde.
        
        when trueRadar < decelHeight then{
            lock throttle to throtVal.
        }

        when trueRadar <= 0 then{
            lock throttle to 0.
            set ship:control:pilotmainthrottle to 0.
            unlock steering.
            print "Status: ECR - Electric Charge Regeneration          " at (2, 21).
            wait until stage:electriccharge > 600.
            set runmode to 1.
        }
    }

    if runmode  = 2 {
        printfunc().

        lock steering to heading(getBearingFromAtoB(), 60, 0).
        print "Status: MTGP - Midterm Trajectory Guidance Phase          " at (2, 21).
        lock throttle to hoverThrottle(). //max(.7, sqrt(errorDistance)/10) * getTwr().
        
        if ship:groundspeed > 100 and errorDistance < 1000 {
            print "Status: RVFS - Reconfiguring velocity for safety              " at (2,21).
            set runmode to 1.
        }

        if trueRadar < 900 {
            set runmode to 1.3.
        }

        if stage:electriccharge < 100 {
            print "Status: RVFS - Charging EC bank for safety              " at (2,21).
            set runmode to 1.7.
        }
        
        if errorDistance < 100 {
            set runmode to 3.
        }

    }

    if runmode = 3 {
        printfunc().
        GEAR on.

        print "Status: FDGP - Final Descent Guidance Phase              " at (2, 21).
        lock throttle to  sqrt(errorDistance)/3 * getTwr().
        lock steering to heading(getBearingFromAtoB(), max(30, 90-(sqrt(errorDistance)*2)), 0).

        when trueRadar < decelHeight then{
            lock throttle to throtVal.
        }

        when trueRadar <= 1.5 then {
            lock steering to up.
        }

        when trueRadar <= 0 then{
            lock throttle to 0.
            set ship:control:pilotmainthrottle to 0.
            SAS on.
            set runmode to 0.
        }
    }

}