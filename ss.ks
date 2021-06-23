//starship atmospheric guidance program

// clearscreen
clearScreen.
run navballLIB.

// target
set lz to ship:geoposition.
local vabH1 to latlng(-0.0968232530104955, -74.6174196286953).
local padC to latlng(-0.185418964943541, -74.4728985609505).
local pad1A to latlng(-0.205692668852836, -74.4731087859307).
local pad1B to latlng(-0.195543224632151, -74.4852084228534).

set targethoverslam to (pad1B).

// initial variables
list engines in engineList.
set runmode to 1.
lock trueRadar to alt:radar - 20.74.
lock g0 to constant:g * ship:body:mass/ship:body:radius^2.
lock shipAcc to (ship:maxThrust/ship:mass) - g0.
lock decelHeight to (ship:verticalspeed^2/(2 * shipAcc)).
lock throtVal to decelHeight/trueRadar * 6.
lock errorDistance to distanceMag.
lock masterAoA to sqrt(errorDistance) * 2.9.

// PID functions

local rollReqPID to pidLoop(0.6, 0.15, 0.02, -25, 25).
set rollReqPID:setpoint to 0.

local pitchReqPID to pidLoop(0.6, 0.15, 0.02, -25, 25).
set pitchReqPID:setpoint to 0.

// calculation functions

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

declare local function getTwr {	// Throttle needed to maintain a specific TWR
	local throttleSetting is g0 * (ship:mass/ship:availableThrust).
    return throttleSetting.
}

declare local function distanceMag {
    if addons:tr:hasimpact {
        local mag is (addons:tr:impactpos:position - targethoverslam:position):mag.
        return mag.
    }
    else {
        return 0.
    }
}
declare local function rollPIDValues {
    if addons:tr:hasimpact{
        local guidanceError is (targethoverslam:lat - addons:tr:impactpos:lat)*1000.
        return guidanceError.
    }
    else {
        return 0.
    }
}

declare local function pitchPIDValues {
    if addons:tr:hasimpact{
        local guidanceError is (targethoverslam:lng - addons:tr:impactpos:lng)*1000.
        return guidanceError.
    }
    else {
        return 0.
    }
}

declare local function hoverThrottle {
    local fg is ship:mass * g0.
    local angleToNormal is vAng(ship:facing:forevector, ship:up:forevector).
    local throttleConfig is fg / cos(angleToNormal) / ship:availableThrust.
    return throttleConfig + (-ship:verticalspeed).
}

declare local function ignitionCount {
    local activeEngines to 0.
    for eng in engineList {
        if eng:ignition {
            set activeEngines to activeEngines + 1.
        }
    }
    return activeEngines.
}

declare local function valveShutdownProcedure {
    if ignitionCount <> 1 {
        for eng in engineList { 
            if eng:ignition and throtVal < 0.5 {
                local engineNumber is engineList:indexof(eng).
                
                engineList[engineNumber]:shutdown.
                set engineList[engineNumber]:gimbal:limit to 0.
            }
            if not eng:ignition and throtVal > 6.6 {
                local engineNumber is engineList:indexof(eng).
                
                engineList[engineNumber]:activate.
                set engineList[engineNumber]:gimbal:limit to 100.
            }
            wait 0.5.
        }
    }
    else {
        return.
    }
}

declare local function regimbal {
    for eng in engineList {
        set engine to engineList:indexof(eng).
        set engineList[engine]:gimbal:limit to 100.
    }
}

declare local function degimbal {
    for eng in engineList {
        set engine to engineList:indexof(eng).
        set engineList[engine]:gimbal:limit to 0.
    }
}

declare local function printfunc {
    print "STARSHIP GUIDANCE CONTROL COMPUTER(SGCC)" at (2,5).
    print "=======================" at (2,7).
    print "Height above terrain: " + round(trueRadar, 1) + "m      " at (2,9).
    print "Vertical velocity: " + round(abs(ship:verticalspeed),1) + "m/s      " at (2,11).
    print "Horizontal velocity: " + round(ship:groundspeed,1) + "m/s      " at (2,13).
    print "DeltaV remaining: " + round(stage:deltaV:current,1) + "m/s      " at (2,15).
    print "Error: " + round(errorDistance,1) + "m          " at (2,17).
    print "Master AoA: " + round(masterAoA,1) + "°          " at (2,19).
    print "Runmode: " + round(runmode) + "          " at (2,21).
    print "Active engines: " + ignitionCount() at (2,23).
    print "=======================" at (2,25).
}

//set currentRoll to ship:facing:roll.
// this prevents uncontrolled rolling
//lock steering to ship:srfretrograde*r(pitchAlongLatitude,pitchAlongLongitude,currentRoll-ship:srfretrograde:roll).


// actual code
set steeringmanager:pitchts to 2.
set steeringmanager:yawts to 2.
set steeringManager:rollts to 2.
//set progradeHeading to compass_for(ship, srfPrograde).

RCS on. SAS off. GEAR off.

until runmode = 0 {
    wait 0.01.
    
    if runmode = 1 {// ascent
        
        BRAKES off.
        lock steering to up.
        lock throttle to 2 * getTwr().

        if trueRadar >= 10000 {
            set runmode to 2.
            AG4 on.
            wait 1.
            lock throttle to 0. degimbal().
        }

    }

    if runmode = 2 {// ballistic guidance
        lock steering to heading(90, pitchReqPID:update(time:seconds, pitchPIDValues()), -rollReqPID:update(time:seconds, rollPIDValues())).// R(descentAOA_Y(), descentAOA_X(), 0) * srfRetrograde.
        set steeringmanager:rollcontrolanglerange to 180. //correct roll independent of pitch/yaw
        set steeringmanager:pitchts to 2.5.
        set steeringManager:rollts to 2.5.

        if trueRadar < 10000 { //rudimentary. something with atmospheric density would be more robust.
            RCS off.
        }
        if ship:verticalspeed < -50 and trueRadar < (decelHeight + 300) {   
            set runmode to 3.
            RCS on. set steeringmanager:rollcontrolanglerange to -1.
            lock throttle to hoverThrottle(). regimbal().
            set currentRoll to ship:facing:roll.
        }
    }

    if runmode = 3 {// motored guidance and touchdown
        
        lock steering to heading(getBearingFromAtoB(), 90-masterAoA, currentRoll).
        set steeringmanager:pitchts to 2.7 - sqrt(errorDistance)/5. //increase sensitivity as distance closes.
        set steeringmanager:yawts to 2.7 - sqrt(errorDistance)/5.

        if ship:velocity:surface:mag < 50{
            lock throttle to throtVal.
            set steeringManager:rollts to 20.
            valveShutdownProcedure().
        }

        if trueRadar < 15 {
            GEAR on.
            lock masterAoA to sqrt(errorDistance). //softer gains
        }

        if ship:groundspeed < 0.3 {
            set steeringmanager:pitchts to 2.7. //remove sensitivity adjustments. ensure equillibrium
            set steeringmanager:yawts to 2.7. 
        }

        if trueRadar <= 0.3 {
            set runmode to 0.
            AG4 off.
            lock throttle to 0.
            wait 2.
            unlock steering.
            SAS on. BRAKES off.
            wait 2.
            SAS off.
        }
    }
    printfunc().
}
