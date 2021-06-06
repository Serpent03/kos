clearScreen.

// initial variables

LIST ENGINES IN engineList.

lock trueRadar to actualHeight().
lock g0 to constant:g * ship:body:mass/ship:body:radius^2.
lock shipAcc to (ship:availableThrust/ship:mass) - g0.
lock decelHeight to (ship:verticalspeed^2/(2 * shipAcc)) * 2.
lock throtVal to decelHeight/trueRadar.

// calculation functions
local function actualHeight {	// get actual height from KSP collision box system
	local bounds_box is ship:bounds.
	return bounds_box:bottomaltradar.
}

declare local function getTwr {	// Throttle needed to maintain a specific TWR
	local throttleSetting is g0 * (ship:mass/ship:availableThrust).
    return throttleSetting.
}

declare local function printfunc {
    print "AUTOMATIC ROVER TOUCHDOWN SYSTEM(ARTS)" at (2,5).
    print "=======================" at (2,7).
    print "Height above terrain: " + round(trueRadar, 1) + "m      " at (2,9).
    print "Vertical velocity: " + round(abs(ship:verticalspeed),1) + "m/s      " at (2,11).
    print "Horizontal velocity: " + round(ship:groundspeed,1) + "m/s      " at (2,13).
    print "DeltaV remaining: " + round(stage:deltaV:current,1) + "m/s      " at (2,15).
    print "=======================" at (2,19).
}

declare local function hoverThrottle {
    local fg is ship:mass * g0.
    local angleToNormal is vAng(ship:facing:forevector, ship:up:forevector).
    local throttleConfig is fg / cos(angleToNormal) / ship:availableThrust.
    return throttleConfig + (-ship:verticalspeed).
}

declare local function checkEngineHealth {
    for eng in engineList {
        if eng:ignition = false {
            set engineNumber to engineList:indexof(eng).
    
            if engineNumber > 4 and engineList[engineNumber-4]:ignition = true{
                engineList[engineNumber-4]:shutdown.
                print "Due to engine " + engineNumber + " failing, engine " + (engineNumber - 4) + " has been decyled for symmetry." at (2,17).
            }

            else if engineNumber < 4 and engineList[engineNumber+4]:ignition = true{
                engineList[engineNumber+4]:shutdown.
                print "Due to engine " + engineNumber + " failing, engine " + (engineNumber + 4) + " has been decyled for symmetry." at (2,17).
            }
            
            else if engineNumber = 4 and engineList[engineNumber-4]:ignition = true{
                engineList[engineNumber-4]:shutdown.
                print "Due to engine " + engineNumber + " failing, engine " + (engineNumber + 4) + " has been decyled for symmetry." at (2,17).
            }
        }
    }
}

// Pre-loop configuration

BRAKES on.
set runmode to 1.
wait until ship:verticalspeed < -5.
lock steering to srfRetrograde.
SAS off.

// Main-loop

until runmode = 0 {

    wait 0.15.
    if trueRadar < decelHeight and runmode = 1{
        lock throttle to throtVal.
        set runmode to 2.
    }
    if trueRadar < 0.2 and runmode = 2{
        lock steering to up.
        lock throttle to hoverThrottle().
        wait 2.
        stage.
        wait 1.
        lock throttle to 4 * getTwr().
        wait 1.
        lock steering to heading(0, 45, 0).
        set runmode to 3.
    }
    if stage:DeltaV:current < 15 and runmode = 3 {
        set runmode to 0.
    }
    
    printfunc().
    checkEngineHealth().

}