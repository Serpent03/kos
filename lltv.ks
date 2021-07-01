// see if I can set up a flag system.
// work on plane alignment

// clearscreen
set core:bootfilename to "lltv.ks".
clearScreen.
//run terminLIB.
run terminLIB.
SAS off. RCS on.
set terminal:charheight to 16.
set terminal:height to 21.
set terminal:width to 41.

// target
local padC to latlng(0.0120544, -156.530782).
set targethoverslam to (padC).

// initial variables
set program to 01.
set noun to 32.
set verb to 06.
set newVerb to false.

set descentFlag to false.
set rollFlag to false.
set monitorFlag to true.

set tgtApo to 0.
set tgtPer to 0.
set tgtIncl to 0.

lock trueRadar to alt:radar - 1.2.
lock g0 to constant:g * ship:body:mass/ship:body:radius^2.
lock shipAcc to (ship:maxThrust/ship:mass) - g0.
lock decelHeight to (ship:verticalspeed^2/(2 * shipAcc)) * 2.
lock throtVal to decelHeight/trueRadar * 5.
lock errorDistance to distanceMag.

// PID Loops

set throttlePid to pidLoop(0.2, 0.05, 0.01, 0, 1).
set throttlePid:setpoint to -(trueRadar/20).

local yawReqPID to pidLoop(0.2, 0.15, 0.02, -30, 30, 0.1).
set yawReqPID:setpoint to 0.

local pitchReqPID to pidLoop(0.2, 0.15, 0.02, -10, 40, 0.1).
set pitchReqPID:setpoint to 0.

// Calculation Functions

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

declare local function distanceMag { // error between predicted and target impact point
    if addons:tr:hasimpact {
        local mag is (addons:tr:impactpos:position - targethoverslam:position):mag.
        return mag.
    }
    else {
        return 0.
    }
}

declare local function pitchPID {	// TDAG EW - trajectory discrepancy avoidance guidance East <-> West.
    if addons:tr:hasimpact {
	    return (addons:tr:impactpos:lng - targetHoverslam:lng)*1000.
    }
    else {
        return 0.
    }
}


declare local function yawPID {	// TDAG NS -trajectory discrepancy avoidance guidance for North <-> South.
    if addons:tr:hasimpact {
	    return -(addons:tr:impactpos:lat - targetHoverslam:lat)*1000.
    }
    else {
        return 0.
    }
}

declare local function deltaLat {
    local dt is time:seconds.
    local lat1 is target:latitude.
    wait 0.1.
    local lat2 is target:latitude.

    return (lat2-lat1)/(time:seconds - dt).
}

declare local function launchAzimuth {
    return arcSin(cos(target:orbit:inclination)/cos(ship:latitude)).
}

local function timeToGoAscent {
    return round(abs(target:latitude - ship:latitude)/abs(deltaLat())).
}

declare local function steeringCommand {
    if not rollFlag {
        lock steering to heading(getBearingFromAtoB(), pitchReqPID:update(time:seconds, pitchPID()), 180).
    }
    if rollFlag {
        lock steering to -r(yawReqPID:update(time:seconds, yawPID()), pitchReqPID:update(time:seconds, pitchPID()), 0) * srfRetrograde.
    }
}


// Data Manipulation Functions
 
declare local function agcData { // this thing was a fucking PAIN to write, LOL
    agcDataDisplay().

                                            print "_______________" at (2,3).                                                                                               print "___________________" at (21,3).                                          
    print "|" at (.5, 5).  print "UPLINK" at (2,5).   print " " at (8,5).   print "TEMP" at (10,5).         print "|" at (17.5, 5).   print "|" at (20.5, 5).   print "COMP" at (22,5). print " " at (30,5). print "PROG" at (32,5).                  print "|" at (39.5, 5).                 
    print "|" at (.5, 6).  print "ACTY" at (2,6).     print " " at (8,6).                                   print "|" at (17.5, 6).   print "|" at (20.5, 6).   print "ACTY" at (22,6). print " " at (30,6). print program + " " at (32, 6).          print "|" at (39.5, 6).             
    print "|" at (.5, 7).  print "NO ATT" at (2,8).   print " " at (8,8).   print "GIMBAL" at (10,8).       print "|" at (17.5, 7).   print "|" at (20.5, 7).   print "VERB" at (22,8). print " " at (30,8). print "NOUN" at (32,8).                  print "|" at (39.5, 7).             
    print "|" at (.5, 8).  print "   " at (2,9).      print " " at (8,9).   print "LOCK" at (10,9).         print "|" at (17.5, 8).   print "|" at (20.5, 8).   print verb + " " at (22,9). print " " at (30,9). print noun + " " at (32,9).          print "|" at (39.5, 8).             
    print "|" at (.5, 9).  print "       " at (2,13). print " " at (8,11).  print "    " at (10,11).        print "|" at (17.5, 9).   print "|" at (20.5, 9).   print "------------------" at (22,10).                                                print "|" at (39.5, 9).             
    print "|" at (.5, 10).                            print "" at (8,13).   print "RESTART" at (10,13).     print "|" at (17.5, 10).  print "|" at (20.5, 10).                                                                                        print "|" at (39.5, 10).            
    print "|" at (.5, 11). print "OPR ERR" at (2,15). print "" at (8,15).   print "TRACKER" at (10,15).     print "|" at (17.5, 11).  print "|" at (20.5, 11).  print "------------------" at (22,12).                                                print "|" at (39.5, 11).            
    print "|" at (.5, 12).                  print "_______________" at (2,16).                              print "|" at (17.5, 12).  print "|" at (20.5, 12).                                                                                        print "|" at (39.5, 12).
    print "|" at (.5, 13).                                                                                  print "|" at (17.5, 13).  print "|" at (20.5, 13).  print "------------------" at (22,14).                                                print "|" at (39.5, 13).
    print "|" at (.5, 14).                                                                                  print "|" at (17.5, 14).  print "|" at (20.5, 14).                                                                                        print "|" at (39.5, 14).
    print "|" at (.5, 15).                                                                                  print "|" at (17.5, 15).  print "|" at (20.5, 15).                  print "___________________" at (21,16).                               print "|" at (39.5, 15).

}

declare local function agcDataDisplay { // where we check what noun is active and display data based on that
    // Program Section
    // I think the actual thing used a VN pair to show this information for various PGMs. Having the actual PGM as constraint is restrictive..

    if monitorFlag { // have to make monitorFlag for update/monitor corresponding to VERB 0X and 1X
        if program <> 1 {
            print "    " at (2,11).
        }
        if program = 12 and noun = 93{ //most likely incorrect noun code
            print tgtApo + "     "  at (32,11).
            print tgtPer + "     "  at (32,13).
            print tgtIncl + "     "  at (32,15).
        }
        if program = 12 and noun = 94{ //most likely incorrect noun code
            print tgtApo + "     "  at (32,11).
            print tgtPer + "     "  at (32,13).
            print timeToGoAscent + "     "  at (32,15).
        }
        // Noun Section
        if noun = 32 {
            print "        "  at (32,11).
            print "" + round(min(999, stage:deltaV:duration)) + "     "  at (32,13).
            print "" + round(eta:periapsis) + "     " at (32,15).
        }
        if noun = 42 {
            print "" + round(ship:apoapsis/1000,1) + "    " at (32,11).
            print "" + round(ship:periapsis/1000,1) + "    " at (32,13).
            print "+" + round(stage:deltaV:current) + "    " at (32,15).
        }
        if noun = 43 {
            print "" + round(ship:geoposition:lat,1) + "    " at (32,11).
            print "" + round(ship:geoposition:lng,1) + "    " at (32,13).
            print "+" + round(ship:altitude) + "    " at (32,15).
        }
        if noun = 54 {
            print "" + round(errorDistance) + "     " at (32,11).
            print "" + round(ship:groundspeed) + "     " at (32,13).
            print "" + round(getBearingFromAtoB()) + "     " at (32,15).
        }
        if noun = 89 {
            print "" + round(targethoverslam:lat,1) + "    " at (32,11).
            print "" + round(targethoverslam:lng,1) + "    " at (32,13).
            print "" + round(targethoverslam:terrainheight) + "     " at (32,15).
        }    
        if noun = 92 {
            print "" + round(min(100, max(0, throtVal*100))) + "     " at (32,11).
            print "" + round(ship:verticalspeed) + "     " at (32,13).
            print "+" + round(trueRadar) + "    " at (32,15).
        }
    }
    if verb = 27 { // try using a random num +/- the free local disk space.
        print "" + (core:volume:freespace + max(-999, min(999, floor(random()/10)))) + " " at (32,11).
    }
}

when terminal:input:haschar then { // checks input from the terminal
    if terminal:input:getchar() = "+" {
        keyRelLogic(true).
        set inputArg to terminal_input_string(32,9).
        set noun to inputArg.
    }
    if terminal:input:getchar() = "-" {
        keyRelLogic(true).
        set inputArg to terminal_input_string(22,9).
        set verb to inputArg.
        set newVerb to true.
    }
    if majorVerbChecker() and newVerb{
        keyRelLogic(true).
        set inputArg to terminal_input_string(32,6).
        set program to inputArg.
        currentProgramParameterCheck().
        set newVerb to false.
    }
    keyRelLogic(false).
    preserve.
} 

declare local function majorVerbChecker { // Verb 37 is used to change program modes. So you enter verb 37, and then your program. For that reason, we gotta make a check
//to confirm that if we ever want to change the program, it only changes on verb 37. we also wanna make sure that it will only change program once per each verb 37 change
    if verb = 37 {
        return true.
    }
    if verb = 69 { // funi number :P but this one is real. Verb 69 was used to reboot computers
        reboot.
    }
    if verb = 01 {
        set monitorFlag to false.
    }
    if verb = 11 {
        set monitorFlag to true.
    }
}

//due to kOS errors, it takes several enters or - or + to get the desired verb/noun input. 
//For example, if you wanted to enter a VERB, you would press - twice, and then press enter once.
//Similarly for a NOUN, you would press + once, and then press enter twice.
//For a PROGRAM, you would first do the sequence for VERB 37, then type in the program, and then press enter once. 

declare local function keyRelLogic { // make a rudimentary logic of KEY REL light. As I understand it more, I will start to add capability.
    parameter io.
    if io {
        print "KEY REL" at (2,13).
        wait 0.1.
        print "       " at (2,13).
        wait 0.1.
        print "KEY REL" at (2,13).
    }
    if not io {
        print "       " at (2,13).
    } 
}

declare local function progLightLogic { // rudimentary PROG light logic.
    parameter io.
    if io {
        print "PROG" at (10,11).
        wait 0.1.
        print "    " at (10,11).
        wait 0.1.
        print "PROG" at (10,11).
    }
    if not io {
        print "    " at (10,11).
    }
}


declare local function currentProgramParameterCheck { // program parameter input logic.
    if program = 12 {
        progLightLogic(true).
        set tgtApo to (terminal_input_string(32, 15)):toscalar().
        set tgtPer to (terminal_input_string(32, 15)):toscalar().
        if hasTarget {
            set tgtIncl to round(target:orbit:inclination,1).
        }
        if not hasTarget {
            set tgtIncl to (terminal_input_string(32, 15)):toscalar().
        }
    }
    progLightLogic(false).
}


set steeringManager:rollts to 3.
set steeringManager:pitchts to 3.
set steeringManager:yawts to 3.

// Actual logic 
until program = 00 {
    agcData().  // print information
    majorVerbChecker().

    if program = 1 {
        unlock steering.
        unlock throttle.
        print "STBY" at (2,11).
    }

    if program = 12 { // I was coincidentally lucky in naming this. P12 was a real program used to ascend from the Lunar surface
     if timeToGoAscent() < 1 {
            lock throttle to 2 * getTwr().
            lock steering to heading(launchAzimuth(), 90-min(90, trueRadar/1000), 0).
            if tgtApo * 1000 <= ship:apoapsis {
                set program to 65.
            }
        }
    }

    if program = 63 { // velocity reduction
        if not descentFlag {
            lock throttle to 0.05.
        }
        if ship:verticalspeed < 0 {
            set descentFlag to true.
            //steeringCommand().
            lock steering to -r(yawReqPID:update(time:seconds, yawPID()), pitchReqPID:update(time:seconds, pitchPID()), 180) * srfRetrograde. // set our steering so that we get to target.
            lock throttle to max(0.1, max(throtVal, sqrt(errorDistance)/500)).
            if ship:groundspeed < 1000 {
                set rollFlag to true.
            }
            if errorDistance < 1000 and ship:groundspeed < 400 {
                set program to 64.
            }
        }
    }

    if program = 64 { // trajectory control
        lock throttle to max(0.1, throtVal).
        // add a LPD change function

        if SAS {
            set program to 66. unlock steering.
        }
    }

    if program = 66 {
        
        lock throttle to max(0.1, throttlePid:update(time:seconds, ship:verticalspeed)). // PGM 66, or rate of descent, lets us descent at a very slow rate.
        if trueRadar < 0.5 or ship:status = "landed" {
            set program to 68. // program 68 is confirmation of touchdown.
        }
    }
    wait 0.15.
}