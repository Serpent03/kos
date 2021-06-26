// clearscreen
clearScreen.
run terminLIB.
SAS off. RCS on.
set terminal:height to 22.
set terminal:width to 42.
set terminal:charheight to 16.

// target
local padC to latlng(-0.185418964943541, -74.4728985609505).
set targethoverslam to (padC).

// initial variables
set program to 1.
set noun to 43.
set verb to 11.
set newVerb to false.

lock trueRadar to alt:radar - 1.3.
lock g0 to constant:g * ship:body:mass/ship:body:radius^2.
lock shipAcc to (ship:maxThrust/ship:mass) - g0.
lock decelHeight to (ship:verticalspeed^2/(2 * shipAcc)) * 2.
lock throtVal to decelHeight/trueRadar * 5.
lock errorDistance to distanceMag.

// PID Loops

set throttlePid to pidLoop(0.2, 0.05, 0.01, 0, 1).
set throttlePid:setpoint to -2.5.

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

declare local function descentAOA_X {	// TDAG EW - trajectory discrepancy avoidance guidance East <-> West.
	return min((30), max(-(30), (addons:tr:impactpos:lng - targetHoverslam:lng)*5000)).
}


declare local function descentAOA_Y {	// TDAG NS -trajectory discrepancy avoidance guidance for North <-> South.
	return min(30, max(-30, (addons:tr:impactpos:lat - targetHoverslam:lat)*5000)).
}


// Data Manipulation Functions
 
declare local function agcData { // this thing was a fucking PAIN to write, LOL
    agcManipulation().

                                            print "_______________" at (2,3).                                                                                               print "___________________" at (21,3).                                          
    print "|" at (.5, 5).  print "UPLINK" at (2,5).   print " " at (8,5).   print "TEMP" at (10,5).         print "|" at (17.5, 5).   print "|" at (20.5, 5).   print "COMP" at (22,5). print " " at (30,5). print "PROG" at (32,5).                  print "|" at (39.5, 5).                 
    print "|" at (.5, 6).  print "ACTY" at (2,6).     print " " at (8,6).                                   print "|" at (17.5, 6).   print "|" at (20.5, 6).   print "ACTY" at (22,6). print " " at (30,6). print program + " " at (32, 6).          print "|" at (39.5, 6).             
    print "|" at (.5, 7).  print "NO ATT" at (2,8).   print " " at (8,8).   print "GIMBAL" at (10,8).       print "|" at (17.5, 7).   print "|" at (20.5, 7).   print "VERB" at (22,8). print " " at (30,8). print "NOUN" at (32,8).                  print "|" at (39.5, 7).             
    print "|" at (.5, 8).  print "   " at (2,9).      print " " at (8,9).   print "LOCK" at (10,9).         print "|" at (17.5, 8).   print "|" at (20.5, 8).   print verb + " " at (22,9). print " " at (30,9). print noun + " " at (32,9).          print "|" at (39.5, 8).             
    print "|" at (.5, 9).  print "       " at (2,13). print " " at (8,11).  print "PROG" at (10,11).        print "|" at (17.5, 9).   print "|" at (20.5, 9).   print "------------------" at (22,10).                                                print "|" at (39.5, 9).             
    print "|" at (.5, 10).                            print "" at (8,13).   print "RESTART" at (10,13).     print "|" at (17.5, 10).  print "|" at (20.5, 10).                                                                                        print "|" at (39.5, 10).            
    print "|" at (.5, 11). print "OPR ERR" at (2,15). print "" at (8,15).   print "TRACKER" at (10,15).     print "|" at (17.5, 11).  print "|" at (20.5, 11).  print "------------------" at (22,12).                                                print "|" at (39.5, 11).            
    print "|" at (.5, 12).                  print "_______________" at (2,16).                              print "|" at (17.5, 12).  print "|" at (20.5, 12).                                                                                        print "|" at (39.5, 12).
    print "|" at (.5, 13).                                                                                  print "|" at (17.5, 13).  print "|" at (20.5, 13).  print "------------------" at (22,14).                                                print "|" at (39.5, 13).
    print "|" at (.5, 14).                                                                                  print "|" at (17.5, 14).  print "|" at (20.5, 14).                                                                                        print "|" at (39.5, 14).
    print "|" at (.5, 15).                                                                                  print "|" at (17.5, 15).  print "|" at (20.5, 15).                  print "___________________" at (21,16).                               print "|" at (39.5, 15).

}

declare local function agcManipulation { // where we check what noun is active and display data based on that
    if program <> 1 {
        print "    " at (2,11).
    }
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
        print "+" + round(alt:radar) + "    " at (32,15).
    }
    if noun = 54 {
        print "" + round(errorDistance) + "     " at (32,11).
        print "" + round(ship:groundspeed) + "     " at (32,13).
        print "" + round(getBearingFromAtoB()) + "     " at (32,15).
    }
    if noun = 92 {
        print "" + round(min(100, max(0, throtVal*100))) + "     " at (32,11).
        print "" + round(ship:verticalspeed) + "     " at (32,13).
        print "+" + round(trueRadar) + "    " at (32,15).
    }
    if verb = 27 {
        print "" + core:volume:capacity at (32,11).
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
    if verbChecker() and newVerb{
        keyRelLogic(true).
        set inputArg to terminal_input_string(32,6).
        set program to inputArg.
        set newVerb to false.
    }
    keyRelLogic(false).
    preserve.
} 
//due to kOS errors, it takes several enters or - or + to get the desired verb/noun input. 
//For example, if you wanted to enter a VERB, you would press - twice, and then press enter once.
//Similarly for a NOUN, you would press + once, and then press enter twice.
//For a PROGRAM, you would first do the sequence for VERB 37, then type in the program, and then press enter once. 

declare local function keyRelLogic { // make a rudimentary logic of KEY REL light. As I understand it more, I will start to add capability.
    parameter io.
    if io {
        print "KEY REL" at (2,13).
    }
    if not io {
        print "       " at (2,13).
    } 
}

declare local function verbChecker { // Verb 37 is used to change program modes. So you enter verb 37, and then your program. For that reason, we gotta make a check
//to confirm that if we ever want to change the program, it only changes on verb 37. we also wanna make sure that it will only change program once per each verb 37 change
    if verb = 37 {
        return true.
    }
    if verb = 69 { // funi number :P but this one is real. Verb 69 was used to reboot computers
        reboot.
    }
}

until program = 00 {
    agcData().  // print information

    if program = 1 {
        unlock steering.
        unlock throttle.
        print "STBY" at (2,11).
    }

    if program = 12 { // this one is not a real program, just made to ascent
        lock throttle to 2 * getTwr().
        lock steering to heading(0, 90, 0).
    }

    if program = 65 {
        if ship:verticalspeed > 10 {
            lock throttle to 0.
        }
        else {
            lock steering to up * r(descentAOA_Y(), descentAOA_X(), 0). // set our steering so that we get to target.
            lock throttle to throtVal.

            if SAS {
                set program to 66. unlock steering. // by enabling SAS, you elect to take manual control and tell the computer to maintain a certain vertical speed.
            }
        }
    }

    if program = 66 {
        
        lock throttle to throttlePid:update(time:seconds, ship:verticalspeed). // PGM 66, or rate of descent, lets us descent at a very slow rate.
        if trueRadar < 0.5 or ship:status = "landed" {
            set program to 68. // program 68 is confirmation of touchdown.
        }
    }
    wait 0.15.
}