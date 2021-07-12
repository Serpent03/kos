// see if I can set up a flag system.
// research open/closed loop orbital-insertion guidance -> check that orbiter wiki page for PEG
// implement state vectors and remove API extraction dependency 
// work on getting the telescope mod for IMU alignments


// clearscreen
set core:bootfilename to "lltv.ks".
clearScreen.

//run terminLIB.
runOncePath("terminLIB").
runOncePath("navballLIB").
SAS off. RCS on.
set terminal:charheight to 22.
set terminal:height to 21.
set terminal:width to 41.

// target
local tgtLand to latlng(0.0120544, 102.265).
set targethoverslam to (tgtLand).

// ---- Initial variables ----
set program to 01.
set noun to 32.
set verb to 06.
set newVerb to false.
set oldVerb to 0.

set lastEventTime to time:seconds.

set monitorOnceFlag to false.
set descentFlag to false.
set enterDataFlag to true.

set R1BIT to true.
set R2BIT to true.
set R3BIT to true.
set RESTARTBIT to false. // write to json and read pgm/flags/BITs as necessary

set VAC_BANK to 0. // implement vec. accu. centers for job and waitlist logic

set tgtApo to 0.
set tgtPer to 0.
set tgtIncl to 0.

lock trueRadar to alt:radar - 1.2.
lock g0 to constant:g * ship:body:mass/ship:body:radius^2.
lock shipAcc to (ship:maxThrust/ship:mass) - g0.
lock decelHeight to (ship:verticalspeed^2/(2 * shipAcc)) * 2.
lock throtVal to decelHeight/trueRadar * 5.
lock errorDistance to distanceMag.

//  ---- PID Loops ----

set throttlePid to pidLoop(0.2, 0.05, 0.01, 0, 1).

local yawReqPID to pidLoop(0.5, 0.15, 0.05, -30, 30).
set yawReqPID:setpoint to 0.

local pitchReqPID to pidLoop(0.5, 0.15, 0.05, -60, 10).
set pitchReqPID:setpoint to 0.

// ---- Calculation Functions ----

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

declare local function PITCH_LAND_GUIDE {	// TDAG EW - trajectory discrepancy avoidance guidance East <-> West.
    if addons:tr:hasimpact {
	    return -(addons:tr:impactpos:lng - targetHoverslam:lng)*1000.
    }
    else {
        return 0.
    }
}


declare local function YAW_LAND_GUIDE {	// TDAG NS -trajectory discrepancy avoidance guidance for North <-> South.
    if addons:tr:hasimpact {
	    return (addons:tr:impactpos:lat - targetHoverslam:lat)*1000.
    }
    else {
        return 0.
    }
}

declare local function YAW_ASCENT_GUIDE { // Inclination correction guidance. Gotta calculate the drift from normal vector resultant manually.
    // this should return the angle from normal vectors of each.
    return -arcCos(sin(ship:orbit:inclination) * sin(target:orbit:inclination) * cos(ship:orbit:longitudeofascendingnode - target:orbit:longitudeofascendingnode) + cos(ship:orbit:inclination) * cos(target:orbit:inclination)).
    //return (target:orbit:inclination - ship:orbit:inclination)*1000.
}


declare local function launchAzimuth {

    local azimuth to arcSin(cos(target:orbit:inclination) / cos(ship:geoposition:lat)).
    local tgtOrbitVel to sqrt(ship:body:mu/(((tgtApo + tgtPer)*1000 + 2*ship:body:radius)/2)).
    local surfRotVel to cos(ship:geoposition:lat) * (2*constant:pi * ship:body:radius/ship:body:rotationperiod).
    local vRotX to tgtOrbitVel * sin(azimuth) - surfRotVel * cos(ship:geoposition:lat).
    local vRotY to tgtOrbitVel * cos(azimuth).
    
    local finalAzimuth to arcTan(vRotX/vRotY).

    return finalAzimuth.
    
}

local function relativeInclination {
    return round(abs(ship:orbit:longitudeofascendingnode - target:orbit:longitudeofascendingnode),3).
}

//  ---- Data Manipulation Functions ----

declare local function agcStatic {
                                                print "_______________" at (2,3).                                                                                               print "___________________" at (21,3).                                          
    print "|" at (.5, 5).                                                                                   print "|" at (17.5, 5).   print "|" at (20.5, 5).                           print " " at (30,5). print "PROG" at (32,5).                  print "|" at (39.5, 5).                 
    print "|" at (.5, 6).                                                                                   print "|" at (17.5, 6).   print "|" at (20.5, 6).                           print " " at (30,6).                                          print "|" at (39.5, 6).             
    print "|" at (.5, 7).                                                                                   print "|" at (17.5, 7).   print "|" at (20.5, 7).   print "VERB" at (22,8). print " " at (30,8). print "NOUN" at (32,8).                  print "|" at (39.5, 7).             
    print "|" at (.5, 8).                                                                                   print "|" at (17.5, 8).   print "|" at (20.5, 8).                                                                                         print "|" at (39.5, 8).             
    print "|" at (.5, 9).                                                                                   print "|" at (17.5, 9).   print "|" at (20.5, 9).   print "------------------" at (22,10).                                                print "|" at (39.5, 9).             
    print "|" at (.5, 10).                                                                                  print "|" at (17.5, 10).  print "|" at (20.5, 10).                                                                                        print "|" at (39.5, 10).            
    print "|" at (.5, 11).                                                                                  print "|" at (17.5, 11).  print "|" at (20.5, 11).  print "------------------" at (22,12).                                                print "|" at (39.5, 11).            
    print "|" at (.5, 12).                      print "_______________" at (2,16).                          print "|" at (17.5, 12).  print "|" at (20.5, 12).                                                                                        print "|" at (39.5, 12).
    print "|" at (.5, 13).                                                                                  print "|" at (17.5, 13).  print "|" at (20.5, 13).  print "------------------" at (22,14).                                                print "|" at (39.5, 13).
    print "|" at (.5, 14).                                                                                  print "|" at (17.5, 14).  print "|" at (20.5, 14).                                                                                        print "|" at (39.5, 14).
    print "|" at (.5, 15).                                                                                  print "|" at (17.5, 15).  print "|" at (20.5, 15).              print "___________________" at (21,16).                                   print "|" at (39.5, 15).

}


declare local function agcData { // this thing was a fucking PAIN to write, LOL
    agcStatic().
    agcDataDisplay().
                                                                                                                                                                                   
    print "UPLINK" at (2,5).   print " " at (8,5).   print "TEMP" at (10,5).           print "COMP" at (22,5).              
    print "ACTY" at (2,6).     print " " at (8,6).                                     print "ACTY" at (22,6). print program + " " at (32, 6).          
    print "NO ATT" at (2,8).   print " " at (8,8).                              
    print "   " at (2,9).      print " " at (8,9).                                     print verb + " " at (22,9). print " " at (30,9). print noun + " " at (32,9).          
    print "       " at (2,13). print " " at (8,11).  print "    " at (10,11).                                                          
                               print "" at (8,13).                                                          
    print "OPR ERR" at (2,15). print "" at (8,15).   print "TRACKER" at (10,15).                                                       
                                                                                                          
                                                                 
}

declare local function agcDataDisplay { // where we check what noun is active and display data based on that
    // Station keeping

    IMU_GimbalCheck().
    RESTARTCHECK().
    restartLightLogic(RESTARTBIT).

    // Program Section
    // I think the actual thing used a VN pair to show this information for various PGMs. Having the actual PGM as constraint is restrictive..

    if program <> 1 {
        print "    " at (2,11).
    }
    if program = 12 and noun = 93{ //most likely incorrect noun code
        registerDisplays(tgtApo, tgtPer, tgtIncl).
    }
    if program = 12 and noun = 94{ //most likely incorrect noun code
        registerDisplays(tgtApo, tgtPer, round(relativeInclination(),2)).
    }
    // Noun Section
    // Add an event timer for PGM 63 to 64 on pitchover.
    if noun = 32 {
        registerDisplays("", round(min(999, stage:deltaV:duration)), round(eta:periapsis)).
        // make this create paste and delete the same file so first iter gets newest data
    }
    if noun = 34 {
        // call MET + time for P64 pitchover and other
        registerDisplays("", "", TIME_TO_EVENT(program)).
    }
    if noun = 35 {
        registerDisplays("", "", TIME_FROM_EVENT()).
    }
    if noun = 42 {
        registerDisplays(round(ship:apoapsis/1000,1), round(ship:periapsis/1000,1), round(stage:deltaV:current)).
    }
    if noun = 43 {
        registerDisplays(round(ship:geoposition:lat,1), round(ship:geoposition:lng,1), round(ship:altitude)).
    }
    if noun = 54 {
        registerDisplays(round(errorDistance), round(ship:groundspeed), round(getBearingFromAtoB())).
    }
    if noun = 61 {
        registerDisplays(round(targethoverslam:lat,1), round(targethoverslam:lng,1), "").
    }
    if noun = 67 {
        registerDisplays(abs(round(ship:geoposition:position:mag - targethoverslam:position:mag)), round(ship:geoposition:lat,1), round(ship:geoposition:lng,1)).
    }
    if noun = 73 {
        registerDisplays(round(ship:altitude), round(ship:velocity:mag), round(pitch_for(ship, prograde))).
    }
    if noun = 92 {
        registerDisplays(round(min(100, max(0, throtVal*100)))+"", round(ship:verticalspeed), round(trueRadar)).
    }

    // Verb Section.

    if verb = 27 { // wishlist; make this VAC/RAM space. will have to define RAM
        registerDisplays((core:volume:freespace), "", "").
    }
}

declare local function registerDisplays {
    parameter R1, R2, R3.
   
    set regData to lexicon().
    regData:add("R1", R1).
    regData:add("R2", R2).
    regData:add("R3", R3).

    writeJson(regData, "0:/regdata.json").

    // need to fix monitor flag.

    if monitorOnceFlag {
        print R1:tostring():padleft(5) at (33,11).
        print R2:tostring():padleft(5) at (33,13).
        print R3:tostring():padleft(5) at (33,15).
        set monitorOnceFlag to false.
    }

    //tostring():padleft(5) looks like a really good alternative.

    if R1BIT {
        print R1:tostring():padleft(7) at (33,11).
    }
    if R2BIT {
        print R2:tostring():padleft(7) at (33,13).
    }
    if R3BIT {
        print R3:tostring():padleft(7) at (33,15).
    }
}

when terminal:input:haschar then { // checks input from the terminal
    if terminal:input:getchar() = "+" {
        keyRelLogic(true).
        set inputArg to terminal_input_string(32,9).
        set noun to inputArg.
    }
    if terminal:input:getchar() = "-" {
        set oldVerb to verb.
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
        set enterDataFlag to true.
    }
    keyRelLogic(false).
    preserve.
} 

declare local function majorVerbChecker { // Verb 37 is used to change program modes. So you enter verb 37, and then your program. For that reason, we gotta make a check
//to confirm that if we ever want to change the program, it only changes on verb 37. we also wanna make sure that it will only change program once per each verb 37 change
    if verb = 37 {
        return true.
    }
    if verb = 33 {
        set enterDataFlag to false.
    }
    if verb = 69 { // funi number :P but this one is real. Verb 69 was used to reboot computers
        set RESTARTBIT to true.
        // links directly to restart check.
    }
    if verb = 01 {
        set monitorOnceFlag to true.
        set R1BIT to false.
    }
    if verb = 02 {
        set monitorOnceFlag to true.
        set R2BIT to false.
    }
    if verb = 03 {
        set monitorOnceFlag to true.
        set R3BIT to false.
    }
    if verb = 04 {
        set monitorOnceFlag to true.
        set R1BIT to false.
        set R2BIT to false.
    }
    if verb = 05 {
        set monitorOnceFlag to true.
        set R1BIT to false.
        set R2BIT to false.
        set R3BIT to false.
    }
    if verb = 11 {
        set R1BIT to true.
    }
    if verb = 12 {
        set R2BIT to true.
    }
    if verb = 13 {
        set R3BIT to true.
    }
    if verb = 14 {
        set R1BIT to true.
        set R2BIT to true.
    }
    if verb = 15 {
        set R1BIT to true.
        set R2BIT to true.
        set R3BIT to true.
    }
}

//due to kOS errors, it takes several enters or - or + to get the desired verb/noun input. 
//For example, if you wanted to enter a VERB, you would press - twice, and then press enter once.
//Similarly for a NOUN, you would press + once, and then press enter twice.
//For a PROGRAM, you would first do the sequence for VERB 37, then type in the program, and then press enter once. 

// ---- Functionability ----

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

declare local function restartLightLogic {
    parameter io.
    if io {
        print "RESTART" at (10,13).
    }
    if  not io {
        print "       " at (10,13).
    }
}

declare local function currentProgramParameterCheck { // program parameter input logic.
    if program = 12 and enterDataFlag {
        progLightLogic(true).
        set tgtApo to (terminal_input_string(33, 15)):toscalar().
        set tgtPer to (terminal_input_string(33, 15)):toscalar().
        if hasTarget {
            set tgtIncl to round(target:orbit:inclination,1).
        }
        else {
            set tgtIncl to (terminal_input_string(33, 15)):toscalar().
        }
    }
    progLightLogic(false).
}

// ---- Event checks and ship health ----

declare local function TIME_TO_EVENT {
    parameter prog.
    local curTime to time:seconds.
    
    if prog = 63 { // time to p64 pitch over.
        local TTPO to time:seconds + abs(100 - ship:groundspeed/shipAcc).
        if TTPO <= 1 {
            set lastEventTime to time:seconds + abs(100 - ship:groundspeed/shipAcc).
            return round(lastEventTime).
        }
        return round(abs(TTPO - curTime)).
    }

    if prog = 01 {
        local TT to time:seconds.
        return round(TT).
    }
}

declare local function TIME_FROM_EVENT {
    return round(abs(lastEventTime - time:seconds)).
}

declare local function IMU_GimbalCheck {
    // integrate relative Euler's angles into this by reading ship:facing.
    if abs(pitch_for(ship)) > 80 {
        if abs(roll_for(ship)) < 100 and abs(roll_for(ship)) > 80 {
            print "GIMBAL" at (10,8).
            print "LOCK" at (10,9).
        }
    }
    else {
        print "      " at (10,8).
        print "    " at (10,9).  
    }   
}

declare local function RESTARTCHECK {
    if RESTARTBIT {
        set dataOut to lexicon().
        dataOut:add("Noun", noun).
        dataOut:add("Verb", oldVerb).
        dataOut:add("Program", program).
        dataOut:add("RESTART", RESTARTBIT).
        writeJson(dataOut, "compState.json").
        set ship:control:pilotmainthrottle to throttle.
        //log output. V,N,P,BIT,Flag (json).
        reboot.
    }
}

declare local function READ_LAST_KEYS {
    // this thing will set the last outputs from the json file.
    if exists("compState.json") {
        set dataIn to readJson("compState.json").
        set noun to dataIn["Noun"].
        set verb to dataIn["Verb"].
        set program to dataIn["Program"].
        set RESTARTBIT to dataIn["RESTART"].
        restartLightLogic(RESTARTBIT).
        wait 1.
        set RESTARTBIT to false.    
    }
}

set steeringManager:rollts to 3.
set steeringManager:pitchts to 3.
set steeringManager:yawts to 3.
agcStatic().
READ_LAST_KEYS().

// Actual logic 
until program = 00 {
    agcData().  

    if program = 1 {
        unlock steering.
        unlock throttle.
        print "STBY" at (2,11).
    }

    if program = 12 { // I was coincidentally lucky in naming this. P12 was a real program used to ascend from the Lunar surface
     if relativeInclination() < 0.5 {
            lock throttle to 2 * getTwr().
            lock steering to heading(launchAzimuth()+yawReqPID:update(time:seconds, YAW_ASCENT_GUIDE()), 90-min(90, trueRadar/1000), 0).
            if tgtApo * 1000 <= ship:apoapsis {
                set program to 65.
            }
        }
    }

    if program = 63 { // velocity reduction
        if not descentFlag {
            lock throttle to 0.5.
        }
        if ship:verticalspeed < 0 {
            set descentFlag to true.
            //steeringCommand().
            lock steering to srfRetrograde * -r(pitchReqPID:update(time:seconds, PITCH_LAND_GUIDE()), yawReqPID:update(time:seconds, YAW_LAND_GUIDE()), 180). // set our steering so that we get to target.
            lock throttle to max(0.1, max(throtVal, sqrt(errorDistance)/500)).
            if errorDistance < 1000 and ship:groundspeed < 100 {
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
        set throttlePid:setpoint to -(trueRadar/20).
        lock throttle to max(0.1, throttlePid:update(time:seconds, ship:verticalspeed)). // PGM 66, or rate of descent, lets us descent at a very slow rate.
        if trueRadar < 1 or ship:status = "landed" {
            set program to 68. // program 68 is confirmation of touchdown.
        }
    }

    if program = 68{
        //set surfaceFlag to true. // I think this was used for abort confirmation of not using the DM engine
        lock throttle to 0.
        wait 2.
        unlock steering.
        SAS off.
        RCS off.
    }
    wait 0.15.
}

deletePath("compState.json").