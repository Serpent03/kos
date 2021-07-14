// see if I can set up a flag system.
// research open/closed loop orbital-insertion guidance -> check that orbiter wiki page for PEG
// implement state vectors through lexicons (matrix) and remove API extraction dependency 
// work on getting the telescope mod for IMU alignments
// implement routines ASAP!


// clearscreen
set core:bootfilename to "lltv.ks".
clearScreen.

//run terminLIB.
runOncePath("terminLIB").
runOncePath("navballLIB").
SAS off. RCS on.
set terminal:charheight to 18.
set terminal:height to 21.
set terminal:width to 41.

// target
local tgtLand to latlng(0.0120544, 12.00151).
set targethoverslam to (tgtLand).

// ---- Initial variables ----
set program to 01.
set noun to 32.
set verb to 06.
set newVerb to false.
set oldVerb to 0.

set lastEventTime to time.
set compTime to time.

set monitorOnceFlag to false.
set descentFlag to false.
set deorbitFlag to false.
set enterDataFlag to true.

set R1BIT to true.
set R2BIT to true.
set R3BIT to true.
set RESTARTBIT to false. // write to json and read pgm/flags/BITs as necessary

set ROUTINES to lexicon().
ROUTINES:add("R30", false). // routine 30 -> orbital param
ROUTINES:add("R31", false). // routine 31 -> noun 54
ROUTINES:add("R36", false). // routine 36 -> docking param

set VAC_BANK to 0. // implement vec. accu. centers for job and waitlist logic

set tgtApo to 0.
set tgtPer to 0.
set tgtIncl to 0.

lock trueRadar to ship:bounds:bottomaltradar. // revert back to KSP collission box system since we're not using Waterfall anymore
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

// ---- Calculation Calls ----

// Trajectory formulation section

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

declare local function SLANT_RANGE {
    parameter groundTrack.
    //parameter stationAlt.
    //parameter height.

    //slant = sqrt(a^2 + b^2 + 2a.b.cos(CA))

    return abs(round(groundTrack/1000,1)).
}

// Landing and ascent guidance Section

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
    return arcCos(sin(ship:orbit:inclination) * sin(target:orbit:inclination) * cos(ship:orbit:longitudeofascendingnode - target:orbit:longitudeofascendingnode) + cos(ship:orbit:inclination) * cos(target:orbit:inclination)).
    //return (target:orbit:inclination - ship:orbit:inclination)*1000.
}

// Orbital Pairup Section

declare local function PLANE_CHANGE_DV { // dv needed to change inclinations
    parameter datum.
    return abs(2 * ship:velocity:orbit:mag * sin(datum)).
}

declare local function CUR_PHASE_ANGLE { // phase angle to target.
    parameter RNDZ_TGT.

    return vang(ship:position-body:position,RNDZ_TGT:position-body:position).
    
}

// need to fix this
declare local function TRNF_ORB_DATA { 
    // returns [0]: Time for transfer, [1]: Phase angle between target/current @ vernal equinox for intercept(how much target moves) [2]: dV @ ΔV1, [3]: dv @ ΔV2
    parameter RNDZ_TGT.

    local SMA to (ship:orbit:semimajoraxis + RNDZ_TGT:orbit:semimajoraxis)/2.
    local TRANSFER_TIME to sqrt(SMA^3 * 4 * constant:pi^2 / ship:body:mu)/2.

    local tgtDegPerMin to 360 / RNDZ_TGT:orbit:period.
    local transferPosit to 180 - (tgtDegPerMin * TRANSFER_TIME).
    

    // since Moon+CSM is in Earth SOI, we don't have to calculate escape vel + additional hyperbolic vel.
    local deltaV1 to sqrt(ship:body:mu/ship:orbit:semimajoraxis) * (sqrt(2 * RNDZ_TGT:orbit:semimajoraxis/(ship:orbit:semimajoraxis + RNDZ_TGT:orbit:semimajoraxis)) - 1).
    local deltaV2 to sqrt(ship:body:mu/RNDZ_TGT:orbit:semimajoraxis) * (1 - sqrt(2 * RNDZ_TGT:orbit:semimajoraxis/(ship:orbit:semimajoraxis + RNDZ_TGT:orbit:semimajoraxis))).

    return list(TRANSFER_TIME/60, transferPosit, deltaV1, deltaV2).
}


// Launch Section

declare local function launchAzimuth { // factoring in the target latitude, ship latitude, earth's rotating and orbital velocity to get needed launch azimuth

    local azimuth to arcSin(cos(target:orbit:inclination) / cos(ship:geoposition:lat)).
    local tgtOrbitVel to sqrt(ship:body:mu/(((tgtApo + tgtPer)*1000 + 2*ship:body:radius)/2)).
    local surfRotVel to cos(ship:geoposition:lat) * (2*constant:pi * ship:body:radius/ship:body:rotationperiod).
    local vRotX to tgtOrbitVel * sin(azimuth) - surfRotVel * cos(ship:geoposition:lat).
    local vRotY to tgtOrbitVel * cos(azimuth).
    
    local finalAzimuth to arcTan(vRotX/vRotY).

    return finalAzimuth.
    
}

declare local function LOAN_DIFF { // longitude of ascending node. if they match up or are below 0.5, then optimal time to launch
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
    // Health checks

    IMU_GimbalCheck().
    RESTARTCHECK().
    restartLightLogic(RESTARTBIT).

    // Program Section
    // I think the actual thing used a VN pair to show this information for various PGMs. Having the actual PGM as constraint is restrictive..

    if program <> 1 {
        print "    " at (2,11).
    }
    if program = 12 and noun = 93{ //most likely incorrect noun code
        registerDisplays(tgtApo, tgtPer, tgtIncl, true, "").
    }
    if program = 12 and noun = 94{ //most likely incorrect noun code
        registerDisplays(tgtApo, tgtPer, round(LOAN_DIFF(),2), true, "").
    }
    // Noun Section
    // Add an event timer for PGM 63 to 64 on pitchover.
    if noun = 32 {
        registerDisplays("", round(min(999, stage:deltaV:duration)), round(eta:periapsis), true, "").
        // make this create paste and delete the same file so first iter gets newest data
    }
    if noun = 34 {
        // call MET + time for P64 pitchover and other
        local DT to TIME_TO_EVENT().
        registerDisplays(DT[0], DT[1], DT[2], true, "").
    }
    if noun = 35 {
        local DT to TIME_FROM_EVENT().
        registerDisplays(DT[0], DT[1], DT[2], true, ""). // see if I can get TIME:HOUR/MIN/SEC conversion on each register
    }
    if noun = 42 {
        registerDisplays(round(ship:apoapsis/1000,1), round(ship:periapsis/1000,1), round(stage:deltaV:current), true, "").
    }
    if noun = 43 {
        registerDisplays(round(ship:geoposition:lat,1), round(ship:geoposition:lng,1), round(ship:altitude), true, "").
    }
    if noun = 44 {
        registerDisplays(round(ship:apoapsis/1000,1), round(ship:periapsis/1000,1), round(sqrt(ship:orbit:semimajoraxis^3 * constant:pi^2 * 4 / body:mu))/2, ROUTINES["R30"], "R30").
    }
    if noun = 54 {
        registerDisplays(round(errorDistance), round(ship:groundspeed), round(getBearingFromAtoB()), ROUTINES["R31"], "R31").
    }
    if noun = 61 {
        registerDisplays(round(targethoverslam:lat,1), round(targethoverslam:lng,1), "", true, "").
    }
    if noun = 67 {
        registerDisplays(SLANT_RANGE(ship:position:mag - targethoverslam:position:mag),round(ship:geoposition:lat,1), round(ship:geoposition:lng,1), true, "").
    }
    if noun = 73 {
        registerDisplays(round(ship:altitude), round(ship:velocity:mag), round(pitch_for(ship, prograde)), true, "").
    }
    if noun = 92 {
        registerDisplays(round(min(100, max(0, throtVal*100)))+"", round(ship:verticalspeed), round(trueRadar), true, "").
    }

    // Verb Section.

    if verb = 27 { // wishlist; make this VAC/RAM space. will have to define RAM
        registerDisplays((core:volume:freespace), "", "").
    }
}

declare local function registerDisplays {
    parameter R1, R2, R3, ROUT_VAL, ROUT_NAME.
   
    local ROUT_BOOL to ROUTINE_CHECK(ROUT_VAL, ROUT_NAME).
    VN_FLASH(verb, noun, ROUT_BOOL, ROUT_NAME).
    
    local regData to lexicon().
    regData:add("R1", R1).
    regData:add("R2", R2).
    regData:add("R3", R3).
    regData:add("ROUT_VAL", ROUT_BOOL).

    writeJson(regData, "regdata.json").

    // need to fix monitor flag.

    if monitorOnceFlag {
        print R1:tostring():padleft(7) at (33,11).
        print R2:tostring():padleft(7) at (33,13).
        print R3:tostring():padleft(7) at (33,15).
        set monitorOnceFlag to false.
    }

    //tostring():padleft(5) looks like a really good alternative.

    if R1BIT and ROUT_BOOL {
        print R1:tostring():padleft(7) at (33,11).
    }
    if R2BIT and ROUT_BOOL {
        print R2:tostring():padleft(7) at (33,13).
    }
    if R3BIT and ROUT_BOOL {
        print R3:tostring():padleft(7) at (33,15).
    }
}

declare local function ROUTINE_BOOL {

    for i in range(ROUTINES:length) {
        set ROUTINES["" + ROUTINES:keys[i] + ""] to false.
    }

    return true. // sets everything to false, then returns true for the routine to be set.
}

declare local function ROUTINE_CHECK {
    parameter ROUT_VAL, ROUT_NAME.
    if verb = 34 {
        set ROUTINES[ROUT_NAME] to false.
    }
    return ROUT_VAL.
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
    if verbChecker() and newVerb{
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

declare local function verbChecker { // Verb 37 is used to change program modes. So you enter verb 37, and then your program. For that reason, we gotta make a check
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
    if verb = 06 {
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
    if verb = 16 {
        set R1BIT to true.
        set R2BIT to true.
        set R3BIT to true.
    }
    if verb = 82 {
        set ROUTINES ["R30"] to ROUTINE_BOOL().
        set noun to 44.
        set verb to 16.
    }
    if verb = 83 {
        set ROUTINES ["R31"] to ROUTINE_BOOL().
        set noun to 54.
        set verb to 16.
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

declare local function progLightLogic { // saving this for the legendary 1202..
    parameter io.
    if io {
        print "PROG" at (10,11).
        wait 0.1.
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

declare local function VN_FLASH {
    parameter V, N, ROUT_VAL, ROUT_NAME.   
    if ROUT_VAL and ROUT_NAME <> ""{
        print V:tostring:padright(2) at (22, 9). print N:tostring:padright(2) at (32, 9).
        wait 0.25.
        print "":tostring:padright(2) at (22, 9). print "":tostring:padright(2) at (32, 9).
        wait 0.05.
    }
}

declare local function currentProgramParameterCheck { // program parameter input logic.
    if program = 12 and enterDataFlag {
        VN_FLASH(verb, noun, true, "data").
        set tgtApo to (terminal_input_string(33, 15)):toscalar().
        set tgtPer to (terminal_input_string(33, 15)):toscalar().
        if hasTarget {
            set tgtIncl to round(target:orbit:inclination,1).
        }
        else {
            set tgtIncl to (terminal_input_string(33, 15)):toscalar().
        }
    }
    VN_FLASH(verb, noun, false, "data").
}

// ---- Event checks  ----

declare local function CLOCK_CALL {
    parameter second.

    local hour to 0.
    local min to 0.
    local oldMin to 0.
    local sec to 0.

    set sec to round(second).
    if floor(second/60) >= 1 {
        set sec to mod(second, 60 * floor(second/60)).
    }

    set min to floor(second/60).
    set oldMin to floor(second/60).

    if floor(min/60) >= 1 {
        set min to mod(min, 60 * floor(min/60)).
    }

    set hour to floor(oldMin/60).

    return list(round(sec), min, hour).
}

declare local function TIME_TO_EVENT {
    
    if program = 63 { // time to p64 pitch over.
        local TTPO to time:seconds + abs(100 - ship:groundspeed/((ship:maxThrust/ship:mass) * cos(pitch_for(ship)))).
        if TTPO <= 1 {
            set lastEventTime to time:seconds + abs(100 - ship:groundspeed/((ship:maxThrust/ship:mass) * cos(pitch_for(ship)))).
            return list(lastEventTime:hour, lastEventTime:minute, lastEventTime:second).
        }
        local clock is CLOCK_CALL(time:seconds - TTPO).
        return list(clock[2], clock[1], clock[0]).
    }

    if program = 01 {
        local TT to time.
        return list(TT:hour, TT:minute, TT:second).
    }
}

declare local function TIME_FROM_EVENT {
    local clock is CLOCK_CALL(time:seconds - lastEventTime:seconds).
    return list(clock[2], clock[1], clock[0]).
}

declare local function COMPUTER_CLOCK_TIME {
    return list(time:hour - compTime:hour, time:minute - compTime:minute, time:second - compTime:second).
}

// ---- Ship Health and housekeeping  ----

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
        local dataOut to lexicon().
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
        wait .3.
        set RESTARTBIT to false.    
    }
}

// Start of computer software

set steeringManager:rollts to 3.
set steeringManager:pitchts to 3.
set steeringManager:yawts to 3.
agcStatic().
READ_LAST_KEYS().

until program = 00 {
    agcData().  

    if program = 1 {
        unlock steering.
        unlock throttle.
        print "STBY" at (2,11).
    }

    if program = 12 { // I was coincidentally lucky in naming this. P12 was a real program used to ascend from the Lunar surface
     if LOAN_DIFF() < 0.5 {
            lock throttle to 2 * getTwr().
            lock steering to heading(launchAzimuth()+yawReqPID:update(time:seconds, YAW_ASCENT_GUIDE()), 90-min(90, trueRadar/1000), 0).
            if tgtApo * 1000 <= ship:apoapsis {
                set program to 65.
            }
        }
    }

    if program = 63 { // velocity reduction
        if not descentFlag {
            if SLANT_RANGE(ship:position:mag - targethoverslam:position:mag) < 550 and not deorbitFlag {
                lock throttle to 0.1.
                if ship:periapsis < 12000 {
                    set deorbitFlag to true.
                }
            }
            if deorbitFlag {
                lock throttle to 1.
            }
        }
        if ship:verticalspeed < 0 {
            set descentFlag to true.
            //steeringCommand().
            lock steering to srfRetrograde * -r(pitchReqPID:update(time:seconds, PITCH_LAND_GUIDE()), yawReqPID:update(time:seconds, YAW_LAND_GUIDE()), 180). // set our steering so that we get to target.
            lock throttle to max(0.1, max(throtVal, sqrt(errorDistance)/300)).
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
        set throttlePid:setpoint to -(trueRadar/10).
        lock throttle to max(0.1, throttlePid:update(time:seconds, ship:verticalspeed)). // PGM 66, or rate of descent, lets us descent at a very slow rate.
        if trueRadar < 6 or ship:status = "landed" {
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