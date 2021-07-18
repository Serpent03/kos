// research open/closed loop orbital-insertion guidance -> check that orbiter wiki page for PEG
// implement state vectors through lexicons (matrix) and remove API extraction dependency 
// work on getting the telescope mod for IMU alignments
// implement routines ASAP! 
// implement executive and waitlist logic. 0.020  + 0.009
// implement routine calculation logic.
// Add P64 LPD displays and logic


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
addons:tr:settarget(targethoverslam).

// ---- Initial variables ----
set program to 01.
set noun to 44.
set verb to 15.
set newVerb to false.
set oldVerb to 0.

set lastEventTime to time.
set compBootTime to time.

set monitorOnceFlag to false.
set RNDZBURNFlag to false.
set PLANEFlag to false.
set RNDZFlag to false.
set descentFlag to false.
set deorbitFlag to false.
set rollFlag to false.
set enterDataFlag to true.

set R1BIT to true.
set R2BIT to true.
set R3BIT to true.
set R1OC to false.
set R2OC to false.
set R3OC to false.
set AUTO_MAN_BIT to true.
set UPDBIT to false.

set RESTARTBIT to false. // write to json and read pgm/flags/BITs as necessary

set ROUTINES to lexicon().
ROUTINES:add("R22", false). // routine 22 -> Rendezvous tracking data processing | P20
ROUTINES:add("R30", false). // routine 30 -> Orbit parameter display | V82
ROUTINES:add("R31", false). // routine 31 -> Landing Trajectory Error Calculations | V83
ROUTINES:add("R36", false). // routine 36 -> Rendezvous out-of-plane display | V90
ROUTINES:add("R38", false). // routine 38 -> LPD Angles, completely made up routine | P64
ROUTINES:add("R61", false). // routine 61 -> Tracking attitude | P20, R52, AUTO_MAN_BIT
ROUTINES:add("R63", false). // routine 63 -> Rendezvous final attitude | R61, V89

set VAC_BANK to lexicon(). // implement vec. accu. centers for exec and waitlist logic
VAC_BANK:add("LAND", false).
VAC_BANK:add("ORBT", false).
VAC_BANK:add("RNDZ", false).
VAC_BANK:add("ATTI", false).
VAC_BANK:add("TRAJ", false).
VAC_BANK:add("THRT", false).


set tgtApo to 0.
set tgtPer to 0.
set tgtIncl to 0.

list engines in eList.
lock trueRadar to ship:bounds:bottomaltradar. // revert back to KSP collission box system since we're not using Waterfall anymore
lock g0 to constant:g * ship:body:mass/ship:body:radius^2.
lock shipAcc to (ship:maxThrust/ship:mass) - g0.
lock decelHeight to (ship:verticalspeed^2/(2 * shipAcc)) * 2.
lock throtVal to decelHeight/trueRadar * 5.
lock errorDistance to distanceMag.


//  ---- PID Loops ----

set throttlePid to pidLoop(0.2, 0.05, 0.01, 0, 1).

local yawReqPID to pidLoop(0.6, 0.15, 0.02, -30, 30, 0.01).
set yawReqPID:setpoint to 0.

local pitchReqPID to pidLoop(0.6, 0.15, 0.02, -25, 25, 0.01).
set pitchReqPID:setpoint to -5.

// ---- Calculation Calls ----

// Miscellaneous Information

declare local function getEngineStability {
    for eng in eList {
        if eng:ignition {
            return eng:fuelstability.
        }
    }
}

// Trajectory formulation section

declare local function getBearingFromAtoB {	// get vector to heading(magnetic) between the predicted impact point and targeted impact point
    // B is target
    // A is impact pos from trajectories.
    set VAC_BANK["TRAJ"] to ADDONS:TR:HASIMPACT.
    
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
    set VAC_BANK["THRT"] to ADDONS:TR:HASIMPACT.

	local throttleSetting is g0 * (ship:mass/ship:availableThrust).
    return throttleSetting.
}

declare local function distanceMag { // error between predicted and target impact point
    set VAC_BANK["TRAJ"] to ADDONS:TR:HASIMPACT.
    
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
    set VAC_BANK["ORBT"] to true.
    set VAC_BANK["TRAJ"] to true.
    //parameter stationAlt.
    //parameter height.

    //slant = sqrt(a^2 + b^2 + 2a.b.cos(CA))

    return abs(round(groundTrack/1000,2)).
}

// Landing and ascent guidance Section

declare local function PITCH_LAND_GUIDE {	// TDAG EW - trajectory discrepancy avoidance guidance East <-> West.
    set VAC_BANK["ATTI"] to ADDONS:TR:HASIMPACT.
    if addons:tr:hasimpact {
	    return -(addons:tr:impactpos:lng - targetHoverslam:lng)*1000.
    }
    else {
        return 0.
    }
}


declare local function YAW_LAND_GUIDE {	// TDAG NS -trajectory discrepancy avoidance guidance for North <-> South.
    set VAC_BANK["ATTI"] to ADDONS:TR:HASIMPACT.
    if addons:tr:hasimpact {
	    return (addons:tr:impactpos:lat - targetHoverslam:lat)*1000.
    }
    else {
        return 0.
    }
}

declare local function YAW_ASCENT_GUIDE {
    set VAC_BANK["ATTI"] to true.
    return TRNF_ORB_DATA(vessel("TCSM"))[7] * 10.
}

declare local function LPD_DESIG {

    set VAC_BANK["ATTI"] to true.
    set VAC_BANK["TRAJ"] to true.

    if addons:tr:hasimpact and ROUTINES["R38"]{

        local vec1 to 90 - vang(ship:body:position, addons:tr:impactpos:position).
        local vec2 to addons:tr:impactpos:bearing/abs(addons:tr:impactpos:bearing).
        local vec3 to vec2 * (90-pitch_for(ship)).//90 - pitch_for(ship).
        local vec4 to min(60, max(0, ((vec1+vec3) * 2.30654761904) - 7)).

        return round(vec4).
    }
    else {
        return 0.
    }
}

// Orbital Pairup Section

declare local function PLANE_CHANGE_DV { // dv needed to change inclinations
    parameter datum.

    set VAC_BANK["ORBT"] to true.
    set VAC_BANK["RNDZ"] to true.

    
    return abs(2 * ship:velocity:orbit:mag * sin(datum)).
}

declare local function TRNF_ORB_DATA { 
    // returns:
    //[0]: Current phase angle, 
    //[1]: Transfer time in minutes
    //[2]: Transfer init phase angle,
    //[3]: dv @ ΔV1
    //[4]: dv @ ΔV2
    //[5]: Time to init phase angle
    //[6]: Distance to closest approach
    //[7]: Relative inclination 

    parameter RNDZ_TGT.

    set VAC_BANK["ORBT"] to true.
    set VAC_BANK["RNDZ"] to true.

    local phaseAngle to vang(ship:position-body:position,RNDZ_TGT:position-body:position).

    local SMA to (ship:orbit:semimajoraxis + RNDZ_TGT:orbit:semimajoraxis)/2.
    local TRANSFER_TIME to constant:pi * sqrt(SMA^3 / ship:body:mu).

    local ownShipAngularVel to sqrt(ship:body:mu/(ship:orbit:semimajoraxis)^3).
    local tgtShipAngularVel to sqrt(RNDZ_TGT:body:mu/(RNDZ_TGT:orbit:semimajoraxis)^3).
    local targetAngularTravel to tgtShipAngularVel * TRANSFER_TIME * constant:radtodeg.
    local transferPosit to (180 - targetAngularTravel).
    local waitTimeToTransfer to constant:degToRad * ((transferPosit-phaseangle)/(tgtShipAngularVel-ownShipAngularVel)).

    // since Moon+CSM is in Earth SOI, we don't have to calculate escape vel + additional hyperbolic vel.
    local deltaV1 to sqrt(ship:body:mu/ship:orbit:semimajoraxis) * (sqrt(2 * RNDZ_TGT:orbit:semimajoraxis/(ship:orbit:semimajoraxis + RNDZ_TGT:orbit:semimajoraxis)) - 1).
    local deltaV2 to sqrt(ship:body:mu/RNDZ_TGT:orbit:semimajoraxis) * (1 - sqrt(2 * RNDZ_TGT:orbit:semimajoraxis/(ship:orbit:semimajoraxis + RNDZ_TGT:orbit:semimajoraxis))).

    local closestAppr to phaseangle / (360 / ship:obt:period - 360 / target:obt:period).

    local inclination to arcCos(sin(ship:orbit:inclination) * sin(RNDZ_TGT:orbit:inclination) * cos(ship:orbit:longitudeofascendingnode - RNDZ_TGT:orbit:longitudeofascendingnode) + cos(ship:orbit:inclination) * cos(RNDZ_TGT:orbit:inclination)).

    return list(phaseAngle, TRANSFER_TIME/60, transferPosit, deltaV1, deltaV2, floor(abs(waitTimeToTransfer)), closestAppr, inclination).
}

declare local function RNDZ_ATT {
    // provide final attitude
    parameter RNDZ_TGT.
    
    set VAC_BANK["ATTI"] to true.
    return list(floor(RNDZ_TGT:direction:yaw), floor(RNDZ_TGT:direction:pitch), floor(RNDZ_TGT:direction:roll)).
}


// Launch Section

declare local function launchAzimuth { // factoring in the target latitude, ship latitude, earth's rotating and orbital velocity to get needed launch azimuth

    set VAC_BANK["TRAJ"] to true.
    set VAC_BANK["ORBT"] to true.

    
    
    local azimuth to arcSin(cos(target:orbit:inclination) / cos(ship:geoposition:lat)).
    local tgtOrbitVel to sqrt(ship:body:mu/(((tgtApo + tgtPer)*1000 + 2*ship:body:radius)/2)).
    local surfRotVel to cos(ship:geoposition:lat) * (2*constant:pi * ship:body:radius/ship:body:rotationperiod).
    local vRotX to tgtOrbitVel * sin(azimuth) - surfRotVel * cos(ship:geoposition:lat).
    local vRotY to tgtOrbitVel * cos(azimuth).
    
    local finalAzimuth to arcTan(vRotX/vRotY).

    return finalAzimuth.
    
}

declare local function LOAN_DIFF { // longitude of ascending node. if they match up or are below 0.5, then optimal time to launch
    set VAC_BANK["TRAJ"] to true.
    set VAC_BANK["RNDZ"] to true.

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
    print "       " at (2,13). print " " at (8,11).                                                            
                               print "" at (8,13).                                                          
    print "OPR ERR" at (2,15). print "" at (8,15).   print "TRACKER" at (10,15).                                                       
                                                                                                          
                                                                 
}

declare local function agcDataDisplay { // where we check what noun is active and display data based on that
    // Health and Event checks

    IMU_GimbalCheck().
    RESTARTCHECK().
    restartLightLogic(RESTARTBIT).
    ROUTINE_CHECKS().
    P_FLAG_CHECK().

    // Program Section
    // I think the actual thing used a VN pair to show this information for various PGMs. Having the actual PGM as constraint is restrictive..
    // actually I think this may be a ROUTINE tied to the program itself! Extended VERB 82 for example which should trigger R30 as used by P20

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
    if noun = 33 {
        local DT to TIME_TO_IGNITION().
        registerDisplays(DT[0], DT[1], DT[2], true, "").
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
    if noun = 36 {
        local DT to COMPUTER_CLOCK_TIME().
        registerDisplays(DT[0], DT[1], DT[2], true, ""). 
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
        registerDisplays(SLANT_RANGE(ship:position:mag - targethoverslam:position:mag),round(ship:geoposition:lat,2), round(ship:geoposition:lng,2), true, "").
    }
    if noun = 68 {
        registerDisplays(LPD_DESIG(), "", "", true, "").
    }
    if noun = 73 {
        registerDisplays(round(ship:altitude), round(ship:velocity:mag), round(pitch_for(ship, prograde),2), true, "").
    }
    if noun = 78 {
        local EG to RNDZ_ATT(vessel("TCSM")).
        registerDisplays(EG[0], EG[1], EG[2], ROUTINES["R63"], "R63").
    }
    // N78 -> YAW ANGLE, PITCH ANGLE, AZIMUTH CONSTR | P20, R61, R63
    // N90 -> Y ACTIVE VEH, Ŷ ACTIVE VEH, Ŷ PASSIVE VEH | P20, R36
    if noun = 92 {
        registerDisplays(round(min(100, max(0, throttle*100))), round(ship:verticalspeed), round(trueRadar), true, "").
    }

    // Verb Section.

    if verb = 27 { // wishlist; make this VAC/RAM space. will have to define RAM
        registerDisplays(VAC_ACCUMULATION(), "", "", true, "").
    }
}

declare local function registerDisplays {
    parameter R1, R2, R3, ROUT_VAL, ROUT_NAME.
   
    
    local ROUT_BOOL to ROUTINE_CANCEL(ROUT_VAL, ROUT_NAME).
    VN_FLASH(verb, noun, ROUT_BOOL, ROUT_NAME).

    local REGVALS to TO_OCTAL(R1, R2, R3, R1OC, R2OC, R3OC).
    
    local regData to lexicon().
    regData:add("R1", REGVALS[0]).
    regData:add("R2", REGVALS[1]).
    regData:add("R3", REGVALS[2]).
    regData:add("ROUT_VAL", ROUT_BOOL).

    writeJson(regData, "regdata.json").

    // need to fix monitor flag.
    // instead of monitor flag use V16 monitor pairup for a single update iter

    if monitorOnceFlag {
        print REGVALS[0]:padleft(7) at (33,11).
        print REGVALS[1]:padleft(7) at (33,13).
        print REGVALS[2]:padleft(7) at (33,15).
        set monitorOnceFlag to false.
    }

    if UPDBIT { // so this should ensure that values get displayed but not update unless in ROUTINE
        print REGVALS[0]:padleft(7) at (33,11).
        print REGVALS[1]:padleft(7) at (33,13).
        print REGVALS[2]:padleft(7) at (33,15).
        set UPDBIT to false.
    }

    //tostring():padleft(5) looks like a really good alternative.

    if R1BIT and ROUT_BOOL {
        print REGVALS[0]:padleft(7) at (33,11).
    }
    if R2BIT and ROUT_BOOL {
        print REGVALS[1]:padleft(7) at (33,13).
    }
    if R3BIT and ROUT_BOOL {
        print REGVALS[2]:padleft(7) at (33,15).
    }
}

declare local function ROUTINE_BOOL {

    for i in range(ROUTINES:length) {
        set ROUTINES["" + ROUTINES:keys[i] + ""] to false.
    }

    return true. // sets everything to false, then returns true for the routine to be set.
}

declare local function ROUTINE_CANCEL {
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
        set UPDBIT to true.
    }
    else if terminal:input:getchar() = "-" {
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
        set verb to oldVerb.
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
        set R1OC to true.
    }
    if verb = 02 {
        set monitorOnceFlag to true.
        set R2BIT to false.
        set R2OC to true.
    }
    if verb = 03 {
        set monitorOnceFlag to true.
        set R3BIT to false.
        set R3OC to true.
    }
    if verb = 04 {
        set monitorOnceFlag to true.
        set R1BIT to false.
        set R2BIT to false.
        set R1OC to true.
        set R2OC to true.
    }
    if verb = 05 {
        set monitorOnceFlag to true.
        set R1BIT to false.
        set R2BIT to false.
        set R3BIT to false.
        set R1OC to true.
        set R2OC to true.
        set R3OC to true.
    }
    if verb = 06 {
        set monitorOnceFlag to true.
        set R1BIT to false.
        set R2BIT to false.
        set R3BIT to false.
        set R1OC to false.
        set R2OC to false.
        set R3OC to false.
    }
    if verb = 11 {
        set R1BIT to true.
        set R1OC to true.
    }
    if verb = 12 {
        set R2BIT to true.
        set R2OC to true.
    }
    if verb = 13 {
        set R3BIT to true.
        set R3OC to true.
    }
    if verb = 14 {
        set R1BIT to true.
        set R2BIT to true.
        set R1OC to true.
        set R2OC to true.
    }
    if verb = 15 {
        set R1BIT to true.
        set R2BIT to true.
        set R3BIT to true.
        set R1OC to true.
        set R2OC to true.
        set R3OC to true.
    }
    if verb = 16 {
        set R1BIT to true.
        set R2BIT to true.
        set R3BIT to true.
        set R1OC to false.
        set R2OC to false.
        set R3OC to false.
    }
    if verb = 58 {
        toggle AUTO_MAN_BIT.
        set verb to oldVerb.
    }
    if verb = 82 {
        set ROUTINES["R30"] to ROUTINE_BOOL().
        set noun to 44.
        set verb to 16.

    }
    if verb = 83 {
        set ROUTINES["R31"] to ROUTINE_BOOL().
        set noun to 54.
        set verb to 16.
    }
    if verb = 89 {
        set ROUTINES["R63"] to ROUTINE_BOOL().
        set noun to 78.
        set verb to 16.
    }
    if verb = 90 {
        set ROUTINES["R36"] to ROUTINE_BOOL().
        set noun to 90.
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

declare local function ROUTINE_CHECKS {
    if program = 20 or ROUTINES["R36"] or ROUTINES["R22"]{
        RNDZ_STATE_CHECK().
    }
}

declare local function TO_OCTAL {
    local parameter REG1, REG2, REG3, REG1oc, REG2oc, REG3oc.
    local REG1L to list().
    local REG2L to list().
    local REG3L to list().

    if REG1 = "" {
        set REG1oc to false.
    }
    if REG2 = "" {
        set REG2oc to false.
    }
    if REG3 = "" {
        set REG3oc to false.
    }

    if REG1oc{
        // convert to Octal
        
        REG1L:add(REG1:tostring():toscalar()).
        set REG1 to "".
        until REG1L[REG1L:length-1] < 8 {
            REG1L:add(floor(REG1L[REG1L:length-1]/8)).
        }
        for i in range(REG1L:length-1, -1) {
            set REG1 to REG1 + mod(REG1L[i], 8).
        }
        set REG1 to round(REG1:toscalar()).
    }
    if REG2oc{
        // convert to Octal
        REG2L:add(REG2:tostring():toscalar()).
        set REG2 to "".
        until REG2L[REG2L:length-1] < 8 {
            REG2L:add(floor(REG2L[REG2L:length-1]/8)).
        }
        for i in range(REG2L:length-1, -1) {
            set REG2 to REG2 + mod(REG2L[i], 8).
        }
        set REG2 to round(REG2:toscalar()).
    }
    if REG3oc{
        // convert to Octal
        REG3L:add(REG3:tostring():toscalar()).
        set REG3 to "".
        until REG3L[REG3L:length-1] < 8 {
            REG3L:add(floor(REG3L[REG3L:length-1]/8)).
        }
        for i in range(REG3L:length-1, -1) {
            set REG3 to REG3 + mod(REG3L[i], 8).
        }
        set REG3 to round(REG3:toscalar()).
    }

    return list(REG1:tostring(), REG2:tostring(), REG3:tostring()).
}

declare local function VN_FLASH {
    parameter V, N, ROUT_VAL, ROUT_NAME.   
    if ROUT_VAL and ROUT_NAME <> ""{
        print V:tostring:padright(2) at (22, 9). print N:tostring:padright(2) at (32, 9).
        wait 0.15.
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

declare local function TIME_TO_IGNITION {
    local timeToIgn to 0.
    if program = 20 { // p20 rndz fire or plane change.
        
        if RNDZFlag {
            set timeToIgn to TRNF_ORB_DATA(vessel("TCSM"))[5].    
            if TRNF_ORB_DATA(vessel("TCSM"))[5] < 1 {
                set timeToIgn to time:seconds + TRNF_ORB_DATA(vessel("TCSM"))[1]*60.
                set lastEventTime to time:seconds.
            }
        }
        if PLANEFlag {
            set timeToIgn to "ETA A/N or D/N".
            if "ETA A/N or D/N" < 1 {
                set timeToIgn to 0.
                set lastEventTime to time:seconds.
            }
        }
    }
    local clock is CLOCK_CALL(timeToIgn).
    return list(clock[2], clock[1], clock[0]).
}

declare local function TIME_TO_EVENT {
    local ET to 0.
    if program = 20 { // p20 rndz fire or plane change.
        
        if RNDZFlag {
            set ET to TRNF_ORB_DATA(vessel("TCSM"))[5].    
            if TRNF_ORB_DATA(vessel("TCSM"))[5] < 1 or abs(TRNF_ORB_DATA(vessel("TCSM"))[0] - TRNF_ORB_DATA(vessel("TCSM"))[2]) <= 0.01{
                set ET to time:seconds + TRNF_ORB_DATA(vessel("TCSM"))[1]*60.
                set lastEventTime to time.
            }
        }
        if PLANEFlag {
            set ET to "ETA A/N or D/N".
            if "ETA A/N or D/N" < 1 {
                set ET to 0.
                set lastEventTime to time.
            }
        }
        local clock is CLOCK_CALL(ET).
        return list(clock[2], clock[1], clock[0]).
    }

    if program = 63 { // time to p64 pitch over.
        set ET to time:seconds + abs(100 - ship:groundspeed/((ship:maxThrust/ship:mass) * cos(pitch_for(ship)))).
        if ET <= 1 {
            set lastEventTime to time:seconds + abs(100 - ship:groundspeed/((ship:maxThrust/ship:mass) * cos(pitch_for(ship)))).
            return list(lastEventTime:hour, lastEventTime:minute, lastEventTime:second).
        }
        local clock is CLOCK_CALL(time:seconds - ET).
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
    local clock is CLOCK_CALL(time:seconds - compBootTime:seconds).
    return list(clock[2], clock[1], clock[0]).
}

declare local function RNDZ_STATE_CHECK {
    set VAC_BANK["ORBT"] to true.
    set VAC_BANK["RNDZ"] to true.

    set RNDZFlag to TRNF_ORB_DATA(vessel("TCSM"))[7] <= 0.2.
    set PLANEFlag to RNDZFlag. toggle PLANEFlag.
}

declare local function P_FLAG_CHECK {
    
    if program = 20 {
        if not RNDZBURNFlag {
            set VAC_BANK["THRT"] to true.
            if TRNF_ORB_DATA(vessel("TCSM"))[0] < 10 and TRNF_ORB_DATA(vessel("TCSM"))[0] > 5{
                set ship:control:fore to 1-getEngineStability().
                set RNDZBURNFlag to getEngineStability() = 1.
            }
        }
    }

    if program = 63 {
        if not rollFlag and descentFlag {
            set VAC_BANK["ATTI"] to true.
            set rollFlag to errorDistance < 10000.
        }
    }
}

// ---- Ship Health and housekeeping  ----

//declare local function ullageChecks {
//    local stab to getEngineStability().
//    set ship:control:fore to 1-stab.
//    return stab = 1.
//}

declare local function IMU_GimbalCheck {
    set VAC_BANK["ATTI"] to true.
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

declare local function VAC_ACCUMULATION {
    local VAC_COUNT to 0.

    for i in range(VAC_BANK:length) {
        if VAC_BANK:values[i] {
            set VAC_COUNT to VAC_COUNT + 1.
        }
    }

    set RESTARTBIT to VAC_COUNT > 5.
    progLightLogic(VAC_COUNT > 5).
    return VAC_COUNT.
}

declare local function VAC_CLEAR {
    for i in range(VAC_BANK:length) {
        set VAC_BANK["" + VAC_BANK:keys[i] + ""] to false.
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

set steeringManager:rollts to 1.875.
set steeringManager:pitchts to 1.875.
set steeringManager:yawts to 1.875.
set steeringManager:rollcontrolanglerange to 180.

agcStatic().
READ_LAST_KEYS().
VAC_CLEAR().

until program = 00 {
    agcData().
    VAC_ACCUMULATION().

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
    set program to 1. VAC_CLEAR().
    }

    if program = 20 { // orbital rendezvous
        set ROUTINES["R61"] to true.
        set ROUTINES["R63"] to true.
        local orbitalData to TRNF_ORB_DATA(vessel("TCSM")).

        if orbitalData[0] < 0.02 and orbitalData[2] < 0.02 { // phase angle. change this to distance
            lock throttle to 0. unlock steering.
        }
        
        if RNDZFlag {
            lock steering to prograde.// * (orbitalData[3])/abs(orbitalData[3]).
            if orbitalData[5] <= 90 {set warp to 0.}
            if RNDZBURNFlag {    //time to target phase angle
                set RNDZBURNFlag to true.
                lock throttle to orbitalData[3].
                if abs(orbitalData[3]) < 1 {
                    lock throttle to 0.
                }
            }
        }
        if PLANEFlag {
            lock steering to vcrs(ship:velocity:orbit,-body:position).// * PLANE_CHANGE_DV(orbitalData[7])/abs(PLANE_CHANGE_DV(orbitalData[7])).
            lock throttle to PLANE_CHANGE_DV(orbitalData[7]).
        }
    }

    if program = 63 { // velocity reduction
        if not descentFlag {
            if SLANT_RANGE(ship:position:mag - targethoverslam:position:mag) < 500 and not deorbitFlag {
                lock throttle to 1 * getTwr().
                lock steering to srfRetrograde.
                if ship:periapsis < 12000 {
                    set deorbitFlag to true.
                }
            }
            if deorbitFlag {
                lock throttle to 3 * getTwr().
            }
        }
        if ship:verticalspeed < -5 {
            set descentFlag to true.
            //steeringCommand().
            if rollFlag {
                lock steering to srfRetrograde * -r(pitchReqPID:update(time:seconds, PITCH_LAND_GUIDE()), yawReqPID:update(time:seconds, YAW_LAND_GUIDE()), 172). // set our steering so that we get to target.
            }
            if not rollFlag {
                lock steering to srfRetrograde * -r(0, yawReqPID:update(time:seconds, -YAW_LAND_GUIDE()), 0).
            }
            lock throttle to max(0.1, max(throtVal, sqrt(errorDistance)/300)).
            if ship:groundspeed < 100 {
                set program to 64.
                VAC_CLEAR().
            }
        }
    }

    if program = 64 { // trajectory control
        lock throttle to max(0.1, throtVal/2).
        set ROUTINES["R38"] to true.
        set pitchReqPID:setpoint to 0.
        // right, so 64 is initiated pitch up due to slow horizontal velocity and initial P63 offset.
        // so set the setpoints back to 0 and include the LPD angle and put tighter constraints on the retrograde AoA
        // add a LPD display & change(tie that into the WASD keys without SAS bool) function
        // possible LPD via vecDraw() from resultant velocity
        // and shift P64 to UP+(). then WASD controls the LZ

        if SAS {
            set program to 66. unlock steering. VAC_CLEAR().
        }
    }

    if program = 66 {
        set throttlePid:setpoint to -(trueRadar/10).
        lock throttle to max(0.1, max(throtVal/2, throttlePid:update(time:seconds, ship:verticalspeed))). // PGM 66, or rate of descent, lets us descent at a very slow rate.
        if trueRadar < 0.5 or ship:status = "landed" {
            set program to 68. // program 68 is confirmation of touchdown.
            VAC_CLEAR().
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