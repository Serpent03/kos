// research open/closed loop orbital-insertion guidance -> check that orbiter wiki page for PEG
// implement state vectors through lexicons (matrix) and remove API extraction dependency 
// work on getting the telescope mod for IMU alignments
// implement executive and waitlist logic. 0.020  + 0.009
// implement ECADR target option code.
// Star codes!

// P20 + PXX: https://history.nasa.gov/afj/loressay.html

// Implemented
// P12 {not fully, GNDTRK, CIRBU}
// P20 {not fully, MARKS}
// P32 CSI - Coelleptic Sequence Initiate.
// P34 TPI - Terminal Phase Initiation
// P35 TPF - Transfer Phase {missing: MID_CRS_CORR}
// P63
// P64
// P66 

// Not Implemented {TODO}
// P31 HAM - Height Adjustment Maneuver. 
// P33 CDH - Constant Delta H. Maintain constant 28km orbit less than CSM
// P36 PCM - Plane Change Maneuver
// P37 RTE - Return to Earth
// P40 SPS - Thrusting. Service Prop System
// P41 RCS - Thrusting.  
// P47 - Thrust Monitoring. RVEL, Range, Range Rate
// P76 LM TGT DV - LM TIG and change in orbital vel
// P79 Final RNDZ - Range, Range Rate, Angular Difference between X-axis(LEM, facing straight through passive port)

// Make a kv pair of rec vn

// clearscreen
set config:obeyhideui to false.
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
set program to 1.
set noun to 44.
set verb to 16.
set newVerb to false.
set oldVerb to 0.
set oldNoun to 0.
set oldProgram to 0.
set progNoun to 0.  // program recommended noun
set progVerb to 0.  // program recommended verb
set ERR to 0.

set tgtVessel to vessel("TCSM").

set lastEventTime to time.
set compBootTime to time.

set monitorOnceFlag to false. //MOF | 1517
set BURN_FLAG to false. //BRN | 
set TPI_PRE_FLAG to false. //RBF | 2202
set TPI_POS_FLAG to false. // PTP | 2024
set PLANEFlag to false. //PLF | 2014
set RNDZFlag to false. //RDF | 2204
set descentFlag to false. //DEF | 0405
set deorbitFlag to false. //DOF | 0417
set rollFlag to false. //ROF | 2217
set surfaceFlag to false. //SRF | 
set performMINKEY to false. //PMK | 2015
set enterDataFlag to true.
set progRecycleFlag to false.
set proceedFlag to false.

set IDLEBIT to false.
set R1BIT to true.
set R2BIT to true.
set R3BIT to true.
set R1OC to false.
set R2OC to false.
set R3OC to false.
set AUTO_MAN_BIT to true.
set PROGBIT to false.
set OEBIT to false.
set UPDBIT to false.
set P20BIT to false.
set CABIT to false.
set CMPACTY to false.
set PL_PERF_BIT to false.

set RESTARTBIT to false. // write to json and read pgm/flags/BITs as necessary

set ROUTINES to lexicon().
ROUTINES:add("R22", false). // routine 22 -> Rendezvous tracking data processing | P20
ROUTINES:add("R30", false). // routine 30 -> Orbit parameter display | V82
ROUTINES:add("R31", false). // routine 31 -> Landing Trajectory Error Calculations | V83
ROUTINES:add("R36", false). // routine 36 -> Rendezvous out-of-plane display | V90
ROUTINES:add("R38", false). // routine 38 -> LPD Angles, completely made up routine | P64
ROUTINES:add("R61", false). // routine 61 -> Tracking attitude | P20, R52, AUTO_MAN_BIT
ROUTINES:add("R62", false). // routine 62 -> Start Crew Defined MNVR | V49
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
lock trueRadar to ship:bounds:bottomaltradar. // revert back to KSP collision box system since we're not using Waterfall anymore
lock g0 to constant:g * ship:body:mass/ship:body:radius^2.
lock shipAcc to (ship:maxThrust/ship:mass) - g0.
lock decelHeight to (ship:verticalspeed^2/(2 * shipAcc)) * 2.
lock throtVal to decelHeight/trueRadar * 2.
lock errorDistance to distanceMag.


//  ---- PID Loops ----

set throttlePid to pidLoop(0.2, 0.05, 0.01, 0, 1).
set throttlePid:setpoint to -10.

local yawReqPID to pidLoop(0.01, 0.015, 0.02, -30, 30).
set yawReqPID:setpoint to 0.

local pitchReqPID to pidLoop(0.01, 0.0015, 0.02, -30, 0).
set pitchReqPID:setpoint to 5.

// ---- Calculation Calls ----

// Miscellaneous Information

declare local function getEngineStability { // Engine ullage needs
    for eng in eList {
        if eng:ignition {
            return eng:fuelstability.
        }
    }
}

declare local function engineIgnitionPermission {
    if proceedFlag {
        return 1.
    }
    else {
        if verb <> 99 set verb to 99.
        return 0.
    }
}

// Trajectory formulation section

declare local function getBearingFromAtoB {	// get vector to heading(magnetic) between the predicted impact point and targeted impact point
    // B is target
    // A is impact pos from trajectories.
    if not VAC_BANK["TRAJ"] {set VAC_BANK["TRAJ"] to true.}
    
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
    if not VAC_BANK["THRT"] {set VAC_BANK["THRT"] to true.}

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

declare local function SLANT_RANGE { // Currently ground track range
    if not VAC_BANK["ORBT"] {set VAC_BANK["ORBT"] to true.}
    if not VAC_BANK["TRAJ"] {set VAC_BANK["TRAJ"] to true.}

    parameter groundTrack.
    //parameter stationAlt.
    //parameter height.

    //slant = sqrt(a^2 + b^2 + 2a.b.cos(CA))

    return abs(round(groundTrack/1000,2)).
}

// Landing and ascent guidance Section

declare local function PITCH_LAND_GUIDE {	// TDAG EW - trajectory discrepancy avoidance guidance East <-> West.
    if not VAC_BANK["ATTI"] {set VAC_BANK["ATTI"] to true.}

    if addons:tr:hasimpact {
        if program = 64{
	        return -(addons:tr:impactpos:lng - targetHoverslam:lng)*1000.
        }
        if program = 63 {
            return -(addons:tr:impactpos:lng - (targetHoverslam:lng-0.25))*1000. //convert to position vector, * 5000 meter offset
        }
    }
    else {
        return 0.
    }
}


declare local function YAW_LAND_GUIDE {	// TDAG NS - trajectory discrepancy avoidance guidance for North <-> South.
    if not VAC_BANK["ATTI"] {set VAC_BANK["ATTI"] to true.}

    if addons:tr:hasimpact {
	    return (addons:tr:impactpos:lat - targetHoverslam:lat)*1000.
    }
    else {
        return 0.
    }
}

declare local function YAW_ASCENT_GUIDE {
    if not VAC_BANK["ATTI"] {set VAC_BANK["ATTI"] to true.}

    return ROUTINE_CALCS()[7] * 10.
}

declare local function LPD_DESIG { // Output LPD ladder trajectory impact

    if not VAC_BANK["ATTI"] {set VAC_BANK["ATTI"] to true.}
    if not VAC_BANK["TRAJ"] {set VAC_BANK["TRAJ"] to true.}

    if addons:tr:hasimpact and ROUTINES["R38"]{

        local vec1 to 90 - vang(ship:body:position, addons:tr:impactpos:position).
        if abs(addons:tr:impactpos:bearing) > 90 { // depending on the direction of craft, this will edit the displayed LPD angle
            set vec2 to 1.
        }
        else {
            set vec2 to -1.
        }
        //if abs(addons:tr:impactpos:bearing) > 90 and abs(addons:tr:impactpos:bearing) < 180 s
        local vec3 to vec2 * (90-pitch_for(ship)).//90 - pitch_for(ship).
        local vec4 to min(60, max(0, ((vec1+vec3) * 2.30654761904) - 7)).

        return round(vec4).
    }
    else {
        return 0.
    }
}

declare local function RAD_ALT {
    if rollFlag{
        return round(abs(ship:geoposition:terrainheight - ship:altitude)).
    }
    else {
        return 0.
    }
}

// Orbital Pairup Section

declare local function CSI_CALC {
    //[0]: ETA to Apogee/Lune
    //[1]: ETA to Perigee/Lune
    //[2]: DV for Apogee/Lune Raise
    //[3]: DV for Perigee/Lune Raise
    //[4]: Apogee/Lune Raise flag
    //[5]: Perigee/Lune Raise flag

    parameter RNDZ_TGT.

    if not VAC_BANK["RNDZ"] {set VAC_BANK["RNDZ"] to true.}  
    if not VAC_BANK["ORBT"] {set VAC_BANK["ORBT"] to true.}

    local etaToApo to eta:apoapsis.
    local etaToPer to eta:periapsis.
    
    local apoRaiseDV to ((RNDZ_TGT:orbit:apoapsis - ship:orbit:apoapsis)/1000)-28.
    local perRaiseDV to ((RNDZ_TGT:orbit:periapsis - ship:orbit:periapsis)/1000)-28.

    local apoFlag to abs(apoRaiseDV) >= 4.
    local perFlag to abs(perRaiseDV) >= 4 and not apoFlag.

    return list(etaToApo, etaToPer, apoRaiseDV, perRaiseDV, apoFlag, perFlag).

}

declare local function PLANE_CHANGE_DV { // dv needed to change inclinations
    parameter datum.

    if not VAC_BANK["RNDZ"] {set VAC_BANK["RNDZ"] to true.}  
    if not VAC_BANK["ORBT"] {set VAC_BANK["ORBT"] to true.}  

    
    return abs(2 * ship:velocity:orbit:mag * sin(datum)).
}

declare local function TRNF_ORB_DATA { 
    //[0]: Current phase angle Φ, 
    //[1]: Transfer time in minutes
    //[2]: Transfer init phase angle φ,
    //[3]: dv @ ΔV1
    //[4]: dv @ ΔV2
    //[5]: Time to init phase angle
    //[6]: Distance to closest approach
    //[7]: Relative inclination θ

    parameter RNDZ_TGT.

    if not VAC_BANK["ATTI"] {set VAC_BANK["ATTI"] to true.}
    if not VAC_BANK["ORBT"] {set VAC_BANK["ORBT"] to true.}
    if not VAC_BANK["RNDZ"] {set VAC_BANK["RNDZ"] to true.}

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

    local closestAppr to phaseangle / (360 / ship:obt:period - 360 / RNDZ_TGT:obt:period).

    local inclination to arcCos(sin(ship:orbit:inclination) * sin(RNDZ_TGT:orbit:inclination) * cos(ship:orbit:longitudeofascendingnode - RNDZ_TGT:orbit:longitudeofascendingnode) + cos(ship:orbit:inclination) * cos(RNDZ_TGT:orbit:inclination)).

    return list(phaseAngle, TRANSFER_TIME/60, transferPosit, deltaV1, deltaV2, floor(abs(waitTimeToTransfer)), closestAppr, inclination).
}

declare local function RNDZ_ATT {
    // provide final attitude
    parameter RNDZ_TGT.
    
    if not VAC_BANK["ATTI"] {set VAC_BANK["ATTI"] to true.}
    return list(floor(RNDZ_TGT:direction:yaw), floor(RNDZ_TGT:direction:pitch), floor(RNDZ_TGT:direction:roll)).
}


// Launch Section

declare local function launchAzimuth { // factoring in the target latitude, ship latitude, earth's rotating and orbital velocity to get needed launch azimuth

    set VAC_BANK["TRAJ"] to true.
    set VAC_BANK["ORBT"] to true.
    
    parameter RNDZ_TGT.
    
    
    local azimuth to arcSin(cos(RNDZ_TGT:orbit:inclination) / cos(ship:geoposition:lat)).
    local tgtOrbitVel to sqrt(ship:body:mu/(((tgtApo + tgtPer)*1000 + 2*ship:body:radius)/2)).
    local surfRotVel to cos(ship:geoposition:lat) * (2*constant:pi * ship:body:radius/ship:body:rotationperiod).
    local vRotX to tgtOrbitVel * sin(azimuth) - surfRotVel * cos(ship:geoposition:lat).
    local vRotY to tgtOrbitVel * cos(azimuth).
    
    local finalAzimuth to arcTan(vRotX/vRotY).

    return finalAzimuth.
    
}

declare local function LOAN_DIFF { // longitude of ascending node. if they match up or are below 0.5, then optimal time to launch

    parameter RNDZ_TGT.

    set VAC_BANK["TRAJ"] to true.
    set VAC_BANK["RNDZ"] to true.

    return round(abs(ship:orbit:longitudeofascendingnode - RNDZ_TGT:orbit:longitudeofascendingnode),3).
}


//  ---- Data Manipulation Functions ----

declare local function agcStatic { // Static items in the display.
                                                print "┏━━━━━━━━━━━━━━━━━┓ " at (0.5,4).                                                                                                 print "┏━━━━━━━━━━━━━━━━━━━┓" at (20.5,4).                                          
    print "┃" at (.5, 5).                                                                                   print "┃" at (17.5, 5).   print "┃" at (20.5, 5).                           print " " at (30,5). print "PROG" at (32,5).                  print "┃" at (39.5, 5).                 
    print "┃" at (.5, 6).                                                                                   print "┃" at (17.5, 6).   print "┃" at (20.5, 6).                           print " " at (30,6).                                          print "┃" at (39.5, 6).             
    print "┃" at (.5, 7).                                                                                   print "┃" at (17.5, 7).   print "┃" at (20.5, 7).   print "VERB" at (22,8). print " " at (30,8). print "NOUN" at (32,8).                  print "┃" at (39.5, 7).             
    print "┃" at (.5, 8).                                                                                   print "┃" at (17.5, 8).   print "┃" at (20.5, 8).                                                                                         print "┃" at (39.5, 8).             
    print "┃" at (.5, 9).                                                                                   print "┃" at (17.5, 9).   print "┃" at (20.5, 9).   print "──────────────────" at (22,10).                                                print "┃" at (39.5, 9).             
    print "┃" at (.5, 10).                                                                                  print "┃" at (17.5, 10).  print "┃" at (20.5, 10).                                                                                        print "┃" at (39.5, 10).            
    print "┃" at (.5, 11).                                                                                  print "┃" at (17.5, 11).  print "┃" at (20.5, 11).  print "──────────────────" at (22,12).                                                print "┃" at (39.5, 11).            
    print "┃" at (.5, 12).                      print "┗━━━━━━━━━━━━━━━━━┛" at (.5,16).                      print "┃" at (17.5, 12).  print "┃" at (20.5, 12).                                                                                        print "┃" at (39.5, 12).
    print "┃" at (.5, 13).                                                                                  print "┃" at (17.5, 13).  print "┃" at (20.5, 13).  print "──────────────────" at (22,14).                                                print "┃" at (39.5, 13).
    print "┃" at (.5, 14).                                                                                  print "┃" at (17.5, 14).  print "┃" at (20.5, 14).                                                                                        print "┃" at (39.5, 14).
    print "┃" at (.5, 15).                                                                                  print "┃" at (17.5, 15).  print "┃" at (20.5, 15).                      print "┗━━━━━━━━━━━━━━━━━━━┛" at (20.5,16).                           print "┃" at (39.5, 15).

}

declare local function agcData { // this thing was a fucking PAIN to write, LOL
    agcStatic().
    agcDataDisplay().
                                                                                                                                                                                   
                               print " " at (8,5).   print "TEMP" at (10,5).                                                                         
                               print " " at (8,6).                                                                                      print program + " " at (32, 6).          
                               print " " at (8,8).                              
    print "   " at (2,9).      print " " at (8,9).                                     //print verb + " " at (22,9). print " " at (30,9). print noun + " " at (32,9).          
                               print " " at (8,11).                                                            
                               print "" at (8,13).                                                          
                               print "" at (8,15).   print "TRACKER" at (10,15).                                                       
                                                                                                                                                                       
}

declare local function agcDataDisplay { // where we check what noun is active and display data based on that
    // Health and Event checks

    local dt to time:seconds.
    set CABIT to round(mod(dt, 1)) <> 0.
    set CMPACTY to floor(mod(dt, 2)) = 0 and not IDLEBIT. 
    RESTARTCHECK().
    restartLightLogic(RESTARTBIT).
    P_FLAG_CHECK().
    REC_VN_CHECK().
    DATA_LIGHTS().
    VNP_DATA().

}

declare local function VNP_DATA {
    // Program Section
    // I think the actual thing used a VN pair to show this information for various PGMs. Having the actual PGM as constraint is restrictive..
    // actually I think this may be a ROUTINE tied to the program itself! Extended VERB 82 for example which should trigger R30 as used by P20

    if program <> 01 {
        print "    " at (2,11).
    }
    if program = 12 and noun = 93{ //most likely incorrect noun code
        registerDisplays(tgtApo, tgtPer, tgtIncl, true, "").
    }
    if program = 12 and noun = 94{ //most likely incorrect noun code
        registerDisplays(tgtApo, tgtPer, round(LOAN_DIFF(tgtVessel),2), true, "").
    }
    // Noun Section
    // Add an event timer for PGM 63 to 64 on pitchover.
    if noun = 32 {
        local ETAP to CLOCK_CALL(eta:periapsis).
        registerDisplays(ETAP[2], ETAP[1], ETAP[0], true, "").
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
    if noun = 47 {
        registerDisplays(round(ship:mass)/1000, round(tgtVessel:mass)/1000, "", true, "").
    }
    if noun = 54 {
        registerDisplays(round(errorDistance), round(ship:groundspeed), round(getBearingFromAtoB()), ROUTINES["R31"], "R31").
    }
    if noun = 61 {
        registerDisplays(round(targethoverslam:lat,1), round(targethoverslam:lng,1), "", true, "").
    }
    if noun = 63 {
        registerDisplays(RAD_ALT(), round(ship:verticalspeed), round(ship:altitude), true, "").
    }
    if noun = 64 {
        registerDisplays("D64" + " " + LPD_DESIG(), round(ship:verticalspeed), RAD_ALT(), true, "").
    }
    if noun = 67 {
        registerDisplays(SLANT_RANGE(ship:geoposition:position:mag - targethoverslam:position:mag),round(ship:geoposition:lat,2), round(ship:geoposition:lng,2), true, "").
    }
    if noun = 68 {
        registerDisplays(LPD_DESIG(), round(ship:verticalspeed), "", ROUTINES["R38"], "R38").
    }
    if noun = 73 {
        registerDisplays(round(ship:altitude), round(ship:velocity:mag), round(pitch_for(ship, prograde),2), true, "").
    }
    if noun = 78 {
        local EG to RNDZ_ATT(tgtVessel).
        registerDisplays(EG[0], EG[1], EG[2], ROUTINES["R63"], "R63").
    }
    // N78 -> YAW ANGLE, PITCH ANGLE, AZIMUTH CONSTR | P20, R61, R63
    // N90 -> Y ACTIVE VEH, Ẏ ACTIVE VEH, Ẏ PASSIVE VEH | P20, R36
    if noun = 92 {
        registerDisplays(round(min(100, max(0, throttle*100))), round(ship:verticalspeed), round(trueRadar), true, "").
    }

    // Verb Section.

    if verb = 27 { // wishlist; make this VAC/RAM space. will have to define RAM
        registerDisplays(VAC_ACCUMULATION(), "", "", true, "").
    }
}

when terminal:input:haschar then { // checks input from the terminal
    getChar(terminal:input:getchar()).
    if verbChecker() and newVerb{
        set oldProgram to program.
        keyRelLogic(true).
        set inputArg to terminal_input_string(32,9).
        set program to inputArg.
        PARAM_CHECK().
        set newVerb to false.
    }
    keyRelLogic(false).
    preserve.
}

declare local function registerDisplays { // 3 register displays
    parameter R1, R2, R3, ROUT_VAL, ROUT_NAME.
   
    
    local ROUT_BOOL to ROUTINE_CANCEL(ROUT_VAL, ROUT_NAME).
    VN_FLASH(verb, noun, ROUT_BOOL, ROUT_NAME).

    local REGVALS to TO_OCTAL(R1, R2, R3, R1OC, R2OC, R3OC).
    
    //local regData to lexicon().
    //regData:add("R1", REGVALS[0]).
    //regData:add("R2", REGVALS[1]).
    //regData:add("R3", REGVALS[2]).
    //regData:add("ROUT_VAL", ROUT_BOOL).

    //writeJson(regData, "regdata.json").

    // need to fix monitor flag.
    // instead of monitor flag use V16 monitor pairup for a single update iter

    if monitorOnceFlag {
        print REGVALS[0]:padleft(7) at (33,11).
        print REGVALS[1]:padleft(7) at (33,13).
        print REGVALS[2]:padleft(7) at (33,15).
        set monitorOnceFlag to false.
    }

    if UPDBIT { // so this should ensure that values get displayed but not updated unless in ROUTINE
        print REGVALS[0]:padleft(7) at (33,11).
        print REGVALS[1]:padleft(7) at (33,13).
        print REGVALS[2]:padleft(7) at (33,15).
        set UPDBIT to false.
    }

    // tostring():padleft(5) looks like a really good alternative.
    // shift from ROUT_BOOL hack to actual calc

    if not CABIT{
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
}

declare local function ROUTINE_BOOL {

    for i in range(ROUTINES:length) {
        set ROUTINES[ROUTINES:keys[i]:tostring()] to false.
    }

    return true. // sets everything to false, then returns true for the routine to be set.
}

declare local function ROUTINE_CANCEL { // Cancel routines on V34 boolean. WIP -> Make it cancel the most latest routine.
    parameter ROUT_VAL, ROUT_NAME.
    if verb = 34 {
        set ROUTINES[ROUT_NAME] to false.
    }
    return ROUT_VAL.
}

declare local function verbChecker { // Verb 37 is used to change program modes. So you enter verb 37, and then your program. For that reason, we gotta make a check
//to confirm that if we ever want to change the program, it only changes on verb 37. we also wanna make sure that it will only change program once per each verb 37 change
    if verb = 37 {
        set verb to oldVerb.
        return true.
    }
    if verb = 33 {
        set verb to oldVerb.
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
    if verb = 25 {
        set PL_PERF_BIT to true.
    }
    if verb = 56 {
        set P20BIT to false.
        set program to 1.
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

    // NOUNS

    if noun = 7 {
        // R1 / R2, set val and flag
        PARAM_CHECK().
        set verb to oldVerb.
        set noun to oldNoun.
    }
}

// ---- Functionality ----

declare local function getChar { // Get character for V/N operation. Make an home grown string input program to avoid this interrupt..
    parameter OPER.
    
    if OPER = "+" {
        set oldNoun to noun.
        keyRelLogic(true).
        set inputArg to terminal_input_string(32,9).
        if inputArg:length > 0 {set noun to inputArg.}
        set UPDBIT to true.
    }
    if OPER = "-" {
        set oldVerb to verb.
        keyRelLogic(true).
        set inputArg to terminal_input_string(22,9).
        if inputArg:length > 0 {set verb to inputArg.}
        set newVerb to true.
    }
    if OPER = "*" {
        set verb to progVerb.
        set noun to progNoun.
    }
    if verb = 99 and OPER = "0" { // PRO KEY. Only active during V99
        set proceedFlag to true.
        set verb to oldVerb.
    }
}

declare local function ROUTINE_CALCS { // Enable calculations based on routine
    if program = 20 and ROUTINES["R36"] or ROUTINES["R22"] {
        RNDZ_STATE_CHECK().
        //return TRNF_ORB_DATA(tgtVessel).
    }
    if program = 32  {
        if P20BIT {return CSI_CALC(tgtVessel).}
        else{set PROGBIT to true. set program to 1.}
    }
    if program = 34 {
        if P20BIT {return TRNF_ORB_DATA(tgtVessel).}
        else {set PROGBIT to true. set OEBIT to true. return false.}
    } 
    else {
        return 0.
    }
}

declare local function ECADR_BIT { // edit BIT registry codes
    parameter key, value.

    if key = "2015" { //minkey routine
        set performMINKEY to ECADR_KEY(value).
    }
    if key = "2202" {
        set TPI_PRE_FLAG to ECADR_KEY(value).
    }
    if key = "2204" {
        set RNDZFlag to ECADR_KEY(value).
    }
    if key = "2024" {
        set TPI_POS_FLAG to ECADR_KEY(value).
    }
    if key = "2217" {
        set rollFlag to ECADR_KEY(value).
    }
    else {
        set OEBIT to true.
    }
}

declare local function ECADR_KEY { // return false, true or operator error
    parameter value.
    if value = 1 {
        return true.
    }
    if value = 0 {
        return false.
    }
    else {
        set OEBIT to true.
        //return false.
    }
}

declare local function TO_OCTAL { // Convert value from decimal to octal
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

declare local function VN_FLASH { // Flash verb and noun during routine
    parameter V, N, ROUT_VAL, ROUT_NAME.
    local bt to floor(mod(time:seconds, 1.5)) = 0.
    if V = 99 {
        if bt {print V:tostring:padright(2) at (22, 9). print N:tostring:padright(2) at (32, 9).}
            else {print "":tostring:padright(2) at (22, 9). print N:tostring:padright(2) at (32, 9).}
    }    
    else {
        print V:tostring:padright(2) at (22, 9). print N:tostring:padright(2) at (32, 9).
    }
}

declare local function PARAM_CHECK { // parameter input logic.
    set OEBIT to false. // so this should recycle on every cppc input
    if program = 12  {
        if enterDataFlag {
            VN_FLASH(verb, noun, true, "data").
            set tgtApo to (terminal_input_string(33, 15)):toscalar().
            set tgtPer to (terminal_input_string(33, 15)):toscalar().
            if tgtVessel <> "" {
                set tgtIncl to round(tgtVessel:orbit:inclination,1).
            }
            else {
                set OEBIT to true.
                set program to oldProgram.
            }
        }
        else {
            VN_FLASH(verb, noun, true, "data").
            if tgtVessel <> "" {
                set tgtApo to round(target:apoapsis). // and CDH would offset by -28km for apolune and perilune
                set tgtPer to round(target:periapsis).
                set tgtIncl to round(tgtVessel:orbit:inclination,1).
            }
            else {
                set OEBIT to true.
                set program to oldProgram.
            }
        }
    }
    if noun = 07 {
        set editKey to (terminal_input_string(33, 11)).
        set editValue to (terminal_input_string(33, 13)).
        if editKey:length <> 4 or editValue:length <> 1 {
            set OEBIT to true.
        }
        else {
            if PL_PERF_BIT {
                ECADR_BIT(editKey, editValue:toscalar()).
                set PL_PERF_BIT to false.
            }
            else {
                set OEBIT to true.
            }
        }
    }
    VN_FLASH(verb, noun, false, "data").
    set enterDataFlag to true.
}

declare local function REC_VN_CHECK { // Check if program recommended noun/verb is enabled
    if verb <> progVerb or noun <> progNoun {
        keyRelLogic("VN").
    }
}

declare local function ROD {
    parameter switchPos.

    if switchPos > 0 {
        set throttlePid:setpoint to throttlePid:setpoint + 2.
    }
    if switchPos < 0 {
        set throttlePid:setpoint to throttlePid:setpoint - 2.
    }
}

// ---- Event checks  ----

declare local function CLOCK_CALL { // convert time into HOURS / MIN / SEC
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

declare local function TIME_TO_IGNITION { // T_IG, Time to Ignition for any event
    local timeToIgn to 0.
    
    if program = 32 {
        local data to ROUTINE_CALCS().

        if data[4] {
            set timeToIgn to eta:periapsis.
        }
        if data[5] {
            set timeToIgn to eta:apoapsis.
        }
    }
    
    if program = 34 { 
        local data to ROUTINE_CALCS().        
        if RNDZFlag {
            set timeToIgn to data[5].    
            if data[5] < 1 {
                set timeToIgn to time:seconds + data[1]*60.
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

declare local function TIME_TO_EVENT { // Time to major event
    local ET to 0.

    if program = 34 { 
        local data to ROUTINE_CALCS().
        
        if RNDZFlag {
            set ET to data[5].    
            if data[5] < 1 or abs(data[0] - data[2]) <= 0.01{
                set ET to time:seconds + data[1]*60.
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
        set ET to abs(100 - ship:groundspeed/((ship:maxThrust/ship:mass) * cos(pitch_for(ship)))).
        if ET <= 3 {
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

    return list(0,0,0).
}

declare local function TIME_FROM_EVENT { // Time from last event
    local clock is CLOCK_CALL(time:seconds - lastEventTime:seconds).
    return list(clock[2], clock[1], clock[0]).
}

declare local function COMPUTER_CLOCK_TIME { // Time since computer bootup
    local clock is CLOCK_CALL(time:seconds - compBootTime:seconds).
    return list(clock[2], clock[1], clock[0]).
}

declare local function RNDZ_STATE_CHECK { // Check if in plane or not during P20
    set VAC_BANK["ORBT"] to true.
    set VAC_BANK["RNDZ"] to true.

    set RNDZFlag to TRNF_ORB_DATA(tgtVessel)[7] <= 0.2.
    set PLANEFlag to RNDZFlag. toggle PLANEFlag.
}

declare local function P_FLAG_CHECK { // Check and manipulate various program flags for functionality
    
    set IDLEBIT to program = 1.

    if program = 32 {
        local data to ROUTINE_CALCS().

        if not BURN_FLAG and data[4] {set BURN_FLAG to data[1] < 10.}
        if not BURN_FLAG and data[5] {set BURN_FLAG to data[0] < 10.}
        if not BURN_FLAG and proceedFlag and throttle = 0 {set proceedFlag to false.}
    }

    if program = 34 {
        local data to ROUTINE_CALCS().
        
        if not TPI_PRE_FLAG {
            if not VAC_BANK["THRT"] set VAC_BANK["THRT"] to true.
            if not VAC_BANK["ORBT"] set VAC_BANK["ORBT"] to true.
            if data[5] < 20{
                set TPI_PRE_FLAG to true.
            }
        }
        if TPI_PRE_FLAG and not BURN_FLAG {
            set BURN_FLAG to data[5] < 5.
        }
        // PLANE flag. Back to P36 then!
        if not BURN_FLAG and proceedFlag and throttle = 0 {set proceedFlag to false.}
    }

    if program = 63 {
        if not rollFlag and descentFlag {
            if not VAC_BANK["ATTI"]. set VAC_BANK["ATTI"] to true.
            set rollFlag to errorDistance < 10000.
        }
    }

    if program = 68 {
        if proceedFlag {
            set proceedFlag to not surfaceFlag.
        }
    }
}

// ---- Ship Health and housekeeping  ----

//declare local function ullageChecks {
//    local stab to getEngineStability().
//    set ship:control:fore to 1-stab.
//    return stab = 1.
//}

declare local function DATA_LIGHTS {
    IMU_GC().
    ATT_LIGHT().
    UPLK_ACTY().
    COMP_ACTY().
    OPR_ERR().
    progLightLogic().
    STBY().
    
}

declare local function IMU_GC {
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

declare local function ATT_LIGHT {
    if VAC_BANK["ATTI"] {
        print "      " at (2,8).
    }
    else {
        print "NO ATT" at (2,8).
    }
}

declare local function UPLK_ACTY {
    if homeConnection:isconnected {
        print "UPLINK" at (2,5).
        print "ACTY" at (2,6).  
    }
    else {
        print "      " at (2,5).
        print "    " at (2,6).         
    }
}

declare local function COMP_ACTY {

    if CMPACTY {
        print "COMP" at (22,5).
        print "ACTY" at (22,6).
    }
    else {
        print "    " at (22,5).
        print "    " at (22,6).       
    }
}

declare local function OPR_ERR {

    if OEBIT {
        print "OPR ERR" at (2,15).
        if mod(time:seconds, 2) <= 0.3 {
            set OEBIT to false. // this should ensure OP_ERR light does not stay on
        }
    }
    else {
        print "       " at (2,15).
    }
}

declare local function STBY {
    if IDLEBIT {
        print "STBY" at (2,11).
    }
    else {
        print "    " at (2,11).
    }
}

declare local function keyRelLogic { // KEY RELEASE logic. a) For inputting V/N b) For not program recommended V/N
    parameter io.

    if io and not CABIT{
        print "KEY REL" at (2,13).
    }
    if io = "VN" and not CABIT {
        print "KEY REL" at (2,13).
    }
    else {
        print "       " at (2,13).
    }
}

declare local function progLightLogic { // saving this for the legendary 1202.. Memory out error/program error
    
    if PROGBIT {
        if ERR = 1202 {
            print "1202":padleft(7) at (33,11).
            print "1202":padleft(7) at (33,13).
        }
        else {print "PROG" at (10,11).}
    }
    if not PROGBIT {
        print "    " at (10,11).
    }
}

declare local function restartLightLogic { // Enable restart light when demanded
    parameter io.
    if io {
        print "RESTART" at (10,13).
    }
    if  not io {
        print "       " at (10,13).
    }
}

declare local function VAC_ACCUMULATION { // Check how many cores are engaged in tasks
    local VAC_COUNT to 0.

    for i in range(VAC_BANK:length) {
        if VAC_BANK:values[i] {
            set VAC_COUNT to VAC_COUNT + 1.
        }
    }

    set RESTARTBIT to VAC_COUNT > 5.
    set PROGBIT to VAC_COUNT > 5.
    set ERR to 1202.
    return VAC_COUNT.
}

declare local function VAC_CLEAR { // Clear cores from task
    for i in range(VAC_BANK:length) {
        set VAC_BANK["" + VAC_BANK:keys[i] + ""] to false.
    }
}

declare local function RESTARTCHECK { // Check if restart BIT is enabled
    if RESTARTBIT {
        local dataOut to lexicon().
        dataOut:add("Noun", noun).
        dataOut:add("Verb", oldVerb).
        dataOut:add("Program", program).
        dataOut:add("RESTART", RESTARTBIT).
        dataOut:add("ERR", ERR).
        dataOut:add("PROG", PROGBIT).
        writeJson(dataOut, "compState.json").
        local throt to throttle.
        set ship:control:pilotmainthrottle to throt.    // this should hopefully not turn off the engines completely..
        //log output. V,N,P,BIT,Flag (json).
        reboot.
    }
}

declare local function READ_LAST_KEYS { // Transfer last computer state
    // this thing will set the last outputs from the json file.
    if exists("compState.json") {
        set dataIn to readJson("compState.json").
        set noun to dataIn["Noun"].
        set verb to dataIn["Verb"].
        set program to dataIn["Program"].
        set RESTARTBIT to dataIn["RESTART"].
        set ERR to dataIn["ERR"].
        set PROGBIT to dataIn["PROG"].
        restartLightLogic(RESTARTBIT).
        wait 0.25.
        progLightLogic().
        wait 0.25.
        set RESTARTBIT to false.
        set PROGBIT to false.
    }
}

// Start of computer software

set steeringManager:rollts to 1.875.
set steeringManager:pitchts to 1.875.
set steeringManager:yawts to 1.875.

READ_LAST_KEYS().
VNP_DATA().

until program = 00 {
    agcData().
    //VAC_CLEAR().

    if program = 1 {
        unlock steering.
        unlock throttle.
    }

    if program = 12 { // I was coincidentally lucky in naming this. P12 was a real program used to ascend from the Lunar surface
     if LOAN_DIFF(tgtVessel) < 0.5 {
            lock throttle to 2 * getTwr().
            lock steering to heading(launchAzimuth(tgtVessel)+yawReqPID:update(time:seconds, YAW_ASCENT_GUIDE()), 90-min(90, trueRadar/1000), 0).
            if tgtApo * 1000 <= ship:apoapsis {
                set program to 1.
            }
        }
    }

    if program = 20 { // orbital rendezvous
        set ROUTINES["R61"] to true.
        set ROUTINES["R63"] to true.
        set ROUTINES["R22"] to true.
        set P20BIT to true.
        if performMINKEY {
            set program to 32.
        }
    }
    
    if program = 32 { // CSI
        
        local orbitalData to ROUTINE_CALCS().
        local stab to getEngineStability().

        if not ROUTINES["R30"] {set ROUTINES["R30"] to true.}

        print orbitalData[2] at (2, 2).
        print orbitalData[3] at (2, 18).

        if orbitalData[4] {
            //print "apo " + orbitalData[2] at (2,2).
            if orbitalData[1] < 60 {set warp to 0.
            lock steering to orbitalData[2]/abs(orbitalData[2]) * ship:velocity:orbit.}
            if BURN_FLAG {
                set ship:control:fore to 1-stab.
                lock throttle to engineIgnitionPermission() * orbitalData[2].
                if orbitalData[2] <= 4 {
                    set BURN_FLAG to false.
                    lock throttle to 0.
                    unlock steering.
                }
            }
        }
        if orbitalData[5] {
            //print "per " + orbitalData[3] at (2,2).
            if orbitalData[0] < 60 {set warp to 0.
            lock steering to orbitalData[3]/abs(orbitalData[3]) * ship:velocity:orbit.}
            if BURN_FLAG {
                set ship:control:fore to 1-stab.
                lock throttle to engineIgnitionPermission() * orbitalData[3].
                if orbitalData[3] <= 4 {
                    set BURN_FLAG to false.
                    lock throttle to 0.
                    unlock steering.
                } 
            }
        }
        if abs(orbitalData[3]) < 4 and abs(orbitalData[2]) < 4 {
            if performMINKEY {
                set program to 34.
            }
        }
    }

    // residual to CDH..?

    if program = 34 { // TPI
        local orbitalData to ROUTINE_CALCS().
        local stab to getEngineStability().

        if orbitalData[0] < 0.02 and orbitalData[2] < 0.02 { // phase angle. change this to distance
            lock throttle to 0. unlock steering.
        }
        
        if RNDZFlag { // this should fall into P32-P35
            if orbitalData[5] <= 90 {
                set warp to 0.
            }
            if TPI_PRE_FLAG {    //time to target phase angle
                lock steering to orbitalData[3]/abs(orbitalData[3]) * ship:velocity:orbit.// * (orbitalData[3])/abs(orbitalData[3]). Inverse or parallel direction to prograde
                if BURN_FLAG {
                    local perm to engineIgnitionPermission().
                    set ship:control:fore to 1-stab.
                    lock throttle to perm * stab * orbitalData[3] * getTwr()/g0. // this way unless the stability is > 0, it won't fire
                    if abs(orbitalData[3]) < 0.5 {
                        lock throttle to 0.
                        set BURN_FLAG to false.
                        set TPI_PRE_FLAG to false.
                        set TPI_POS_FLAG to true.
                        // now this shifts to the next runmode, where V58 has use.
                    }
                }
            }
            if TPI_POS_FLAG and performMINKEY {
                set program to 35.
            }
        }
        if PLANEFlag {
            lock steering to vcrs(ship:velocity:orbit,-body:position).// * PLANE_CHANGE_DV(orbitalData[7])/abs(PLANE_CHANGE_DV(orbitalData[7])). For inverse/pll
            lock throttle to PLANE_CHANGE_DV(orbitalData[7]).
        }
    }

    if program = 35 { // Post TPI; TPF
        if TPI_POS_FLAG {
            if not AUTO_MAN_BIT {
                unlock steering.
            }
            if AUTO_MAN_BIT {
                lock steering to tgtVessel:direction * R(0, 270, 90).
            }
            // figure out course correction
            // and est. closest appr.
        }
    }
    
    // P32-P35. Setup a P20 BIT that lets it know you've got tracking data  
    // I wonder if I can setup a MCC state vector upload through archive + JSON reading

    if program = 63 { // velocity reduction
        set steeringManager:rollcontrolanglerange to 180.
        if progNoun <> 63 set progNoun to 63.
        if progVerb <> 16 set progVerb to 16.
        if not descentFlag {
            set progNoun to 44.
            if not ROUTINES["R30"] {set ROUTINES["R30"] to true.}
            if SLANT_RANGE(ship:geoposition:position:mag - targethoverslam:position:mag) < 550 {set warp to 0.}
            if SLANT_RANGE(ship:geoposition:position:mag - targethoverslam:position:mag) < 500 and not deorbitFlag {
                lock throttle to engineIgnitionPermission() * 1 * getTwr().
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
            if progNoun <> 54 set progNoun to 54.
            if not ROUTINES["R31"] {set ROUTINES["R31"] to true.}
            set descentFlag to true.
            //steeringCommand().
            if rollFlag {
                lock steering to srfRetrograde * -r(pitchReqPID:update(time:seconds, PITCH_LAND_GUIDE()), yawReqPID:update(time:seconds, YAW_LAND_GUIDE()), 162). // set our steering so that we get to target.
            }
            else {
                lock steering to srfRetrograde * -r(0, yawReqPID:update(time:seconds, -YAW_LAND_GUIDE()), 0).
            }
            lock throttle to max(0.6, max(throtVal, sqrt(errorDistance)/400)).
            if ship:groundspeed < 300 {
                set program to 64.
            }
        }
    }

    if program = 64 { // trajectory control
        if progNoun <> 68 set progNoun to 68.
        if not ROUTINES["R38"] {set ROUTINES["R38"] to true.}
        lock throttle to max(0.1, throtVal).
        set pitchReqPID:maxoutput to 50.
        set pitchReqPID:minoutput to -50.
        set pitchReqPID:setpoint to 0.
        // right, so 64 is initiated pitch up due to slow horizontal velocity and initial P63 offset.
        // so set the setpoints back to 0 and include the LPD angle and put tighter constraints on the retrograde AoA
        // add a LPD display & change(tie that into the WASD keys without SAS bool) function
        // possible LPD via vecDraw() from resultant velocity
        // and shift P64 to UP+(). then WASD controls the LZ

        if SAS {
            set program to 66. unlock steering.
        }
    }

    if program = 66 {
        ROD(ship:control:pilottranslation:z).
        lock throttle to max(0.1, throttlePid:update(time:seconds, ship:verticalspeed)). // PGM 66, or rate of descent, lets us descent at a very slow rate.
        if trueRadar < 0.2 or ship:status = "landed" {
            set program to 68. // program 68 is confirmation of touchdown.
        }
    }

    if program = 68{
        set surfaceFlag to true. // I think this was used for abort confirmation of not using the DM engine
        lock throttle to 0.
        unlock steering.
        SAS off.
        RCS off.
        set program to 1.
        set steeringManager:rollcontrolanglerange to 1.
        VAC_CLEAR().
    }

    if program = 70 { // DPS abort, recirc. Take from P12

    }

    if program = 71 { // APS abort, ascent+recirc. Take from P12

    }

    VAC_ACCUMULATION().
    wait 0.05.
}

deletePath("compState.json").