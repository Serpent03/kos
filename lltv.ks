// implement state vectors through lexicons (matrix) and remove API extraction dependency 
// work on getting the telescope mod for IMU alignments
// implement executive and waitlist logic. 0.020  + 0.009
// implement ECADR target option code.
// Star codes!
// Kalman filtering for IMU platform and PID
// switch PGNS throttle to acceleration instead of altitude ratios
// F_req = m.(PGNS_THROT); F_req/F_max = throttle ratio. Possibly more accurate. 

// P20 + PXX: https://history.nasa.gov/afj/loressay.html.

// Implemented
// P12 {not fully, GNDTRK, CIRBU}
// P20 {not fully, MARKS}
// P32 CSI - Coelleptic Sequence Initiate.
// P34 TPI - Terminal Phase Initiation
// P35 TPF - Transfer Phase {missing: MID_CRS_CORR}
// P63  {quartic trajectory}
// P64  {quartic trajectory}
// P66  {quartic trajectory}

// Not Implemented {TODO}
// P31 HAM - Height Adjustment Maneuver. 
// P33 CDH - Constant Delta H. Maintain constant 28km orbit less than CSM
// P36 PCM - Plane Change Maneuver
// P37 RTE - Return to Earth
// P40 SPS - Thrusting. Service Prop System
// P41 RCS - Thrusting.  
// P42 APS - Thrusting.
// P47 - Thrust Monitoring. RVEL, Range, Range Rate
// P76 LM TGT DV - LM TIG and change in orbital vel
// P79 Final RNDZ - Range, Range Rate, Angular Difference between X-axis(LEM, facing straight through passive port)

// Add an event timer for PGM 63 to 64 on pitchover.
// This should be achieved with T_GO. First result to
// perfect solution, and then start for next program.

// Iterate and improve the current implementation of the 
// PEG to support accurate ascent and descent
// for guidance and T_GO parameters

// clearscreen
set config:obeyhideui to false.
set core:bootfilename to "lltv.ksm".
set config:ipu to 850.
clearScreen.
clearVecDraws().

//run terminLIB.
runOncePath("terminLIB").
runOncePath("navballLIB").
runOncePath("peg").
SAS off. RCS on.
set terminal:charheight to 18.
set terminal:height to 20.
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
set vnCheckProg to program.
set oldERR to 0.
set ERR to 0.
set oldCache to 0.

set INSALT to altitude.
set deltaAlt to alt:radar - INSALT.

set tgtVessel to vessel("TCSM").

set lastEventTime to time.
set compBootTime to time.
set timeToIgn to time:seconds.
set ET to time:seconds.
set MESTIM to time:seconds.
set LPD_TERM to 0.
set LET to time:seconds - lastEventTime:seconds.

set monitorOnceFlag to false. //MOF | 1517
set BURN_FLAG to false. //BRN | 
set TPIPRFLG to false. //RBF | 2202
set TPIPOFLG to false. // PTP | 2024
set PLNFLG to false. //PLF | 2014
set RNDFLG to false. //RDF | 2204
set DESFLG to false. //DEF | 0405
set DHINTG to false. // I-loop for altitude updates.
set ROLFLG to false. //ROF | 2217
set SURFLG to false. //SRF | 
set TLTFLG to false.
set performMINKEY to false. //PMK | 2015
set enterDataFlag to true.
set progRecycleFlag to false. // maybe by switching cycles..
set proceedFlag to false. // PRO | V99
set KEYRELFLG to false.
set APSFLG to false.

set IDLEBIT to false.
set R1BIT to true.
set R2BIT to true.
set R3BIT to true.
set R1OC to false.
set R2OC to false.
set R3OC to false.
set BLANK_BIT to false.
set AUTOMANUBIT to true.
set PROGBIT to false.
set OEBIT to false.
set UPDROU to false.
set UPDBIT to false.
set VNBIT to false.
set KEYRUPT to false.
set P12BIT to false.
set P20BIT to false.
set CABIT to false.
set CMPACTY to false.
set PL_PERF_BIT to false.
set RESTARTBIT to false. // write to json and read pgm/flags/BITs as necessary

set ROUTINES to lexicon().
ROUTINES:add("R10", false). // routine 10 -> Ground track radar determination | P63, P64
ROUTINES:add("R22", false). // routine 22 -> Rendezvous tracking data processing | P20
ROUTINES:add("R30", false). // routine 30 -> Orbit parameter display | V82
ROUTINES:add("R31", false). // routine 31 -> Landing Trajectory Error Calculations | V83
ROUTINES:add("R36", false). // routine 36 -> Rendezvous out-of-plane display | V90
ROUTINES:add("R38", false). // routine 38 -> LPD Angles, completely made up routine | P64
ROUTINES:add("R61", false). // routine 61 -> Tracking attitude | P20, R52, AUTOMANUBIT
ROUTINES:add("R62", false). // routine 62 -> Start Crew Defined MNVR | V49
ROUTINES:add("R63", false). // routine 63 -> Rendezvous final attitude | R61, V89

// PROGRAM, VERB/NOUN
set REC_VN_KEYS to lexicon().
REC_VN_KEYS:add("1", "16/36").
REC_VN_KEYS:add("12", "16/33").
REC_VN_KEYS:add("32", "16/13").
REC_VN_KEYS:add("33", "16/11").
REC_VN_KEYS:add("63", "16/33").
REC_VN_KEYS:add("64", "16/64").
REC_VN_KEYS:add("66", "16/68").

// DIGIT SYNCHRONIZATION

set DIGSYN to "00000".

// TEST DYNDISP 
set INFO_KEYS to lexicon().
INFO_KEYS:add("63",list(round(deltaAlt), round(ship:verticalspeed), round(ship:altitude), true, "")).


set VAC_BANK to 0. // implement vec. accu. centers for exec and waitlist logic

// set up AVERAGE_G across the board.
// Ship orbital parameters

set tgtApo to 0.
set tgtPer to 0.
set tgtIncl to 0.

set ownApo to round(apoapsis/10).
set ownPer to round(periapsis/10).
set ownVy to round(verticalSpeed).
set ownVx to round(ship:groundspeed).
set slantRange to round(SLANT_RANGE(ship:geoposition:position, targethoverslam:position)).

list engines in eList.
set trueRadar to alt:radar. // revert back to KSP collision box system since we're not using Waterfall anymore
set g0 to constant:g * ship:body:mass/ship:body:radius^2.
set errorDistance to distanceMag.


//set LPD_VECDRAW to VECDRAW(v(0,0,0),v(0,0,0), red, "TGTLAND", 0.7, true, 0.3, true, false). 

//  ---- PID Loops ----

set throttlePid to pidLoop(0.2, 0.05, 0.01, 0, 1).
set throttlePid:setpoint to 0.

local yawReqPID to pidLoop(0.6, 0.15, 0.02, -30, 30).
set yawReqPID:setpoint to 0.

local pitchReqPID to pidLoop(0.03, 0.015, 0.02, -30, 0).
set pitchReqPID:setpoint to 166. //5km. 500 PIDout = 15km.

local DHINTG_LOOP to pidLoop(0.1, 0.05, 0).

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

declare local function changeRate {
    parameter oldTime, oldValue, stepTime.
    local newTime to time:seconds.
    local newCache to 0.

    set newCache to (oldCache - oldValue)/(newTime - oldTime). 
    set MESTIM to newTime.

    if newTime - oldTime > stepTime {
        set oldCache to newCache.
        return newCache.
    }
    else {
        return oldCache.
    }
}

declare local function updateState {
    set trueRadar to alt:radar. // revert back to KSP collision box system since we're not using Waterfall anymore
    set g0 to constant:g * ship:body:mass/ship:body:radius^2.
    set errorDistance to distanceMag.

    if  ROUTINES["R30"] {
        set ownApo to round(apoapsis/10).
        set ownPer to round(periapsis/10).
    }

    set ownVy to round(verticalSpeed).
    set ownVx to round(ship:groundspeed).
    set slantRange to round(SLANT_RANGE(ship:geoposition:position, targethoverslam:position)).

    // set up other stuff to be updated here 
    // alt, e, slant range, .. etc based on ROUTINE calls
}

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

declare local function SLANT_RANGE { // [km] Currently ground track range

    parameter CPOS, TPOS.

    local GCA to vAng(CPOS, TPOS). // great circle angle
    local GCR to 2 * constant:pi * ship:body:radius. // great circle radius

    return round(abs(GCR * GCA/360)/1000, 2).
}

// Ascent guidance section

// Since I don't want to bloat this file(too much), refer to "peg.ks" for lunar ascent GNC

// Landing guidance section

declare local function PITCH_LAND_GUIDE {	// TDAG EW - trajectory discrepancy avoidance guidance East <-> West.


    if addons:tr:hasimpact {
        return -(addons:tr:impactpos:lng - targetHoverslam:lng)*1000 *-1.
    }
    else {
        return 0.
    }
}


declare local function YAW_LAND_GUIDE {	// TDAG NS - trajectory discrepancy avoidance guidance for North <-> South.


    if addons:tr:hasimpact {
	    return (addons:tr:impactpos:lat - targetHoverslam:lat)*1000 *-1.
    }
    else {
        return 0.
    }
}

declare local function LPD_DESIG { // Output LPD ladder trajectory impact

    if addons:tr:hasimpact and ROUTINES["R38"]{

        local vec1 to 90 - vang(ship:body:position, targetHoverslam:position).
        if abs(addons:tr:impactpos:bearing) > 90 { // depending on the direction of craft, this will edit the displayed LPD angle
            set vec2 to 1.
        }
        else {
            set vec2 to -1.
        }
        //if abs(addons:tr:impactpos:bearing) > 90 and abs(addons:tr:impactpos:bearing) < 180 s
        local vec3 to vec2 * (90-pitch_for(ship)).//90 - pitch_for(ship).
        local vec4 to min(60, max(0, ((vec1+vec3) * 2.30654761904) - 7)).

        set vec4 to abs(vec4):tostring().
        if vec4:length < 2 {
            return "0" + vec4.
        }
        else {return vec4.}
    }
    else {
        return "00".
    }
}

declare local function RAD_ALT { // Radar altitude

    // have an integration function to incorporate Delta-H with time
    // I-loop? and then return Delta-H as the I-loop accumulation.
    // also shift surface altitude from trueRadar to this computer altitude.
    // only during descent though. don't care about else

    if ROLFLG and ROUTINES["R10"] {
        set deltaAlt to alt:radar - INSALT.

        if DHINTG { // start integrating it. P-I loop maybe
            set INSALT to INSALT - DHINTG_LOOP:update(time:seconds, deltaAlt).
            //print INSALT at (2,0).
        }

    }
    else {
        set INSALT to altitude.
    }
}

declare local function LAND_THROT { // Two dimensional product.

    local tᵧVal to 0. local tₓVal to 0.

    local distₓ to slantRange.
    local sAcc to (ship:availablethrust/ship:mass) - g0.
   
    if program = 63 {
        // this should give throttle updates corresponding to 
        // P63 targets -> 5km uprange, -500m below the surface
        local dyHeight to (ownVy^2 / (2 * sAcc)).
        set tᵧVal to dyHeight/(trueRadar)*3.
        
        local dxHeight to ownVx^2 / (2 * (ship:availablethrust/ship:mass)).
        set tₓVal to dxHeight/(distₓ-5.6).

        local throtProduct to sqrt((tᵧVal)^2 + (tₓVal)^2)/1000.

        //if throtProduct > 0.6 {
        //    return 1.
        //}
        //else {
        //    return throtProduct.
        //}
        return throtProduct.
    }
    if program = 64 {
        // throttle updates for the P64 targets. 30m above landing area
        // to null vertical rate and bring it to a hover. when within
        // 500 metres of the target site this will only return throttle
        // one-dimensionally for vertical rates, so as to not eventually
        // divide the tₓVal by 0, as distₓ approaches 0.
    
        local dyHeight to (ownVy^2 / (2 * sAcc)).
        set tᵧVal to dyHeight/(trueRadar-150)*3.

        local dxHeight to ownVx^2 / (2 * (ship:availablethrust/ship:mass)).
        set tₓVal to dxHeight/max(1,(distₓ)).

        local throtProduct to sqrt((tᵧVal)^2 + (tₓVal)^2)/1000.

        if distₓ > 0.5 {
            //if throtProduct > 0.6 {
            //    return 1.
            //}
            //else {
            //    return throtProduct.
            //}
            return throtProduct.
        }
        else {return tᵧVal*2.}
    }

    // in theory this should be sqrt((tₓVal)^2 + (tᵧVal)^2) = throtVal
}

// Orbital Pairup Section

declare local function CSI_CALC {
    //[0]: ETA to Apogee/Lune
    //[1]: ETA to Perigee/Lune
    //[2]: DV for Apogee/Lune Raise
    //[3]: DV for Perigee/Lune Raise
    //[4]: Apogee/Lune Raise flag
    //[5]: Perigee/Lune Raise flag

    // this section needs rework. Instead of working with altitudes,
    // I should work with the actual orbital velocity at that point using
    // createOrbit() or somesuch and then query at the velocityAt() with
    // perigee/lune and apogee/lune.

    parameter RNDZ_TGT.

    local etaToApo to eta:apoapsis.
    local etaToPer to eta:periapsis.
    
    local apoRaiseDV to abs((RNDZ_TGT:orbit:apoapsis - ship:orbit:apoapsis)/1000)-28.
    local perRaiseDV to abs((RNDZ_TGT:orbit:periapsis - ship:orbit:periapsis)/1000)-28.

    local apoFlag to abs(apoRaiseDV) >= 4.
    local perFlag to abs(perRaiseDV) >= 4 and not apoFlag.

    return list(etaToApo, etaToPer, apoRaiseDV, perRaiseDV, apoFlag, perFlag).

}

declare local function RNDZ_STATE_CHECK { // Check if in plane or not during P20

    // set flag for P36(Plane Change Maneuver). After P32. 

    set RNDFLG to TRNF_ORB_DATA(tgtVessel)[7] <= 0.3.
    set PLNFLG to RNDFLG. toggle PLNFLG.
}

declare local function PLANE_CHANGE_DV { // dv needed to change inclinations
    parameter datum.
    
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
    

    return list(floor(RNDZ_TGT:direction:yaw), floor(RNDZ_TGT:direction:pitch), floor(RNDZ_TGT:direction:roll)).
}


// Launch Section

declare local function launchAzimuth { // factoring in the target latitude, ship latitude, earth's rotating and orbital velocity to get needed launch azimuth

    
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

    return round(abs(ship:orbit:longitudeofascendingnode - RNDZ_TGT:orbit:longitudeofascendingnode),3).
}


//  ---- Computer Interrupts ----

when terminal:input:haschar then { // checks input from the terminal
    set KEYRUPT to true.
    getChar(terminal:input:getchar()).
    if verbChecker() and newVerb{
        VN_SYNC(37, noun, false).
        set oldProgram to program.
        keyRelLogic(true).
        local inputArg to terminal_input_string(32,9).
        if inputArg:length > 0 {set program to inputArg.}
        PARAM_CHECK().
        set newVerb to false.
    }
    keyRelLogic(false).
    set KEYRUPT to false.
    preserve.
}

when ag10 and not APSFLG  then { // staging logic for APS abort/ign
    list engines in eList.
    stage.
    stage.
    preserve.
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
    print "┃" at (.5, 12).                      print "┗━━━━━━━━━━━━━━━━━┛" at (.5,16).                     print "┃" at (17.5, 12).  print "┃" at (20.5, 12).                                                                                        print "┃" at (39.5, 12).
    print "┃" at (.5, 13).                                                                                  print "┃" at (17.5, 13).  print "┃" at (20.5, 13).  print "──────────────────" at (22,14).                                                print "┃" at (39.5, 13).
    print "┃" at (.5, 14).                                                                                  print "┃" at (17.5, 14).  print "┃" at (20.5, 14).                                                                                        print "┃" at (39.5, 14).
    print "┃" at (.5, 15).                                                                                  print "┃" at (17.5, 15).  print "┃" at (20.5, 15).                          print "┗━━━━━━━━━━━━━━━━━━━┛" at (20.5,16).                   print "┃" at (39.5, 15).

}


declare local function agcData { // this thing was a fucking PAIN to write, LOL
    set VAC_BANK to VAC_BANK + 1.
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
    set UPDROU to round(mod(dt,2)) = 2. 
    set CABIT to floor(mod(dt, 0.54)*10) = 5.
    set CMPACTY to round(mod(dt, 1.4)) = 0 and not IDLEBIT.
    set CMPACTY to CMPACTY or KEYRUPT.
    set VNBIT to floor(mod(dt, 1.4)) = 0.
    
    RESTARTCHECK().
    P_FLAG_CHECK().
    REC_VN_CHECK().
    DATA_LIGHTS().
    VNP_DATA().
    // shift VNP_DATA to lex().
    // if hasKey {VNP_DATA[noun:tostring()]} -> SELNOU
    // registerDisplays(SELNOU[0], SELNOU[1], SELNOU[2], SELNOU[3], SELNOU[4])

}

declare local function VNP_DATA {
    // Program Section
    // I think the actual thing used a VN pair to show this information for various PGMs. Having the actual PGM as constraint is restrictive..
    // actually I think this may be a ROUTINE tied to the program itself! Extended VERB 82 for example which should trigger R30 as used by P20

    if program = 12 and noun = 93{ //most likely incorrect noun code
        registerDisplays(tgtApo, tgtPer, tgtIncl, true, "").
    }
    if program = 12 and noun = 94{
        registerDisplays(tgtApo, tgtPer, round(LOAN_DIFF(tgtVessel),2), true, "").
    }
    // Noun Section

    if noun = 09 {
        registerDisplays(oldERR, ERR, "", true, "").
    }
    if noun = 32 {
        local ETAP to CLOCK_CALL(eta:periapsis).
        registerDisplays(ETAP[2], ETAP[1], ETAP[0], true, "").
        // make this create paste and delete the same file so first iter gets newest data
    }
    if noun = 33 {
        local DT to CLOCK_CALL(timeToIgn).
        registerDisplays(DT[2], DT[1], DT[0], true, "").
    }
    if noun = 34 {
        // call MET + time for P64 pitchover and other
        local DT to CLOCK_CALL(ET).
        registerDisplays(DT[2], DT[1], DT[0], true, "").
    }
    if noun = 35 {
        local DT to CLOCK_CALL(LET).
        registerDisplays(DT[2], DT[1], DT[0], true, ""). // see if I can get TIME:HOUR/MIN/SEC conversion on each register
    }
    if noun = 36 {
        local DT to COMPUTER_CLOCK_TIME().
        registerDisplays(DT[0], DT[1], DT[2], true, ""). 
    }
    if noun = 42 {
        registerDisplays(ownApo, ownPer, round(stage:deltaV:current), true, "").
    }
    if noun = 43 {
        registerDisplays(round(ship:geoposition:lat,2)*100, round(ship:geoposition:lng,2)*100, round(ship:altitude), true, "").
    }
    if noun = 44 {
        registerDisplays(ownApo, ownPer, round(sqrt(ship:orbit:semimajoraxis^3 * constant:pi^2 * 4 / body:mu)/2), ROUTINES["R30"], "R30").
    }
    if noun = 47 {
        registerDisplays(round(ship:mass)/1000, round(tgtVessel:mass)/1000, "", true, "").
    }
    if noun = 54 {
        registerDisplays(round(errorDistance), round(ownVx), round(getBearingFromAtoB()), ROUTINES["R31"], "R31").
    }
    // N55 -> YAW ANGLE, PITCH ANGLE, AZIMUTH CONSTR(Xε) | P20, R61, R63
    if noun = 61 {
        registerDisplays(round(targethoverslam:lat,1), round(targethoverslam:lng,1), "", true, "").
    }
    if noun = 63 {
        registerDisplays(round(deltaAlt), round(ownVy), round(ship:altitude), true, "").
    }
    if noun = 64 {
        // D64 is T_GO..
        registerDisplays(round(LPD_TERM) + "─" + LPD_DESIG(), round(ownVy), INSALT, ROUTINES["R10"], "R10").
    }
    if noun = 67 {
        registerDisplays(slantRange*100,round(ship:geoposition:lat,2)*100, round(ship:geoposition:lng,2)*100, true, "").
    }
    if noun = 68 {
        registerDisplays(round(ownVx), round(ownVy), round(deltaAlt), ROUTINES["R10"], "R10").
    }
    if noun = 73 {
        registerDisplays(round(ship:altitude), round(ship:velocity:surface:mag), round(pitch_for(ship, prograde),2), true, "").
    }
    if noun = 77 {
        registerDisplays("TMNSC", round(tgtVessel:velocity:orbit:y-ship:velocity:orbit:y), "", true, "").
    }
    if noun = 78 {
        local EG to RNDZ_ATT(tgtVessel).
        registerDisplays(EG[0], EG[1], EG[2], ROUTINES["R63"], "R63").
    }
    // N90 -> Y ACTIVE VEH, Ẏ ACTIVE VEH, Ẏ PASSIVE VEH | P20, R36
    if noun = 92 {  
        registerDisplays(round(LAND_THROT()*100), ownVy, round(trueRadar), true, "").
    }

    // Verb Section.

    if verb = 27 { // wishlist; make this VAC/RAM space. will have to define RAM
        registerDisplays(VAC_BANK, "", "", true, "").
    }
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
    if verb = 57 {
        set DHINTG to true.
        set verb to oldVerb.
    }
    if verb = 58 {
        toggle AUTOMANUBIT.
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

    if noun = 07 {
        // R1 / R2, set val and flag
        PARAM_CHECK().
        set verb to oldVerb.
        set noun to oldNoun.
    }
}

declare local function registerDisplays { // 3 register displays
    parameter R1, R2, R3, ROUT_VAL, ROUT_NAME.
   
    
    local ROUT_BOOL to ROUTINE_CANCEL(ROUT_VAL, ROUT_NAME).
    VN_FLASH().

    local REGVALS to TO_OCTAL(R1, R2, R3, R1OC, R2OC, R3OC).
    set REGVALS to DIGIT_SYNC(REGVALS[0], REGVALS[1], REGVALS[2], R1OC, R2OC, R3OC).

    //local regData to lexicon().
    //regData:add("R1", REGVALS[0]).
    //regData:add("R2", REGVALS[1]).
    //regData:add("R3", REGVALS[2]).
    //regData:add("ROUT_VAL", ROUT_BOOL).

    //writeJson(regData, "regdata.json").

    // need to fix monitor flag.
    // instead of monitor flag use V16 monitor pairup for a single update iter

    if CABIT and not BLANK_BIT {

        if monitorOnceFlag {
            print REGVALS[0]:padleft(7) at (32,11).
            print REGVALS[1]:padleft(7) at (32,13).
            print REGVALS[2]:padleft(7) at (32,15).
            set monitorOnceFlag to false.
        }

        if UPDBIT { // so this should ensure that values get displayed but not updated unless in ROUTINE
            print REGVALS[0]:padleft(7) at (32,11).
            print REGVALS[1]:padleft(7) at (32,13).
            print REGVALS[2]:padleft(7) at (32,15).
            set UPDBIT to false.
        }

        // tostring():padleft(5) looks like a really good alternative.
        // shift from ROUT_BOOL hack to actual calc

        if R1BIT and ROUT_BOOL {
            print REGVALS[0]:padleft(7) at (32,11).
        }
        if R2BIT and ROUT_BOOL {
            print REGVALS[1]:padleft(7) at (32,13).
        }
        if R3BIT and ROUT_BOOL {
            print REGVALS[2]:padleft(7) at (32,15).
        }
    }

    if CABIT and BLANK_BIT {
        // should blank out the registers on PRO demand or PGM
        print "      ":padleft(7) at (32,11).
        print "      ":padleft(7) at (32,13).
        print "      ":padleft(7) at (32,15).
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

// ---- Functionality ----

declare local function getChar { 
    // Get character for V/N operation. Make an home grown string input program to avoid this interrupt..
    // okay putting VN_FLASH() over in the terminLIB.ks should fix this issue regardless.
    parameter OPER.
    
    keyRelLogic(OPER = "+" or OPER = "-").
    VN_SYNC(verb, noun, OPER = "-" and OPER <> "+").
    if OPER = "+" {
        local inputArg to terminal_input_string(32,9).
        set oldNoun to noun.
        if inputArg:length > 0 {set noun to inputArg.}
        set UPDBIT to true.
    }
    if OPER = "-" {
        set oldVerb to verb.
        local inputArg to terminal_input_string(22,9).
        if inputArg:length > 0 {set verb to inputArg.}
        set newVerb to true.
    }
    if OPER = "*" {
        set KEYRELFLG to true.
    }
    if verb = 99 and OPER = "0" { // PRO KEY. Only active during V99 or when needed.
        set proceedFlag to true.
        set verb to oldVerb.
    }
    
}

declare local function ROUTINE_UPDATE {
    parameter newCalc, dT.
    // just assign a temporary old val to
    // old cache. send updates in gaps of
    // +≈ 02 seconds. 
    if dT {
        set oldCache to newCalc.
        return newCalc.
    }
    else {
        return oldCache.
    }
}

declare local function ROUTINE_CALCS { // Enable calculations based on routine
    set VAC_BANK to VAC_BANK + 1.
    if program = 12 {
        return PEG_LOOP().
    }
    if program = 20 and ROUTINES["R36"] or ROUTINES["R22"] {
        
        RNDZ_STATE_CHECK().
        // determine P32 to P36 or continue sequence
        //return TRNF_ORB_DATA(tgtVessel).
    }
    if program = 32 {
        if P20BIT {local cache to CSI_CALC(tgtVessel). return cache.}
        else{set PROGBIT to true. set program to oldProgram.}
    }
    if program = 34 or 36 {
        if P20BIT {local cache to TRNF_ORB_DATA(tgtVessel). return cache.}
        else {set PROGBIT to true. set OEBIT to true.}
    }
    if program = 63 or program = 64 or program = 66 {
        RAD_ALT().
        set VAC_BANK to VAC_BANK + 1.
        return LAND_THROT(). // so by count, 7 "tasks" during P63. busy core.
    }
    else {
        return.
        // find a better way instead of just subbing lists
        // for routine calc returns.
    }
}

declare local function DIGIT_SYNC {
    parameter R1, R2, R3, OCR1, OCR2, OCR3.

    set R1 to R1:tostring().
    set R2 to R2:tostring().
    set R3 to R3:tostring().
    
    set R1S to "+".
    set R2S to "+".
    set R3S to "+".

    if R1:find("-") = 0 {
        set R1S to "-".
        set R1 to R1:remove(0, 1).
    }
    if R2:find("-") = 0 {
        set R2S to "-".
        set R2 to R2:remove(0, 1).
    }
    if R3:find("-") = 0 {
        set R3S to "-".
        set R3 to R3:remove(0, 1).
    }

    local STRADD1 to "".
    local STRADD2 to "".
    local STRADD3 to "".

    if R1:length <= 5 {
        set STRADD1 to DIGSYN:remove(0, R1:length).
        set STRADD1 to STRADD1 + R1.
        if not OCR1 {
            set STRADD1 to R1S + STRADD1.
        }
    }
    if R2:length <= 5 {
        set STRADD2 to DIGSYN:remove(0, R2:length).
        set STRADD2 to STRADD2 + R2.
        if not OCR2 {
            set STRADD2 to R2S + STRADD2.
        }
    }
    if R3:length <= 5 {
        set STRADD3 to DIGSYN:remove(0, R3:length).
        set STRADD3 to STRADD3 + R3.
        if not OCR3 {
            set STRADD3 to R3S + STRADD3.
        }
    }
    
    return list(STRADD1, STRADD2, STRADD3).
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

declare function VN_FLASH { // Flash verb and noun during routine
    // I believe flashing V/N pair also indicate more data
    // or to be entering PRO or +/- RX display data
    if verb = 99 {
        if VNBIT {print verb:tostring:padright(2) at (22, 9). print "":tostring:padright(2) at (32, 9).}
            else {print "":tostring:padright(2) at (22, 9). print "":tostring:padright(2) at (32, 9).}
    }    
    if KEYRUPT {
        if VNBIT {print verb:tostring:padright(2) at (22, 9). print noun:tostring:padright(2) at (32, 9).}
            else {print "":tostring:padright(2) at (22, 9). print "":tostring:padright(2) at (32, 9).}
    }
    else {
        print verb:tostring:padright(2) at (22, 9). print noun:tostring:padright(2) at (32, 9).
    }
}


declare local function PARAM_CHECK { // parameter input logic.
    set OEBIT to false. // so this should recycle on every input
    if program = 12 and not P12BIT {
        set lastEventTime to time.
        if enterDataFlag {
            set tgtApo to (terminal_input_string(33, 15)):toscalar().
            set tgtPer to (terminal_input_string(33, 15)):toscalar().
            if tgtVessel <> "" {
                set tgtIncl to round(tgtVessel:orbit:inclination,1).
            }
            else {
                set OEBIT to true.
                set program to oldProgram.
            }
        INIT_VAL(tgtApo, tgtPer, tgtIncl, 0).
        }
        else {
            if tgtVessel <> "" {
                set tgtApo to 85. // and CDH would offset by -28km for apolune and perilune
                set tgtPer to 16.
                set tgtIncl to round(tgtVessel:orbit:inclination,1).
                INIT_VAL(tgtApo, tgtPer, tgtIncl, 0).
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
    set enterDataFlag to true.
}

declare local function ECADR_BIT { // edit BIT registry codes
    parameter key, value.

    if key = "2015" { //minkey routine
        set performMINKEY to ECADR_KEY(value).
    }
    if key = "2202" {
        set TPIPRFLG to ECADR_KEY(value).
    }
    if key = "2204" {
        set RNDFLG to ECADR_KEY(value).
    }
    if key = "2024" {
        set TPIPOFLG to ECADR_KEY(value).
    }
    if key = "2217" {
        set ROLFLG to ECADR_KEY(value).
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

declare local function REC_VN_CHECK { // Check if program recommended noun/verb is enabled

    local RUPTBOOL to REC_VN_KEYS:haskey(program:tostring()).

    if RUPTBOOL {
        local RECV to REC_VN_KEYS[program:tostring()][0] + REC_VN_KEYS[program:tostring()][1].
        local RECN to REC_VN_KEYS[program:tostring()][3] + REC_VN_KEYS[program:tostring()][4].
        
        if verb <> RECV or noun <> RECN {
            keyRelLogic("VN").
        }
        if vnCheckProg <> program or KEYRELFLG {
            set vnCheckProg to program.
            set KEYRELFLG to false.
            set verb to RECV.
            set noun to RECN.
        }
    }
}

declare local function VN_FLAG_OPERATOR {
    if program = 12 {
        if not proceedFlag {
            set REC_VN_KEYS["12"] to "16/33".
        }
        if proceedFlag {
            if not TLTFLG {
                set REC_VN_KEYS["12"] to "16/68".
            }
            if TLTFLG {
                set REC_VN_KEYS["12"] to "16/44".
            }
        }
    }
    if program = 63 {
        if not DESFLG {
            set REC_VN_KEYS["63"] to "16/33".
        }
        if DESFLG {
            set REC_VN_KEYS["63"] to "16/67".
            if ROLFLG {
                set REC_VN_KEYS["63"] to "16/63".
            }
        }
    }
    if program = 64 {
        // not sure what to put here.
        // assumably at LPD_TERM end we switch to 
        // 16/68 ..?
    }
}

declare local function ROD {
    parameter switchPos.

    if switchPos > 0 {
        set throttlePid:setpoint to throttlePid:setpoint + 1.
    }
    if switchPos < 0 {
        set throttlePid:setpoint to throttlePid:setpoint - 1.
    }
}

declare local function LPD_UPDATE {
    parameter x, y.
    local tempPos to 0.
    // figure out distance from half a degree
    if x > 0 {
        //+starvector
        set tempPos to tgtLand:position + (ship:facing:starvector*175).
        set tgtLand to ship:body:geopositionof(tempPos).
        set targethoverslam to (tgtLand).
    }
    if x < 0 {
        //-starvector
        set tempPos to tgtLand:position + (-ship:facing:starvector*175).
        set tgtLand to ship:body:geopositionof(tempPos).
        set targethoverslam to (tgtLand).
    }
    if y > 0 {
        //-topvector
        set tempPos to tgtLand:position + (ship:facing:topvector*175).
        set tgtLand to ship:body:geopositionof(tempPos).
        set targethoverslam to (tgtLand).
    }
    if y < 0 {
        //+topvector
        set tempPos to tgtLand:position + (-ship:facing:topvector*175).
        set tgtLand to ship:body:geopositionof(tempPos).
        set targethoverslam to (tgtLand).
    }
}

// ---- Event checks  ----

declare local function CLOCK_CALL { 
    // convert time into HOURS / MIN / SEC
    parameter second.

    local hour to 0.
    local min to 0.
    local oldMin to 0.
    local sec to 0.

    set sec to floor(second).
    // can probably mod(a,b) this
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
    set timeToIgn to 0.
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
        if RNDFLG {
            set timeToIgn to data[5].    
            if data[5] < 1 {
                set timeToIgn to time:seconds + data[1]*60.
                set lastEventTime to time:seconds.
            }
        }
    }

    if program = 36 {
        local data to ROUTINE_CALCS().
        if PLNFLG {
            set timeToIgn to "ETA A/N or D/N".
            if "ETA A/N or D/N" < 1 {
                set timeToIgn to 0.
                set lastEventTime to time:seconds.
            }
        }
    }

    if program = 63 {
        local rng to slantRange-500.
        if rng > 0 {
            set timeToIgn to rng/ownVx*1000.
            set BLANK_BIT to  timeToIgn < 35 and timeToIgn > 30.
        }
        if not proceedFlag {
            set lastEventTime to time.
        }
    }
}

declare local function TIME_TO_EVENT { // Time to major event
    set ET to 0.
    if program = 34 { 
        local data to ROUTINE_CALCS().
        
        if RNDFLG {
            set ET to data[5].    
            if data[5] < 1 or abs(data[0] - data[2]) <= 0.01{
                set ET to time:seconds + data[1]*60.
                set lastEventTime to time.
            }
        }
        if PLNFLG {
            set ET to "ETA A/N or D/N".
            if "ETA A/N or D/N" < 1 {
                set ET to 0.
                set lastEventTime to time.
            }
        }
    }

    if program = 63 { // time to p64 pitch over.
        set ET to max(0, (ship:velocity:surface:mag - 280)/(ship:availableThrust*cos(pitch_for(ship))/ship:mass)).
        if ET <= 1 {set program to 64.}
    }

    if program = 64 { // time to end P64 designation.
        set ET to max(0, (ship:velocity:surface:mag - 200)/(ship:availableThrust*cos(pitch_for(ship))/ship:mass)).
    }

    if program = 01 {
        local TT to time.
        return list(TT:hour, TT:minute, TT:second).
    }
}

declare local function TIME_FROM_EVENT { // Time from last event
    set LET to time:seconds - lastEventTime:seconds.
}

declare local function COMPUTER_CLOCK_TIME { // Time since computer bootup
    local clock is CLOCK_CALL(time:seconds - compBootTime:seconds).
    return list(clock[2], clock[1], clock[0]).
}

declare local function P_FLAG_CHECK { // Check and manipulate various program flags for functionality
    
    TIME_TO_IGNITION().
    TIME_TO_EVENT().
    TIME_FROM_EVENT().
    VN_FLAG_OPERATOR().

    set APSFLG to eList:length = 1.
    set IDLEBIT to program = 1.
    set P12BIT to program = 12.

    if program = 6 {
        set BLANK_BIT to true or BLANK_BIT.
    }
    if not program = 6 {
        set BLANK_BIT to false or BLANK_BIT.
    }

    if program = 12 {
        if not TLTFLG {set TLTFLG to ownVy > 12.}
        set proceedFlag to throttle > 0.
    }

    if program = 32 {
        local data to ROUTINE_CALCS().

        if not BURN_FLAG and data[4] {set BURN_FLAG to data[1] < 10.}
        if not BURN_FLAG and data[5] {set BURN_FLAG to data[0] < 10.}
        if not BURN_FLAG and proceedFlag and throttle = 0 {set proceedFlag to false.}
    }

    if program = 34 {
        local data to ROUTINE_CALCS().
        
        if not TPIPRFLG {

            if data[5] < 20{
                set TPIPRFLG to true.
            }
        }
        if TPIPRFLG and not BURN_FLAG {
            set BURN_FLAG to data[5] < 5.
        }
        // PLANE flag. Back to P36 then!
        if not BURN_FLAG and proceedFlag and throttle = 0 {set proceedFlag to false.}
    }

    if program = 63 {
        if not DHINTG_LOOP:kp = 0.1 {set DHINTG_LOOP:kp to 0.1.}
        if not ROLFLG and DESFLG {
            set ROLFLG to errorDistance < 20000.
        }
        else {
            set ROUTINES["R10"] to ROLFLG.
        }
    }

    if program = 64 {
        if not DHINTG_LOOP:kp = 0.35 {set DHINTG_LOOP:kp to 0.35.}
    }

    if program = 65 or program = 66 or program = 67 {
        if not DHINTG_LOOP:kp = 0.55 {set DHINTG_LOOP:kp to 0.55.}
    }

    if program = 68 {
        if proceedFlag {
            set proceedFlag to not SURFLG.
        }
    }

    if program = 70 {
        if APSFLG {set program to 71.}
    }

    if program = 63 or program = 64 or program = 66 or program = 68{
        if not APSFLG and abort {
            set program to 70. set abort to false.
        }
        if APSFLG and abort {
            set program to 71. set abort to false.
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
    restartLightLogic(RESTARTBIT).
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
    if not IDLEBIT { // add func through custom gimbal logic
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
        if round(mod(time:seconds, 2)) = 2 {
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

declare function keyRelLogic { // KEY RELEASE logic. a) For inputting V/N b) For not program recommended V/N
    parameter io.

    if io {
        print "KEY REL" at (2,13).
    }
    else if io = "VN" and CABIT {
        print "KEY REL" at (2,13).
    }
    else {
        print "       " at (2,13).
    }
}

declare local function progLightLogic { // saving this for the legendary 1202.. Memory out error/program error
    
    if PROGBIT {
        print "PROG" at (10,11).
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

    print "VAC " + VAC_BANK at (2,20).

    set RESTARTBIT to VAC_BANK >= 7.
    set PROGBIT to VAC_BANK >= 7.
    if RESTARTBIT and PROGBIT {set ERR to 642.}

    if not RESTARTBIT {set VAC_BANK to 0.}

    // actually right here a mod(dt,2) could be
    // made which would call onto 31201 for
    // executive job overflow

}

declare local function VAC_CLEAR { // Clear cores from task
    set VAC_BANK to 0.
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
        set ship:control:pilotmainthrottle to throttle.    // this should hopefully not turn off the engines completely..
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
        set oldERR to ERR.
        restartLightLogic(RESTARTBIT).
        wait 0.25.
        progLightLogic().
        wait 0.25.
        set RESTARTBIT to false.
        set PROGBIT to false.
        deletePath("compState.json").
    }
}

// Bootup/Restart init

set steeringManager:rollts to 1.375.
set steeringManager:pitchts to 1.375.
set steeringManager:yawts to 1.375.

READ_LAST_KEYS().
agcStatic().
print "05":tostring:padright(2) at (22, 9). 
print "09":tostring:padright(2) at (32, 9).
wait 0.3.
print "+00000":padleft(7) at (32,11).
wait 0.3.
print "-00000":padleft(7) at (32,13).
wait 0.3.
print oldERR:tostring():padleft(7) at (32,15).
wait 1.1.
print "":padleft(7) at (32,11).
print "":padleft(7) at (32,13).
print "":padleft(7) at (32,15).
set oldERR to ERR.
set ERR to 0.
wait 0.5.
VNP_DATA().

// Start of computer software

until program = 00 {
    updateState().
    agcData().
    local tmes to time:seconds.
    //VAC_CLEAR().

    if program = 1 {
        unlock steering.
        unlock throttle.
    }

    if program = 12 { // BURN_BABY_BURN

        set P12BIT to true.
        if not ROUTINES["R30"] {set ROUTINES["R30"] to true.}
        if SURFLG {set SURFLG to false.}
    
        local ascentData to ROUTINE_CALCS().
        local clearanceAtt to ship:facing.

        set BLANK_BIT to not proceedFlag.
        lock throttle to engineIgnitionPermission().

        if TLTFLG {
            // this should preserve memory and turn towards orbital insertion only when clear of the ship
            // burn with LTGL until apoapsis is near tgt per. then shiftover to PID control
            lock steering to heading(270, ascentData[0], 0).
            if ascentData[1] < 0.15 {
                lock throttle to 0. set ship:control:pilotmainthrottle to 0.
                set program to 1.
            }
        }
        else { // have it ascend the same orientation for 10m to prevent potential FOD
            if trueRadar < 6 {lock steering to clearanceAtt.}
            if ownVy < 15 and trueRadar > 6 {lock steering to heading(270, 90, 0).}
        }
    }

    if program = 20 { // orbital rendezvous
        set ROUTINES["R30"] to true.
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

        //print orbitalData[2] at (2, 2).
        print orbitalData[3] at (2, 18).

        if orbitalData[4] {
            print "apo " + orbitalData[2] at (2,2).
            if orbitalData[1] < 60 {set warp to 0.
            lock steering to orbitalData[2] * ship:velocity:orbit.}
            if BURN_FLAG {
                set ship:control:fore to 1-stab.
                lock throttle to engineIgnitionPermission() * sqrt(abs(orbitalData[2])/5).
                if abs(orbitalData[2]) <= 4 {
                    set BURN_FLAG to false.
                    lock throttle to 0.
                    unlock steering.
                }
            }
        }
        if orbitalData[5] {
            print "per " + orbitalData[3] at (2,2).
            if orbitalData[0] < 60 {set warp to 0.
            lock steering to orbitalData[3] * ship:velocity:orbit.}
            if BURN_FLAG {
                set ship:control:fore to 1-stab.
                lock throttle to engineIgnitionPermission() * sqrt(abs(orbitalData[3])/5).
                if abs(orbitalData[3]) <= 4 {
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
    // Okay so off to PCM here, then resume @ CDH

    if program = 34 { // TPI 
        if not ROUTINES["R30"] {set ROUTINES["R30"] to true.}
        
        local orbitalData to ROUTINE_CALCS().
        local stab to getEngineStability().

        if orbitalData[0] < 0.02 and orbitalData[2] < 0.02 { // phase angle. change this to distance
            lock throttle to 0. unlock steering.
        }
        
        if RNDFLG { // this should fall into P32-P35
            if orbitalData[5] <= 90 {
                set warp to 0.
            }
            if TPIPRFLG {    //time to target phase angle
                lock steering to orbitalData[3]/abs(orbitalData[3]) * ship:velocity:orbit.// * (orbitalData[3])/abs(orbitalData[3]). Inverse or parallel direction to prograde
                if BURN_FLAG {
                    set ship:control:fore to 1-stab.
                    lock throttle to engineIgnitionPermission() * stab * orbitalData[3] * getTwr()/g0. // this way unless the stability is > 0, it won't fire
                    if abs(orbitalData[3]) < 0.5 {
                        lock throttle to 0.
                        set BURN_FLAG to false.
                        set TPIPRFLG to false.
                        set TPIPOFLG to true.
                        // now this shifts to the next runmode, where V58 has use.
                    }
                }
            }
            if TPIPOFLG and performMINKEY {
                set program to 35.
            }
        }
        if PLNFLG {
            lock steering to vcrs(ship:velocity:orbit,-body:position).// * PLANE_CHANGE_DV(orbitalData[7])/abs(PLANE_CHANGE_DV(orbitalData[7])). For inverse/pll
            lock throttle to PLANE_CHANGE_DV(orbitalData[7]).
        }
    }

    if program = 35 { // Post TPI; TPF
        if TPIPOFLG {
            if not AUTOMANUBIT {
                unlock steering.
            }
            if AUTOMANUBIT {
                lock steering to tgtVessel:direction * R(0, 270, 90).
            }
            // figure out course correction
            // and est. closest appr.
            // via posAt() and iterate for a circle
        }
    }
    
    // P32-P35. Setup a P20 BIT that lets it know you've got tracking data  
    // I wonder if I can setup a MCC state vector upload through archive + JSON reading

    if program = 63 { // velocity reduction
        set pitchReqPID:maxoutput to 0.
        set pitchReqPID:minoutput to -20.
        set pitchReqPID:ki to 0.021.
        set pitchReqPID:setpoint to 332.
        
        local throttleData is ROUTINE_CALCS().
        set steeringManager:rollcontrolanglerange to 180.
        lock steering to srfRetrograde.
        //print proceedFlag at (2,0).

        if not DESFLG {
            if not ROUTINES["R30"] {set ROUTINES["R30"] to true.}
            if (timeToIgn) < 15 {set warp to 0.}
            if (timeToIgn) < 7 {
                set ship:control:fore to 1.
            }
            if abs(timeToIgn) < 2 {
                if engineIgnitionPermission() = 1 {
                    lock throttle to 0.1.
                    set ship:control:fore to 0.
                }
                set DESFLG to LET > 26.
            }
        }
        if DESFLG {
            if not ROUTINES["R31"] {set ROUTINES["R31"] to true.}
            //steeringCommand().
            if ROLFLG {
                lock steering to srfRetrograde * -r(pitchReqPID:update(time:seconds, PITCH_LAND_GUIDE()), yawReqPID:update(time:seconds, YAW_LAND_GUIDE()), 162). // set our steering so that we get to target.
            }
            else {
                lock steering to srfRetrograde * -r(0, yawReqPID:update(time:seconds, -1 * YAW_LAND_GUIDE()), 0).
            }
            lock throttle to max(0.1, throttleData).
            if ownVx < 320 {
                set program to 64.
            }
        }
    }

    if program = 64 { // trajectory control
        local throttleData is ROUTINE_CALCS().
        local errorROC to changeRate(MESTIM, errorDistance, 0.5).
        set LPD_TERM to round(max(0, (50-errorDistance)/(errorROC))).
        if not ROUTINES["R38"] {set ROUTINES["R38"] to true.}

        lock steering to srfRetrograde * -r(pitchReqPID:update(time:seconds, PITCH_LAND_GUIDE()), yawReqPID:update(time:seconds, YAW_LAND_GUIDE()), 162).
        lock throttle to max(0.1, throttleData).
        
        set pitchReqPID:maxoutput to 0.
        set pitchReqPID:minoutput to -45.
        set pitchReqPID:setpoint to 0.
        set pitchReqPID:ki to 1.
        set pitchReqPID:kp to 2.

        if LPD_TERM >= 5 {
            LPD_UPDATE(ship:control:pilottranslation:x, ship:control:pilottranslation:y).
        }

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
        lock throttle to max(0.1, min(0.6, throttlePid:update(time:seconds, ownVy))). // PGM 66, or rate of descent, lets us descent at a very slow rate.
        if trueRadar <= 0 or ship:status = "landed" {
            set program to 68. // program 68 is confirmation of touchdown.
        }
    }

    if program = 68{
        set SURFLG to true. // I think this was used for abort confirmation of not using the DM engine
        lock throttle to 0.
        unlock steering.
        SAS off.
        RCS off.
        set steeringManager:rollcontrolanglerange to 1.
        VAC_CLEAR().
    }

    if program = 70 { // DPS abort, recirc. Take from P12
    }

    if program = 71 or abort{ // APS abort, ascent+recirc. Take from P12
        lock steering to up.
    }

    VAC_ACCUMULATION().
    print (time:seconds - tmes) at (2,18).
    wait 0.01.
}

deletePath("compState.json").