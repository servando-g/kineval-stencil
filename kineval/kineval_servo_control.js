
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | arm servo control

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.setpointDanceSequence = function execute_setpoints() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_dance) return; 

    // STENCIL: implement FSM to cycle through dance pose setpoints

    if (kineval.params.dance_pose_index >= kineval.params.dance_sequence_index.length) {
        kineval.params.dance_pose_index = 0;
        kineval.params.update_pd_dance = false;
        return;
    }

    idx = kineval.params.dance_sequence_index[kineval.params.dance_pose_index];

    for (x in robot.joints) {
        if (Math.abs(kineval.params.setpoint_target[x] - robot.joints[x].angle) > 0.01) {
            return;
        }
        kineval.params.setpoint_target[x] = kineval.setpoints[idx][x];
    }

    kineval.params.dance_pose_index++;

}

kineval.setpointClockMovement = function execute_clock() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_clock) return; 

    var curdate = new Date();
    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = curdate.getSeconds()/60*2*Math.PI;
    }
}


kineval.robotArmControllerSetpoint = function robot_pd_control () {

    // if update not requested, exit routine
    if ((!kineval.params.update_pd)&&(!kineval.params.persist_pd)) return; 

    kineval.params.update_pd = false; // if update requested, clear request and process setpoint control

    // STENCIL: implement P servo controller over joints    
    
    for (x in robot.joints) {
        error = kineval.params.setpoint_target[x]-robot.joints[x].angle;
        robot.joints[x].control = robot.joints[x].servo.p_gain * error;
    }

}


