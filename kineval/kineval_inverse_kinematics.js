
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

    // STENCIL: see instructor for random time trial code
    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

    // get endeffector Cartesian position in the world
    endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);

    // compute distance of endeffector to target
    kineval.params.trial_ik_random.distance_current = Math.sqrt(
                    Math.pow(kineval.params.ik_target.position[0][0]-endeffector_world[0][0],2.0)
                    + Math.pow(kineval.params.ik_target.position[1][0]-endeffector_world[1][0],2.0)
                    + Math.pow(kineval.params.ik_target.position[2][0]-endeffector_world[2][0],2.0) );

    // if target reached, increment scoring and generate new target location
    // KE 2 : convert hardcoded constants into proper parameters
    if (kineval.params.trial_ik_random.distance_current < 0.01) {
        kineval.params.ik_target.position[0][0] = 1.2*(Math.random()-0.5);
        kineval.params.ik_target.position[1][0] = 1.2*(Math.random()-0.5)+1.5;
        kineval.params.ik_target.position[2][0] = 0.7*(Math.random()-0.5)+0.5;
        kineval.params.trial_ik_random.targets += 1;
        textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;
    }
}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration

    // [NOTICE]: Please assign the following 3 variables to test against CI grader

    // ---------------------------------------------------------------------------
    // robot.dx = []              // Error term, matrix size: 6 x 1, e.g., [[1],[1],[1],[0],[0],[0]]
    // robot.jacobian = []        // Jacobian matrix of current IK iteration matrix size: 6 x N
    // robot.dq = []              // Joint configuration change term (don't include step length)  
    // ---------------------------------------------------------------------------

    // Explanation of above 3 variables:
    // robot.dq = T(robot.jacobian) * robot.dx  // where T(robot.jacobian) means apply some transformations to the Jacobian matrix, it could be Transpose, PseudoInverse, etc.
    // dtheta = alpha * robot.dq   // alpha: step length

    var T_p = matrix_multiply(robot.links[robot.joints[endeffector_joint].child].xform, endeffector_position_local)

    robot.dx = [[endeffector_target_world.position[0][0] - T_p[0][0]],
                [endeffector_target_world.position[1][0] - T_p[1][0]],
                [endeffector_target_world.position[2][0] - T_p[2][0]],
                [0],
                [0],
                [0]]

    jacobian = [[], [], [], [], [], []]
    var i = 0
    for (joint in robot.joints) {

        var axis = [[robot.joints[joint].axis[0]], [robot.joints[joint].axis[1]], [robot.joints[joint].axis[2]],[1]]
        var T_k = matrix_multiply(robot.joints[joint].xform, axis)

        var origin = [[0],[0],[0], [1]]
        var T_o = matrix_multiply(robot.joints[joint].xform, origin)

        var z_i = [T_k[0][0]-T_o[0][0],
                   T_k[1][0]-T_o[1][0],
                   T_k[2][0]-T_o[2][0]]

        if (typeof robot.joints[joint].type == 'undefined' || typeof robot.joints[joint].type == 'revolute') {
            var o = [T_p[0][0] - T_o[0][0],
                     T_p[1][0] - T_o[1][0],
                     T_p[2][0] - T_o[2][0]]

            var v_c = vector_cross(z_i, o)
            jacobian[0][i] = v_c[0]
            jacobian[1][i] = v_c[1]
            jacobian[2][i] = v_c[2]
            jacobian[3][i] = z_i[0]
            jacobian[4][i] = z_i[1]
            jacobian[5][i] = z_i[2]

        }
        else {
            jacobian[0][i] = z_i[0]
            jacobian[1][i] = z_i[1]
            jacobian[2][i] = z_i[2]
            jacobian[3][i] = 0
            jacobian[4][i] = 0
            jacobian[5][i] = 0
        }
        
        if (endeffector_joint == joint) {
            break
        }

        i++
   
    }

    var jacob = jacobian;

    if (kineval.params.ik_pseudoinverse) {
        jacob = matrix_pseudoinverse(jacob)
    }
    else {
        jacob = matrix_transpose(jacob)
    }

    dq = matrix_multiply(jacob, robot.dx)

    j = 0
    for (joint in robot.joints) {
        robot.joints[joint].control += kineval.params.ik_steplength*dq[j][0]

        if (endeffector_joint == joint) {
            break
        }
        
        j++
    }

    robot.jacobian = jacobian
    robot.dq = dq


}



