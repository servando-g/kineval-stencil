
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 

    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }

    // STENCIL: call kineval.buildFKTransforms();
    kineval.buildFKTransforms();
}

kineval.buildFKTransforms = function buildFKTransforms () {
    mstack = [generate_identity()]
    this.traverseFKBase(robot.origin);
    this.traverseFKLink(robot.base);
}

kineval.traverseFKBase = function traverseFKBase (origin) {
    var translation = generate_translation_matrix(origin.xyz[0],origin.xyz[1],origin.xyz[2]);
    var r = matrix_multiply(generate_rotation_matrix_Y(origin.rpy[1]),generate_rotation_matrix_Z(origin.rpy[2]));
    var rotation = matrix_multiply(generate_rotation_matrix_X(origin.rpy[0]),r);

    var transform = matrix_multiply(translation, rotation);

    mstack.push(matrix_multiply(mstack[mstack.length - 1], transform));

    // robot.origin.xform = mstack[mstack.length -1];
}

kineval.traverseFKLink = function traverseFKLink (link) {
    // if (link != robot.base) {
    //     robot.links[link].xform = robot.joints[robot.links[link].parent].xform;
    // }
    if (robot.links[link].children) {
        
        for (var i = 0; i < robot.links[link].children.length; i++) {
            var child_name = robot.links[link].children[i];
            this.traverseFKJoint(child_name);
        }
        
    }
    robot.links[link].xform = mstack.pop()
}

kineval.traverseFKJoint = function traverseFKJoint (joint) {
    cur_joint = joint;
    var translation = generate_translation_matrix(robot.joints[joint].origin.xyz[0],robot.joints[joint].origin.xyz[1],robot.joints[joint].origin.xyz[2]);
    var r = matrix_multiply(generate_rotation_matrix_Y(robot.joints[joint].origin.rpy[1]), generate_rotation_matrix_Z(robot.joints[joint].origin.rpy[2]));
    var rotation = matrix_multiply(generate_rotation_matrix_X(robot.joints[joint].origin.rpy[0]),r);
    
    var transform = matrix_multiply(translation, rotation);

    xform = matrix_multiply(mstack[mstack.length-1], transform);

    mstack.push(xform);
    
    robot.joints[joint].xform = mstack[mstack.length - 1];

    var child_name = robot.joints[joint].child;
    this.traverseFKLink(child_name);
    
}

    // STENCIL: implement buildFKTransforms, which kicks off
    //   a recursive traversal over links and 
    //   joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // To use the keyboard interface, assign the global variables 
    //   "robot_heading" and "robot_lateral", 
    //   which represent the z-axis (heading) and x-axis (lateral) 
    //   of the robot's base in its own reference frame, 
    //   transformed into the world coordinates.
    // The axes should be represented in unit vector form 
    //   as 4x1 homogenous matrices

    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //

