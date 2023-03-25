//   CREATE ROBOT STRUCTURE

// KE 

links_geom_imported = false;

//////////////////////////////////////////////////
/////     DEFINE ROBOT AND LINKS
//////////////////////////////////////////////////

// create robot data object
robot = new Object(); // or just {} will create new object

// give the robot a name
robot.name = "greeter";

// initialize start pose of robot in the world
robot.origin = {xyz: [0,0.25,0], rpy:[0,0,0]};  

// specify base link of the robot; robot.origin is transform of world to the robot base
robot.base = "base";  

        
// specify and create data objects for the links of the robot
robot.links = {
    "base": {},  
    "torso": {}, 
    "head": {},
    "rightshoulder": {},
    "leftshoulder": {},
    "rightforearm": {},
    "leftforearm": {},
  
};
/* for you to do
, "shoulder_left": {}  , "upperarm_left": {} , "forearm_left": {} };
*/

//////////////////////////////////////////////////
/////     DEFINE JOINTS AND KINEMATIC HIERARCHY
//////////////////////////////////////////////////

/*      joint definition template
        // specify parent/inboard link and child/outboard link
        robot.joints.joint1 = {parent:"link1", child:"link2"};
        // joint origin's offset transform from parent link origin
        robot.joints.joint1.origin = {xyz: [5,3,0], rpy:[0,0,0]}; 
        // joint rotation axis
        robot.joints.joint1.axis = [0.0,0.0,1.0]; 
*/


// roll-pitch-yaw defined by ROS as corresponding to x-y-z 
//http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file

// specify and create data objects for the joints of the robot
robot.joints = {};

robot.joints.torso_yaw= {parent:"base", child:"torso"};
robot.joints.torso_yaw.origin = {xyz: [0,0.25,0.0], rpy:[0,0,0]};
robot.joints.torso_yaw.axis = [0.0,0.0,0]; 

robot.joints.head_yaw = {parent: "torso", child:"head"};
robot.joints.head_yaw.origin = {xyz: [0,1.5,0], rpy:[0,0,0]};
robot.joints.head_yaw.axis = [0,0,0];

robot.joints.rightshoulder_pitch = {parent: "torso", child: "rightshoulder"};
robot.joints.rightshoulder_pitch.origin = {xyz: [0,1,-0.25], rpy:[-Math.PI/4,0,0]};
robot.joints.rightshoulder_pitch.axis = [1,0,0];

robot.joints.leftshoulder_pitch = {parent: "torso", child: "leftshoulder"};
robot.joints.leftshoulder_pitch.origin = {xyz: [0,1,0.25], rpy:[3*Math.PI/4,0,0]};
robot.joints.leftshoulder_pitch.axis = [0,0,1];

robot.joints.rightforearm_pitch = {parent: "rightshoulder", child: "rightforearm"};
robot.joints.rightforearm_pitch.origin = {xyz: [0,0.8,0], rpy:[0,0,0]};
robot.joints.rightforearm_pitch.axis = [0,0,1];

robot.joints.leftforearm_pitch = {parent: "leftshoulder", child: "leftforearm"};
robot.joints.leftforearm_pitch.origin = {xyz: [0,0.8,0], rpy:[0,0,0]};
robot.joints.leftforearm_pitch.axis = [0,0,1];




// specify name of endeffector frame
robot.endeffector = {};
robot.endeffector.frame = "torso_yaw";
robot.endeffector.position = [[0],[0],[0.0],[1]]

//red is x
//green is y
//blue is z

//////////////////////////////////////////////////
/////     DEFINE LINK threejs GEOMETRIES
//////////////////////////////////////////////////

/*  threejs geometry definition template, will be used by THREE.Mesh() to create threejs object
    // create threejs geometry and insert into links_geom data object
    links_geom["link1"] = new THREE.CubeGeometry( 5+2, 2, 2 );

    // example of translating geometry (in object space)
    links_geom["link1"].applyMatrix( new THREE.Matrix4().makeTranslation(5/2, 0, 0) );

    // example of rotating geometry 45 degrees about y-axis (in object space)
    var temp3axis = new THREE.Vector3(0,1,0);
    links_geom["link1"].rotateOnAxis(temp3axis,Math.PI/4);
*/

// define threejs geometries and associate with robot links 
links_geom = {};

links_geom["base"] = new THREE.CubeGeometry( 1, 0.5, 1 );
links_geom["base"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["torso"] = new THREE.CubeGeometry( 0.5, 2, 0.5 );
links_geom["torso"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0.5, 0) );

links_geom["head"] = new THREE.CubeGeometry( 0.75, 0.75, 0.75);
links_geom["head"].applyMatrix( new THREE.Matrix4().makeTranslation(0,0.375,0));

links_geom["rightshoulder"] = new THREE.CubeGeometry(0.3,0.8,0.3);
links_geom["rightshoulder"].applyMatrix( new THREE.Matrix4().makeTranslation(0.0,0.375,0));


links_geom["leftshoulder"] = new THREE.CubeGeometry(0.3,0.8,0.3);
links_geom["leftshoulder"].applyMatrix( new THREE.Matrix4().makeTranslation(0.0,0.375,0));

links_geom["rightforearm"] = new THREE.CubeGeometry(0.3,0.7,0.3);
links_geom["rightforearm"].applyMatrix( new THREE.Matrix4().makeTranslation(0,0.25,0));

links_geom["leftforearm"] = new THREE.CubeGeometry(0.3, 0.7, 0.3);
links_geom["leftforearm"].applyMatrix( new THREE.Matrix4().makeTranslation(0,0.25,0));





robot.partner_name = "servando";