//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

// STENCIL: reference quaternion code has the following functions:
//   quaternion_from_axisangle
//   quaternion_normalize
//   quaternion_to_rotation_matrix
//   quaternion_multiply

// **** Function stencils are provided below, please uncomment and implement them ****//

kineval.quaternionFromAxisAngle = function quaternion_from_axisangle(axis,angle) {
    // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
    
    // axis = kineval.quaternionNormalize(axis);

    var q = {
        a: Math.cos(angle/2),
        b: axis[0] * Math.sin(angle/2),
        c: axis[1] * Math.sin(angle/2),
        d: axis[2] * Math.sin(angle/2)
    };

    return q

}

kineval.quaternionNormalize = function quaternion_normalize(q1) {
    // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component

    q_norm = Math.sqrt(((q1.a**2) + (q1.b**2) + (q1.c**2) + (q1.d**2)))
    
    var q = {
        a: q1.a/q_norm,
        b: q1.b/q_norm,
        c: q1.c/q_norm,
        d: q1.d/q_norm
    };

    return q;

}

kineval.quaternionMultiply = function quaternion_multiply(q1,q2) {
    // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
    
    var q = {
        a: q1.a * q2.a - q1.b * q2.b - q1.c * q2.c - q1.d * q2.d,
        b: q1.a * q2.b + q1.b * q2.a + q1.c * q2.d - q1.d * q2.c,
        c: q1.a * q2.c - q1.b * q2.d + q1.c * q2.a + q1.d * q2.b,
        d: q1.a * q2.d + q1.b * q2.c - q1.c * q2.b + q1.d * q2.a 
    };

    return q;

}

kineval.quaternionToRotationMatrix = function quaternion_to_rotation_matrix (q) {
    // returns 4-by-4 2D rotation matrix

    var m = [];
    m[0] = [1 - 2*q.c*q.c - 2*q.d*q.d, 2*q.b*q.c - 2*q.a*q.d, 2*q.b*q.d + 2*q.a*q.c, 0];
    m[1] = [2*q.b*q.c + 2*q.a*q.d, 1 - 2*q.b*q.b - 2*q.d*q.d, 2*q.c*q.d - 2*q.a*q.b, 0];
    m[2] = [2*q.b*q.d - 2*q.a*q.c, 2*q.c*q.d + 2*q.a*q.b, 1 - 2*q.b*q.b - 2*q.c*q.c, 0];
    m[3] = [0, 0, 0, 1];
    
    return m;

}