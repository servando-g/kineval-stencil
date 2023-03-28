//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}


// STENCIL: reference matrix code has the following functions:
//   matrix_multiply
//   matrix_transpose
//   matrix_pseudoinverse
//   matrix_invert_affine
//   vector_normalize
//   vector_cross
//   generate_identity
//   generate_translation_matrix
//   generate_rotation_matrix_X
//   generate_rotation_matrix_Y
//   generate_rotation_matrix_Z



// **** Function stencils are provided below, please uncomment and implement them ****//



function matrix_multiply(m1,m2) {
    // returns 2D array that is the result of m1*m2

    // Check to see if the number of columns in m1 
    // equals the number of rows in m2
    if (m1[0].length != m2.length) {
        return
    }
    
    var result  = new Array(m1.length);
    
    for (let i = 0; i < m1.length; i++) {
        result[i] = []
        for (let j = 0; j < m2[0].length; j++) {
            var sum = 0;
            for (let k = 0; k < m1[0].length; k++) {
                sum += m1[i][k] * m2[k][j];
            }
            result[i][j] = sum;
        }
    }
    return result;
}

function matrix_transpose(m) {
    // returns 2D array that is the result of m1*m2

    const m_trans = new Array(m[0].length);
    
    for (let i = 0; i < m[0].length; i++) {
        m_trans[i] = new Array(m.length)
        for (let j = 0; j < m.length; j++) {
            m_trans[i][j] = m[j][i];   
        }
    }

    return m_trans;

}

function matrix_pseudoinverse(m) {
    // returns pseudoinverse of matrix m

    var mt = matrix_transpose(m);



    if (m.length > m[0].length) {
        var mtm = matrix_multiply(mt, m);
        var imtm = numeric.inv(mtm);
        var inv = matrix_multiply(imtm, mt);
    }
    else {
        var mmt = matrix_multiply(m, mt);
        var immt = numeric.inv(mmt);
        var inv = matrix_multiply(immt);
    }

    return inv

}

// function matrix_invert_affine(m) {
//     // returns 2D array that is the invert affine of 4-by-4 matrix m

// }

function vector_normalize(v) {
    // returns normalized vector for v
    var sum = 0;
    for (let i = 0; i < v.length; i++) {
        sum += v[i]**2;
    }
    
    var norm_val = Math.sqrt(sum);

    for (let i = 0; i < v.length; i++) {
        v[i] = v[i]/norm_val;
    }

    return v;

}

function vector_cross(a,b) {
    // return cross product of vector a and b with both has 3 dimensions
    c = [(a[1]*b[2] - a[2]*b[1]),(a[2]*b[0] - a[0]*b[2]),(a[0]*b[1] - a[1]*b[0])];
    return c;
}

function generate_identity() {
    // returns 4-by-4 2D array of identity matrix
    const id_mat = [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ]; 

    return id_mat;
}

function generate_translation_matrix(tx, ty, tz) {
    // returns 4-by-4 matrix as a 2D array
    const trans_mat = [
        [1, 0, 0, tx],
        [0, 1, 0, ty],
        [0, 0, 1, tz],
        [0, 0, 0, 1]
    ]; 

    return trans_mat;
}

function generate_rotation_matrix_X(angle) {
    // returns 4-by-4 matrix as a 2D array, angle is in radians
    const rx_mat = [
        [1, 0, 0, 0],
        [0, Math.cos(angle), -Math.sin(angle), 0],
        [0, Math.sin(angle), Math.cos(angle), 0],
        [0, 0, 0, 1]
    ]; 

    return rx_mat;
}

function generate_rotation_matrix_Y(angle) {
    // returns 4-by-4 matrix as a 2D array, angle is in radians
    const ry_mat = [
        [Math.cos(angle), 0, Math.sin(angle), 0],
        [0, 1, 0, 0],
        [-Math.sin(angle), 0, Math.cos(angle), 0],
        [0, 0, 0, 1]
    ]; 

    return ry_mat;
    
}

function generate_rotation_matrix_Z(angle) {
    // returns 4-by-4 matrix as a 2D array, angle is in radians
    const rz_mat = [
        [Math.cos(angle), -Math.sin(angle), 0, 0],
        [Math.sin(angle), Math.cos(angle), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ]; 

    return rz_mat;

}