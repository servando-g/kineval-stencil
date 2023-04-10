/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | RRT Methods

    Stencil methods for student implementation of RRT-based search.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see search_canvas.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

function iterateRRT() {


    // STENCIL: implement a single iteration of an RRT algorithm.
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "extended" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   insertTreeVertex - adds and displays new configuration vertex for a tree
    //   insertTreeEdge - adds and displays new tree edge between configurations
    //   drawHighlightedPath - renders a highlighted path in a tree
}

function iterateRRTConnect() {

    if (search_iter_count == search_max_iterations) {
        return "Failed"
    }

    var q_rand = randomConfig()
    var q_new = []
    var q_dummy = []
    if (extendRRT(T_a, q_rand, q_new) != "Trapped") {
        if (connectRRT(T_b, q_new, q_dummy) == "Reached") {
            dfsPath(T_a, T_b)
            search_iterate = false
            return "Succeeded"
        }
    }

    temp = T_a
    T_a = T_b
    T_b = temp

    return "Extended"

    // STENCIL: implement a single iteration of an RRT-Connect algorithm.
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "extended" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   insertTreeVertex - adds and displays new configuration vertex for a tree
    //   insertTreeEdge - adds and displays new tree edge between configurations
    //   drawHighlightedPath - renders a highlighted path in a tree
}

function iterateRRTStar() {

}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

    // STENCIL: implement RRT-Connect functions here, such as:
    //   extendRRT
    //   connectRRT
    //   randomConfig
    //   newConfig
    //   findNearestNeighbor
    //   dfsPath

function extendRRT(T,q, q_new) {
    var near_idx = findNearestNeighbor(q,T)
    var q_near = T.vertices[near_idx].vertex
    if (!newConfig(q, q_near, q_new)) {
        insertTreeVertex(T, q_new)
        T.vertices[(T.vertices.length - 1)].parent = near_idx
        insertTreeEdge(T, near_idx, (T.vertices.length - 1))
        var distance = calcDistance(q_new, q)
        if (distance <= eps) {
            return "Reached"
        }
        else {
            return "Advanced"
        }
    }
    return "Trapped"
}

function calcDistance(q_new, q) {
    var x = (q_new[0] - q[0]) ** 2
    var y = (q_new[1] - q[1]) ** 2

    var distance = Math.sqrt(x+y)

    return distance

}
  
function connectRRT(T,q, q_dummy) {
    
    while (1) {

        var S = extendRRT(T,q,q_dummy)

        if (S != "Advanced") {
            return S
        }

        q_dummy = []

    }
}

function randomConfig() {
    var x = Math.random()*(10)+(-2)
    var y = Math.random()*(10)+(-2)

    var q_rand = [x, y]

    return q_rand
}

function newConfig(q, q_near, q_new) {

    var x = (q[0] - q_near[0])
    var y = (q[1] - q_near[1]) 

    var distance = Math.sqrt(x**2+y**2)

    var ux = x/distance
    var uy = y/distance

    q_new[0] = q_near[0] + eps*ux
    q_new[1] = q_near[1] + eps*uy

    return testCollision(q_new)

}

function findNearestNeighbor(q, T) {
    
    var nearest_nbr = 0
    var min_distance = undefined

    for (let index = 0; index < T.vertices.length; index++) {

        var t_x = T.vertices[index].vertex[0]
        var t_y = T.vertices[index].vertex[1]

        var x = (q[0] - t_x) ** 2
        var y = (q[1] - t_y) ** 2

        var distance = Math.sqrt(x+y)

        if (min_distance == undefined) {
            min_distance = distance
            nearest_nbr = index
        }
        else if (distance < min_distance) {
            min_distance = distance
            nearest_nbr = index
        }   

    }

    return nearest_nbr

}

function dfsPath(Ta, Tb) {
    var shared_node = Ta.vertices.length - 1

    var from_start = []
    var to_goal = []

    var idx = shared_node

    while (idx != 0) {
        from_start.unshift(Ta.vertices[idx])

        idx = Ta.vertices[idx].parent
    }

    from_start.unshift(Ta.vertices[0])

    idx = Tb.vertices[Tb.vertices.length - 1].parent
    while (idx != 0) {
        to_goal.push(Tb.vertices[idx])

        idx = Tb.vertices[idx].parent
    }

    to_goal.push(Tb.vertices[0])

    var path = from_start.concat(to_goal)

    drawHighlightedPath(path)
}