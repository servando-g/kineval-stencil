/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | Graph Search Methods

    Stencil methods for student implementation of graph search algorithms.

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

function initSearchGraph() {

    // create the search queue
    visit_queue = [];

    // initialize search graph as 2D array over configuration space
    //   of 2D locations with specified spatial resolution
    G = [];
    for (iind=0,xpos=-2;xpos<7;iind++,xpos+=eps) {
        G[iind] = [];
        for (jind=0,ypos=-2;ypos<7;jind++,ypos+=eps) {
            G[iind][jind] = {
                i:iind,j:jind, // mapping to graph array
                x:xpos,y:ypos, // mapping to map coordinates
                parent:null, // pointer to parent in graph along motion path
                distance:10000, // distance to start via path through parent
                visited:false, // flag for whether the node has been visited
                priority:null, // visit priority based on fscore
                queued:false // flag for whether the node has been queued for visiting
            };

            // STENCIL: determine whether this graph node should be the start
            //   point for the search

            x_init = q_init[0];
        
            y_init = q_init[1];

            x_upp = G[iind][jind].x + ((eps-0.01)/2);
            x_low = G[iind][jind].x - ((eps-0.01)/2);

            y_upp = G[iind][jind].y + ((eps-0.01)/2);
            y_low = G[iind][jind].y - ((eps-0.01)/2);

            if ((x_upp > x_init && x_low < x_init) && (y_upp > y_init && y_low < y_init)) {
                visit_queue.push(G[iind][jind]);
                G[iind][jind].distance = 0;
                G[iind][jind].queued = true;
            }

        }
    }

}

function iterateGraphSearch() {


    // STENCIL: implement a single iteration of a graph search algorithm
    //   for A-star (or DFS, BFS, Greedy Best-First)
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "iterating" otherwise.
    //
    //   When search is complete ("failed" or "succeeded") set the global variable 
    //   search_iterate to false. 
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   drawHighlightedPathGraph - draws a path back to the start location
    //   draw_2D_configuration - draws a square at a given location

    if (visit_queue.length == 0) {
        search_iterate = false;
        return "failed";
    }
    
    node = minheap_extract(visit_queue);
    node.visited = true;

    draw_2D_configuration([G[node.i][node.j].x,G[node.i][node.j].y], "visited");

    i = node.i;
    j = node.j;

    // Right node from curent node
    if (i != 800) {
        adjacent_nbr(node,a=1,b=0);
        collide = adjacent_nbr(node,a=1,b=0);
    }
    // Bottom node from current node
    if (j != 800) {
        adjacent_nbr(node,a=0,b=1);
        collide = adjacent_nbr(node,a=0,b=1);
    }
    // Left node from current node
    if (i != 0) {
        adjacent_nbr(node,a=-1,b=0);
        collide = adjacent_nbr(node,a=-1,b=0);
    }
    // Top node from current node
    if (j != 0) {
        adjacent_nbr(node,a=0,b=-1);
        collide = adjacent_nbr(node,a=0,b=-1);
    }

    if (collide == true) {
        return "failed";
    }

    if (!search_iterate) {
        return "succeeded"
    }

    return "iterating";
    
}

// This function will check the neighbor node has been visited.
// Input: node - the current node
//        a - the shift in i
//        b - the shift in j
function adjacent_nbr(node, a=0, b=0) {
    if (testCollision([G[node.i+a][node.j+b].x,G[node.i+a][node.j+b].y])) {
        return true;
    }
    if (G[node.i+a][node.j+b].visited == false) {
        if (G[node.i+a][node.j+b].distance > node.distance + eps) {
            G[node.i+a][node.j+b].parent = node;
            G[node.i+a][node.j+b].distance = h_score(G[node.i][node.j]);
            G[node.i+a][node.j+b].priority = G[node.i+a][node.j+b].distance + g_score(G[node.i+a][node.j+b]);
            minheap_insert(visit_queue, G[node.i+a][node.j+b])
            G[node.i+a][node.j+b].queued = true;
            draw_2D_configuration([G[node.i+a][node.j+b].x,G[node.i+a][node.j+b].y], "queued");

        }
        if (end_goal(G[node.i+a][node.j+b])) {
            drawHighlightedPathGraph(G[node.i+a][node.j+b]);
            search_iterate = false;
            return search_iterate;
        }
    }
    return;
}

// This function will check to see if the next node is the goal node.
// Input: node - the current node
function end_goal(node) {
    x_goal = q_goal[0];
        
    y_goal = q_goal[1];

    x_upp = node.x + ((eps-0.01)/2);
    x_low = node.x - ((eps-0.01)/2);

    y_upp = node.y + ((eps-0.01)/2);
    y_low = node.y - ((eps-0.01)/2);

     if ((x_upp > x_goal && x_low < x_goal) && (y_upp > y_goal && y_low < y_goal)) {
        return true;
     }
     else {
        return false;
     }

}

// This function will provide the distance to the goal node
function g_score(nbr) {

    x_dif = (q_goal[0] - nbr.x) ** 2;
    y_dif = (q_goal[1]- nbr.y) ** 2;

    return Math.sqrt(x_dif + y_dif);
}

//This function will provide the distance to the start node
function h_score(parent) {

    return parent.distance + eps;
    
}


//////////////////////////////////////////////////
/////     MIN HEAP IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

    // STENCIL: implement min heap functions for graph search priority queue.
    //   These functions work use the 'priority' field for elements in graph.

    // If the visit_queue does not have an item it will fail since there is nothing
    // to determine. 

function minheap_insert(heap, new_element) {

    // STENCIL: implement your min binary heap insert operation
    // push() places the new element at the end of the array
    heap.push(new_element);

    // The last index of heap
    let current_index = heap.length - 1;

    while (current_index > 0) {
        let parent_index = Math.round(current_index/2) - 1;
        if (heap[parent_index].priority > heap[current_index].priority) {
            let temp = heap[current_index];
            heap[current_index] = heap[parent_index];
            heap[parent_index] = temp;
            current_index = parent_index;
        }
        else {
            break;
        }
    }

    
}

function minheap_extract(heap) {

    if (heap.length == 0) {
        return;
    }
    else if (heap.length == 1) {
        return heap.pop();
    }

    // STENCIL: implement your min binary heap extract operation
    let first_index = heap[0];
    
    heap[0] = heap.pop();

    var current_index = 0;

    var left_index = (current_index * 2) + 1;
    var right_index = (current_index * 2) + 2;

    while ((left_index < heap.length && heap[current_index].priority > heap[left_index]?.priority) || 
           (right_index < heap.length && heap[current_index].priority > heap[right_index]?.priority)) {

        let index = left_index;
        if (heap[left_index].priority > heap[right_index].priority) {
            index = right_index;
        }

        if (heap[current_index].priority > heap[index].priority) {
            let temp = heap[current_index];
            heap[current_index] = heap[index];
            heap[index] = temp;
            current_index = index;
            left_index = (current_index * 2) + 1;
            right_index = (current_index * 2) + 2;
        }
        
        if (left_index > heap.length - 1 || right_index > heap.length - 1) {
            return first_index;
        }
    }

    return first_index

}