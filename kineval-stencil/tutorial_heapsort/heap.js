/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Heap Sort Stencil | JavaScript support functions

    Quick JavaScript Code-by-Example Tutorial 
     
    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License 

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/


// create empty object 
minheaper = {}; 

// define insert function for min binary heap
function minheap_insert(heap, new_element) {

    // STENCIL: implement your min binary heap insert operation
    // push() places the new element at the end of the array
    heap.push(new_element);

    // The last index of heap
    let current_index = heap.length - 1;

    while (current_index > 0) {
        let parent_index = Math.round(current_index/2) - 1;
        if (heap[parent_index] > heap[current_index]) {
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

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign 
      a reference to the function within the minheap object, which can be called
      as minheap.insert
*/

// define extract function for min binary heap
function minheap_extract(heap) {

    if (heap.length == 0) {
        return;
    }
    else if (heap.length == 1) {
        return heap.pop()
    }

    // STENCIL: implement your min binary heap extract operation
    let first_index = heap[0];
    heap[0] = heap.pop();

    var current_index = 0;

    var left_index = (current_index * 2) + 1;
    var right_index = (current_index * 2) + 2;

    while (heap[current_index] > heap[left_index] || heap[current_index] > heap[right_index]) {
        let index = left_index;
        if (heap[left_index] > heap[right_index]) {
            index = right_index;
        }

        if (heap[current_index] > heap[index]) {
            let temp = heap[current_index];
            heap[current_index] = heap[index];
            heap[index] = temp;
            current_index = index;
            left_index = (current_index * 2) + 1;
            right_index = (current_index * 2) + 2;
        } 
    }

    return first_index

}

// assign extract function within minheaper object
minheaper.extract = minheap_extract;
    // STENCIL: ensure extract method is within minheaper object
console.log(minheaper.insert)
console.log(minheaper.extract)


