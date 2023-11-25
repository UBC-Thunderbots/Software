#include "extlibs/enlsvg/indirect_heap.h"

#include <iostream>

namespace Pathfinding
{
    // Runs in constant time if reinitialising heap of same size.
    void IndirectHeap::initialise(int capacity, double default_key)
    {
        this->default_key = default_key;
        this->heap_size   = 0;

        if (ticket_check.size() != capacity)
        {
            key_list.resize(capacity);
            in_list.resize(capacity);
            out_list.resize(capacity);

            ticket_check.clear();
            ticket_check.resize(capacity, 0);
            ticket_number = 1;
        }
        else if (ticket_number == -1)
        {
            ticket_check.clear();
            ticket_check.resize(capacity, 0);
            ticket_number = 1;
        }
        else
        {
            ticket_number++;
        }
    }

    void IndirectHeap::reinitialise()
    {
        this->default_key = POS_INF;
        this->heap_size   = 0;

        if (ticket_number == -1)
        {
            std::fill(ticket_check.begin(), ticket_check.end(), 0);
            ticket_number = 1;
        }
        else
        {
            ticket_number++;
        }
    }

    IndirectHeap::IndirectHeap(int capacity) : heap_size(0)
    {
        initialise(capacity, POS_INF);
    }

    // Runtime: O(lgn)
    void IndirectHeap::decreaseKey(int out_index, double new_key)
    {
        // Assume new_key < old key
        int in_index = getIn(out_index);

        // Optimisation: Jump the newly set value to the bottom of the heap.
        // Faster if there are a lot of POSITIVE_INFINITY values.
        // This is equivalent to an insert operation.
        if (getKey(in_index) == POS_INF)
        {
            swapData(in_index, heap_size);
            in_index = heap_size;
            ++heap_size;
        }
        setKey(in_index, new_key);

        bubbleUp(in_index);
    }


    // Runtime: O(lgn)
    // returns index of min element
    int IndirectHeap::popMinIndex()
    {
        if (heap_size == 0)
            std::cout << "ERROR: EMPTY HEAP!" << std::endl;
        else if (heap_size == 1)
        {
            --heap_size;
            return getOut(0);
        }

        int s = getOut(0);
        swapData(0, heap_size - 1);

        --heap_size;
        bubbleDown(0);

        return s;
    }

}  // namespace Pathfinding
