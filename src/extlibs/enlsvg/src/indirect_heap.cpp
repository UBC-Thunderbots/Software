#include "extlibs/enlsvg/Pathfinding/indirect_heap.h"

#include <iostream>

namespace Pathfinding
{
    // Runs in constant time if reinitialising heap of same size.
    void IndirectHeap::initialise(int capacity, double defaultKey)
    {
        this->defaultKey = defaultKey;
        this->heapSize   = 0;

        if (ticketCheck.size() != capacity)
        {
            keyList.resize(capacity);
            inList.resize(capacity);
            outList.resize(capacity);

            ticketCheck.clear();
            ticketCheck.resize(capacity, 0);
            ticketNumber = 1;
        }
        else if (ticketNumber == -1)
        {
            ticketCheck.clear();
            ticketCheck.resize(capacity, 0);
            ticketNumber = 1;
        }
        else
        {
            ticketNumber++;
        }
    }

    void IndirectHeap::reinitialise()
    {
        this->defaultKey = POS_INF;
        this->heapSize   = 0;

        if (ticketNumber == -1)
        {
            std::fill(ticketCheck.begin(), ticketCheck.end(), 0);
            ticketNumber = 1;
        }
        else
        {
            ticketNumber++;
        }
    }

    IndirectHeap::IndirectHeap(int capacity) : heapSize(0)
    {
        initialise(capacity, POS_INF);
    }

    // Runtime: O(lgn)
    void IndirectHeap::decreaseKey(int outIndex, double newKey)
    {
        // Assume newKey < old key
        int inIndex = getIn(outIndex);

        // Optimisation: Jump the newly set value to the bottom of the heap.
        // Faster if there are a lot of POSITIVE_INFINITY values.
        // This is equivalent to an insert operation.
        if (getKey(inIndex) == POS_INF)
        {
            swapData(inIndex, heapSize);
            inIndex = heapSize;
            ++heapSize;
        }
        setKey(inIndex, newKey);

        bubbleUp(inIndex);
    }


    // Runtime: O(lgn)
    // returns index of min element
    int IndirectHeap::popMinIndex()
    {
        if (heapSize == 0)
            std::cout << "ERROR: EMPTY HEAP!" << std::endl;
        else if (heapSize == 1)
        {
            --heapSize;
            return getOut(0);
        }
        // nodeList.size() > 1

        // s = Data at 0 = out[0]
        // t = Data at lastIndex = out[lastIndex]
        // key[0] = key[lastIndex], remove key[lastIndex]
        // in[s] = -1
        // in[t] = 0
        // out[0] = out[lastIndex], remove out[lastIndex]

        // E temp = keyList.get(0);
        int s = getOut(0);
        swapData(0, heapSize - 1);

        --heapSize;
        bubbleDown(0);

        return s;
    }

}  // namespace Pathfinding
