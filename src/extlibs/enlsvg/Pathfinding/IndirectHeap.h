#ifndef _INDIRECT_HEAP_H_
#define _INDIRECT_HEAP_H_

#include <limits>
#include <vector>

namespace Pathfinding
{
    constexpr double POS_INF = std::numeric_limits<double>::infinity();

    class IndirectHeap
    {
       public:
        void initialise(int size, double defaultKey);
        void reinitialise();
        IndirectHeap(int capacity);
        void decreaseKey(int outIndex, double newKey);
        int popMinIndex();

       private:
        std::vector<double> keyList;
        std::vector<int> inList;
        std::vector<int> outList;
        int heapSize;

        double defaultKey = POS_INF;

        std::vector<int> ticketCheck;
        int ticketNumber = 0;

       public:
        inline double getMinValue() const
        {
            return getKey(0);
        }

       private:
        inline double getKey(int index) const
        {
            return ticketCheck[index] == ticketNumber ? keyList[index] : defaultKey;
        }

        inline int getIn(int index) const
        {
            return ticketCheck[index] == ticketNumber ? inList[index] : index;
        }

        inline int getOut(int index) const
        {
            return ticketCheck[index] == ticketNumber ? outList[index] : index;
        }

        inline void setKey(int index, double value)
        {
            if (ticketCheck[index] != ticketNumber)
            {
                keyList[index]     = value;
                inList[index]      = index;
                outList[index]     = index;
                ticketCheck[index] = ticketNumber;
            }
            else
            {
                keyList[index] = value;
            }
        }

        inline void setIn(int index, int value)
        {
            if (ticketCheck[index] != ticketNumber)
            {
                keyList[index]     = defaultKey;
                inList[index]      = value;
                outList[index]     = index;
                ticketCheck[index] = ticketNumber;
            }
            else
            {
                inList[index] = value;
            }
        }

        inline void setOut(int index, int value)
        {
            if (ticketCheck[index] != ticketNumber)
            {
                keyList[index]     = defaultKey;
                inList[index]      = index;
                outList[index]     = value;
                ticketCheck[index] = ticketNumber;
            }
            else
            {
                outList[index] = value;
            }
        }

        inline void bubbleUp(int index)
        {
            int parent = (index - 1) / 2;
            while (index > 0 && getKey(index) < getKey(parent))
            {
                // If meets the conditions to bubble up,
                swapData(index, parent);
                index  = parent;
                parent = (index - 1) / 2;
            }
        }

        inline void swapData(int a, int b)
        {
            // s = Data at a = out[a]
            // t = Data at b = out[b]
            // key[a] <-> key[b]
            // in[s] <-> in[t]
            // out[a] <-> out[b]

            int s = getOut(a);
            int t = getOut(b);

            swapKey(a, b);
            swapIn(s, t);
            swapOut(a, b);
        }

        /**
         * swap integers in list
         */
        inline void swapKey(int i1, int i2)
        {
            double temp = getKey(i1);
            setKey(i1, getKey(i2));
            setKey(i2, temp);
        }

        /**
         * swap integers in list
         */
        inline void swapOut(int i1, int i2)
        {
            int temp = getOut(i1);
            setOut(i1, getOut(i2));
            setOut(i2, temp);
        }

        /**
         * swap integers in list
         */
        inline void swapIn(int i1, int i2)
        {
            int temp = getIn(i1);
            setIn(i1, getIn(i2));
            setIn(i2, temp);
        }

        inline int smallerNode(int index1, int index2) const
        {
            if (index1 >= heapSize)
            {
                if (index2 >= heapSize)
                    return -1;
                return index2;
            }
            if (index2 >= heapSize)
                return index1;

            return getKey(index1) < getKey(index2) ? index1 : index2;
        }

        inline void bubbleDown(int index)
        {
            int leftChild    = 2 * index + 1;
            int rightChild   = 2 * index + 2;
            int smallerChild = smallerNode(leftChild, rightChild);

            while (smallerChild != -1 && getKey(index) > getKey(smallerChild))
            {
                // If meets the conditions to bubble down,
                swapData(index, smallerChild);

                // Recurse
                index        = smallerChild;
                leftChild    = 2 * index + 1;
                rightChild   = 2 * index + 2;
                smallerChild = smallerNode(leftChild, rightChild);
            }
        }

       public:
        inline int size() const
        {
            return heapSize;
        }

        inline bool isEmpty() const
        {
            return heapSize <= 0;
        }
    };

}  // namespace Pathfinding

#endif
