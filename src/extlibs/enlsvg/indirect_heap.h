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
        void initialise(int size, double default_key);
        void reinitialise();
        IndirectHeap(int capacity);
        void decreaseKey(int out_index, double new_key);
        int popMinIndex();

       private:
        std::vector<double> key_list;
        std::vector<int> in_list;
        std::vector<int> out_list;
        int heap_size;

        double default_key = POS_INF;

        std::vector<int> ticket_check;
        int ticket_number = 0;

       public:
        inline double getMinValue() const
        {
            return getKey(0);
        }

       private:
        inline double getKey(int index) const
        {
            return ticket_check[index] == ticket_number ? key_list[index] : default_key;
        }

        inline int getIn(int index) const
        {
            return ticket_check[index] == ticket_number ? in_list[index] : index;
        }

        inline int getOut(int index) const
        {
            return ticket_check[index] == ticket_number ? out_list[index] : index;
        }

        inline void setKey(int index, double value)
        {
            if (ticket_check[index] != ticket_number)
            {
                key_list[index]     = value;
                in_list[index]      = index;
                out_list[index]     = index;
                ticket_check[index] = ticket_number;
            }
            else
            {
                key_list[index] = value;
            }
        }

        inline void setIn(int index, int value)
        {
            if (ticket_check[index] != ticket_number)
            {
                key_list[index]     = default_key;
                in_list[index]      = value;
                out_list[index]     = index;
                ticket_check[index] = ticket_number;
            }
            else
            {
                in_list[index] = value;
            }
        }

        inline void setOut(int index, int value)
        {
            if (ticket_check[index] != ticket_number)
            {
                key_list[index]     = default_key;
                in_list[index]      = index;
                out_list[index]     = value;
                ticket_check[index] = ticket_number;
            }
            else
            {
                out_list[index] = value;
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
            if (index1 >= heap_size)
            {
                if (index2 >= heap_size)
                    return -1;
                return index2;
            }
            if (index2 >= heap_size)
                return index1;

            return getKey(index1) < getKey(index2) ? index1 : index2;
        }

        inline void bubbleDown(int index)
        {
            int left_child    = 2 * index + 1;
            int right_child   = 2 * index + 2;
            int smaller_child = smallerNode(left_child, right_child);

            while (smaller_child != -1 && getKey(index) > getKey(smaller_child))
            {
                // If meets the conditions to bubble down,
                swapData(index, smaller_child);

                // Recurse
                index         = smaller_child;
                left_child    = 2 * index + 1;
                right_child   = 2 * index + 2;
                smaller_child = smallerNode(left_child, right_child);
            }
        }

       public:
        inline int size() const
        {
            return heap_size;
        }

        inline bool isEmpty() const
        {
            return heap_size <= 0;
        }
    };

}  // namespace Pathfinding

#endif
