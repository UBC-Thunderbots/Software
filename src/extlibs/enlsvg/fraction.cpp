#include "extlibs/enlsvg/fraction.h"

#include <iostream>
#include <sstream>
#include <string>
using namespace std;

namespace Pathfinding
{
    Fraction operator*(const Fraction& o, const Fraction& o_2)
    {
        return Fraction(o.n * o_2.n, o.d * o_2.d);
    }

    Fraction operator/(const Fraction& o, const Fraction& o_2)
    {
        return Fraction(o.n * o_2.d, o.d * o_2.n);
    }

    Fraction operator+(const Fraction& o, const Fraction& o_2)
    {
        return Fraction(o.n * o_2.d + o_2.n * o.d, o.d * o_2.d);
    }

    Fraction operator-(const Fraction& o, const Fraction& o_2)
    {
        return Fraction(o.n * o_2.d - o_2.n * o.d, o.d * o_2.d);
    }

    Fraction operator*(const Fraction& o, long o_2)
    {
        return Fraction(o.n * o_2, o.d);
    }

    Fraction operator/(const Fraction& o, long o_2)
    {
        return Fraction(o.n, o.d * o_2);
    }

    Fraction operator+(const Fraction& o, long o_2)
    {
        return Fraction(o.n + o_2 * o.d, o.d);
    }

    Fraction operator-(const Fraction& o, long o_2)
    {
        return Fraction(o.n - o_2 * o.d, o.d);
    }

    Fraction operator*(long o, const Fraction& o_2)
    {
        return Fraction(o * o_2.n, o_2.d);
    }

    Fraction operator/(long o, const Fraction& o_2)
    {
        return Fraction(o * o_2.d, o_2.n);
    }

    Fraction operator+(long o, const Fraction& o_2)
    {
        return Fraction(o * o_2.d + o_2.n, o_2.d);
    }

    Fraction operator-(long o, const Fraction& o_2)
    {
        return Fraction(o * o_2.d - o_2.n, o_2.d);
    }

    ostream& operator<<(ostream& stream, Fraction& obj)
    {
        return stream << obj.toString();
    }


    Fraction parseFraction(string s)
    {
        long i = s.find_first_of('/');
        if (i == string::npos)
        {
            long n;
            stringstream(s) >> n;

            return Fraction(n);
        }
        else
        {
            long n, d;
            stringstream(s.substr(0, i)) >> n;
            stringstream(s.substr(i + 1)) >> d;

            return Fraction(n, d);
        }
    }

}  // namespace Pathfinding
