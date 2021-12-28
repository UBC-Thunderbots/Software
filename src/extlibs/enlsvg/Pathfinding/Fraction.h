#ifndef FRACTION_H
#define FRACTION_H

#include <string>
#include <sstream>
using namespace std;

namespace Pathfinding {

struct Fraction {
    int n, d;

    inline Fraction() {}
        
    inline Fraction(int n, int d) {
        init(n, d);
    }

    inline Fraction(int n): n(n), d(1) {}

    inline void init(int n) {
        this->n = n;
        this->d = 1;
    }

    inline void init(int n, int d) {
        /*if (d == 0) {
            cout << "\nERROR: Zero denominator: " << n << + "/" << d << "\n";
            return;
        }*/
        const int g = gcd(n,d);
        // Denominators are strictly positive.
        if (d/g > 0) {
            this->n = n/g;
            this->d = d/g;
        } else {
            this->n = -n/g;
            this->d = -d/g;
        }
    }

    inline float toFloat() const {
        return float(n)/d;
    }

    inline string toString() const {
        stringstream a;
        a << n;
        if (d != 1)
            a << "/" << d;
        return a.str();
    }

    inline static int gcd(int a, int b) {
        return a == 0 ? b : gcd(b%a, a);
    }

    /**
     * @return largest integer leq to this.
     */
    inline int floor() const {
        return n > 0 ? n/d : (n+1)/d - 1;
    }
    
    /**
     * @return smallest integer geq to this.
     */
    inline int ceil() const {
        return n > 0 ? (n-1)/d + 1 : n/d;
    }

    inline Fraction& operator*=(const Fraction& o) {
        init(n*o.n, d*o.d);
        return *this;
    }

    inline Fraction& operator/=(const Fraction& o) {
        init(n*o.d, d*o.n);
        return *this;
    }

    inline Fraction& operator+=(const Fraction& o) {
        init(n*o.d + o.n*d, d*o.d);
        return *this;
    }

    inline Fraction& operator-=(const Fraction& o) {
        init(n*o.d - o.n*d, d*o.d);
        return *this;
    }

    inline bool operator==(const Fraction &o) const {
        // simplest form is guaranteed unique.
        return (n == o.n) && (d == o.d);
    }

    inline bool operator!=(const Fraction &o) const {
        return !(*this == o);
    }

    inline bool operator<(const Fraction &o) const {
        // Note: denominators are POSITIVE.
        return o.d*n < o.n*d;
    }

    inline bool operator>(const Fraction &o) const {
        return o.d*n > o.n*d;
    }

    inline bool operator<=(const Fraction &o) const {
        return !(*this > o);
    }

    inline bool operator>=(const Fraction &o) const {
        return !(*this < o);
    }

    inline Fraction multiplyDivide(int multiply, int divide) const {
        return Fraction((long)n*multiply, (long)d*divide);
    }
    

    inline bool isPositive() const {
        return n > 0;
    }

    inline bool isNegative() const {
        return n < 0;
    }

    inline bool isZero() const {
        return n == 0;
    }

    inline bool isInvalid() const {
        return d == 0;
    }

    inline bool isWholeNumber() const {
        return d == 1;
    }
};

Fraction operator*(const Fraction& o, const Fraction& o2);
Fraction operator/(const Fraction& o, const Fraction& o2);
Fraction operator+(const Fraction& o, const Fraction& o2);
Fraction operator-(const Fraction& o, const Fraction& o2);
Fraction operator*(const Fraction& o, int o2);
Fraction operator/(const Fraction& o, int o2);
Fraction operator+(const Fraction& o, int o2);
Fraction operator-(const Fraction& o, int o2);
Fraction operator*(int o, const Fraction& o2);
Fraction operator/(int o, const Fraction& o2);
Fraction operator+(int o, const Fraction& o2);
Fraction operator-(int o, const Fraction& o2);
ostream& operator<< (ostream& stream, Fraction& obj);

Fraction parseFraction(string s);

}

#endif