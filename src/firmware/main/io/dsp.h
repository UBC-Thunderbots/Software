#ifndef PROCESSING_H
#define PROCESSING_H

#include <stddef.h>

// Contains signal processing functions

// steps a linear transfer function forward via the direct form II method
// http://en.wikipedia.org/wiki/Digital_filter#Direct_Form_II
// requrires two vectors with the filter coefficients, one for the numerator (B coeffs)
// and one for the denominator (A coeffs, feedback terms)
// additionally, a state vector preserve the filter state between subsequent calls
// each coefficient vector should be of length ORDER+1 while the state vector should be of
// length ORDER The elements in the coefficient vectors should be in the z^-n ordering
// where n is index The first element in the A coeffs vector should(must?) be normalized
// to one.
float runDF2(float input, const float* num, const float* den, float* state, size_t order);

#endif
