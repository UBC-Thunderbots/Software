#pragma once

#include <cmath>
#include <numeric>
#include <vector>

/**
 * @brief Calculate the mean of a vector of data
 * 
 * @tparam T The type of the data
 *
 * @param data The data
 *
 * @return double The mean of the data
 */
template <typename T>
double mean(const std::vector<T> data)
{
    if (data.size() == 0)
    {
        return 0.0;
    }

    double sum = static_cast<double>(std::accumulate(data.begin(), data.end(), 0.0));
    return sum / static_cast<double>(data.size());
}

/**
 * @brief Calculate the sample standard deviation of a vector of data
 *
 * Returns 0.0 if the data has fewer than 2 elements.
 * 
 * @tparam T The type of the data
 *
 * @param data The data
 *
 * @return double The standard deviation of the data
 */
template <typename T>
double stdevSample(const std::vector<T> data)
{
    double n = static_cast<double>(data.size());

    if (n < 2)
    {
        return 0.0;
    }

    double data_mean = mean(data);

    double sq_residuals = std::accumulate(
        data.begin(), data.end(), 0.0, [data_mean](double res, double val) {
            return res + std::pow(static_cast<double>(val) - data_mean, 2);
        });

    return std::sqrt(sq_residuals / (n - 1));
}
