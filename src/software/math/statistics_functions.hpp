#pragma once

#include <numeric>

template <typename T>
double mean(const std::vector<T> data)
{
    double sum = static_cast<double>(std::accumulate(data.begin(), data.end(), 0.0));
    return sum / static_cast<double>(data.size());
}

template<typename T>
double stdevSample(const std::vector<T> data)
{
    double n = static_cast<double>(data.size());

    if (n < 2)
    {
        return 0.0;
    }

    double data_mean = mean(data);

    double sq_residuals = std::accumulate(data.begin(), data.end(), 0.0, [data_mean](double res, double val) { return res + pow(static_cast<double>(val) - data_mean, 2); });

    return std::sqrt(sq_residuals / (n-1));
}
