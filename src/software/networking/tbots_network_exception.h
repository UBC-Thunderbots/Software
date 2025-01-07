#pragma once

#include <string>

class TbotsNetworkException : public std::exception
{
   public:
    /**
     * Raises a networking exception that we can detect.
     *
     * @param message The message to be displayed when the exception is caught.
     */
    TbotsNetworkException(const std::string& message) : message(message) {}

    virtual const char* what() const noexcept override;

   private:
    std::string message;  // Explanation of the exception
};
