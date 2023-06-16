#include <exception>
#include <string>

class PowerServiceException
{
   public:
    PowerServiceException(const std::string& message);
    const char* what() const noexcept;

   private:
    std::string message_;
};
