#include <bitset>
#include <initializer_list>

class RobotCapabilities
{
   public:
    typedef enum
    {
        Kick    = 0,
        Chip    = 1,
        Dribble = 2,
        MAX     = 3
    } RobotCapability;

    static RobotCapabilities allCapabilities();

    RobotCapabilities(const std::initializer_list<RobotCapability>& _capabilities);

    bool hasCapability(const RobotCapability& capability) const;

    bool operator==(const RobotCapabilities& other) const;

   private:
    std::bitset<MAX> capabilities;
};

inline RobotCapabilities RobotCapabilities::allCapabilities()
{
    return RobotCapabilities{RobotCapabilities::Kick, RobotCapabilities::Chip,
                             RobotCapabilities::Dribble};
}

inline RobotCapabilities::RobotCapabilities(
    const std::initializer_list<RobotCapability>& _capabilities)
{
    for (const RobotCapability& cap : _capabilities)
    {
        capabilities.set(cap);
    }
}

inline bool RobotCapabilities::hasCapability(const RobotCapability& capability) const
{
    return capabilities.test(capability);
}

inline bool RobotCapabilities::operator==(const RobotCapabilities& other) const
{
    return capabilities == other.capabilities;
}
