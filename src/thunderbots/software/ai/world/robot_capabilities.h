#pragma once
#include <bitset>
#include <initializer_list>
#include <ostream>

class RobotCapabilityFlags
{
   public:
    typedef enum
    {
        Kick    = 0,
        Chip    = 1,
        Dribble = 2,
        MAX     = 3
    } RobotCapability;

    static RobotCapabilityFlags allCapabilities();

    RobotCapabilityFlags(const std::initializer_list<RobotCapability>& capabilities);

    bool hasCapability(const RobotCapability& capability) const;

    bool hasAllCapabilities(const RobotCapabilityFlags& other) const;

    bool operator==(const RobotCapabilityFlags& other) const;

    void addCapability(const RobotCapability& capability);

    void removeCapability(const RobotCapability& capability);

   private:
    std::bitset<MAX> capability_bits;
};

inline RobotCapabilityFlags RobotCapabilityFlags::allCapabilities()
{
    return RobotCapabilityFlags{RobotCapabilityFlags::Kick, RobotCapabilityFlags::Chip,
                                RobotCapabilityFlags::Dribble};
}

inline RobotCapabilityFlags::RobotCapabilityFlags(
    const std::initializer_list<RobotCapability>& _capabilities)
{
    for (const RobotCapability& cap : _capabilities)
    {
        capability_bits.set(cap);
    }
}

inline bool RobotCapabilityFlags::hasCapability(const RobotCapability& capability) const
{
    return capability_bits.test(capability);
}

inline bool RobotCapabilityFlags::operator==(const RobotCapabilityFlags& other) const
{
    return capability_bits == other.capability_bits;
}

inline void RobotCapabilityFlags::addCapability(
    const RobotCapabilityFlags::RobotCapability& capability)
{
    capability_bits.set(capability, false);
}

inline void RobotCapabilityFlags::removeCapability(
    const RobotCapabilityFlags::RobotCapability& capability)
{
    capability_bits.set(capability);
}

inline bool RobotCapabilityFlags::hasAllCapabilities(
    const RobotCapabilityFlags& other) const
{
    return (capability_bits & other.capability_bits) == other.capability_bits;
}

inline std::ostream& operator<<(std::ostream& os,
                                const RobotCapabilityFlags& capabilityFlags)
{
    os << "{";
    if (capabilityFlags.hasCapability(RobotCapabilityFlags::Dribble))
    {
        os << "Dribble, ";
    }
    if (capabilityFlags.hasCapability(RobotCapabilityFlags::Kick))
    {
        os << "Kick, ";
    }
    if (capabilityFlags.hasCapability(RobotCapabilityFlags::Chip))
    {
        os << "Chip";
    }
    os << "}";
    return os;
}
