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

    /**
     * returns a RobotCapabilityFlags object with all of the capabilities
     * @return RobotCapabilityFlags with all capabilities set
     */
    static RobotCapabilityFlags allCapabilities();

    /**
     * Construct a new RobotCapabilities with the specified capabilities
     * @param capabilities a list of capabilities
     */
    RobotCapabilityFlags(const std::initializer_list<RobotCapability>& capabilities);

    /**
     * Returns true if the RobotCapabilityFlags has the specified capability, otherwise
     * false
     * @param capability a capability
     * @return true if the RobotCapabilityFlags has the specified capability
     */
    bool hasCapability(const RobotCapability& capability) const;

    /**
     * Returns true if the current RobotCapabilityFlags has at least the capabilities of
     * the other RobotCapabilityFlags
     * @param other another RobotCapabilityFlags
     * @return true if this has at least the capabilities in the other
     * RobotCapabilityFlags
     */
    bool hasAllCapabilities(const RobotCapabilityFlags& other) const;

    /**
     * True if the other RobotCapabilityFlags has exactly the same capabilities as this
     * one
     * @param other another RobotCapabilityFlags
     * @return True if the other RobotCapabilityFlags has exactly the same capabilities as
     * this one
     */
    bool operator==(const RobotCapabilityFlags& other) const;

    /**
     * adds a capability to this RobotCapabilityFlags
     * @param capability a capability
     */
    void addCapability(const RobotCapability& capability);

    /**
     * Removes a capability from this RobotCapabilityFlags
     * @param capability a capability
     */
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
    capability_bits.set(capability, true);
}

inline void RobotCapabilityFlags::removeCapability(
    const RobotCapabilityFlags::RobotCapability& capability)
{
    capability_bits.set(capability, false);
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
