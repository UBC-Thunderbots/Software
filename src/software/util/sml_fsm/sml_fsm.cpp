#include "software/util/sml_fsm/sml_fsm.h"

std::string stripFSMState(std::string s)
{
    auto pos = s.find_last_of(":<");
    if (pos != std::string::npos)
    {
        s = s.substr(pos + 1);
    }

    pos = s.find_first_of('>');
    if (pos != std::string::npos)
    {
        return s.substr(0, pos);
    }
    else
    {
        return s;
    }
}
