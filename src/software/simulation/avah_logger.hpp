#pragma once

#include <google/protobuf/text_format.h>

#include <fstream>

#include "software/logger/logger.h"

namespace avah
{

inline std::string path;
inline int i{0};

[[maybe_unused]] inline void set_path(const std::string& p)
{
    path = p;
    std::ofstream out{path};
    out.close();
}

[[maybe_unused]] inline void AVAH_LOG(auto x)
{
    std::ofstream out{path, std::ios::app};
    out << i++ << ": " << x << "\n\n";
    out.close();
}

[[maybe_unused]] inline void AVAH_LOG_PROTO(auto x)
{
    std::string text;
    if (google::protobuf::TextFormat::PrintToString(x, &text))
    {
        AVAH_LOG(text);
    }
    else
    {
        LOG(FATAL) << "COULDNT LOG PROTO";
    }
}

}  // namespace avah
