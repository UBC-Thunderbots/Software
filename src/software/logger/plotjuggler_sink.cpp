
#include "software/logger/plotjuggler_sink.h"

#include <google/protobuf/util/json_util.h>
#include "shared/constants.h"

PlotJugglerSink::PlotJugglerSink() :
    udp_sender(PLOTJUGGLER_GUI_DEFAULT_HOST, PLOTJUGGLER_GUI_DEFAULT_PORT, false)
{
}

void PlotJugglerSink::sendToPlotJuggler(g3::LogMessageMover log_entry)
{
    if (log_entry.get()._level.value == PLOTJUGGLER.value)
    {
        udp_sender.sendString(log_entry.get().message());
    }
}

std::ostream& operator<<(std::ostream& os,
                         const TbotsProto::PlotJugglerValue& plotjuggler_value)
{
    std::string json_string;

    google::protobuf::util::JsonPrintOptions options;
    options.add_whitespace = false;
    options.always_print_primitive_fields = true;
    options.preserve_proto_field_names    = true;

    MessageToJsonString(plotjuggler_value, &json_string, options);

    os << json_string;
    return os;
}
