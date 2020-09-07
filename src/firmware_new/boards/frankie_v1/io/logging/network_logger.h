#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "shared/proto/tbots_log.nanopb.h"

void tbots_log(TbotsProto_LogLevel log_level, const char* log_msg);
