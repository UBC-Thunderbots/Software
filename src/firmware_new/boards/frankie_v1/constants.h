#pragma once

// This file contains all constants that are shared between our software (AI)
// and firmware code. Since this needs to be compiled by both C and C++, everything
// should be defined in a way that's compatible with C.
//
// NOTE: This file is here until its moved to the shared folder after the new firmware
// is setup
const char* AI_MULTICAST_ADDRESS = "ff02::c3d0:42d2:bb8";
const unsigned RECEIVE_PORT      = 42000;
const unsigned SEND_PORT         = 42001;
