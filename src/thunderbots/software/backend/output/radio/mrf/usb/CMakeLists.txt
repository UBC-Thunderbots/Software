# Find libusb
pkg_search_module("LIBUSB_1" REQUIRED "libusb-1.0")

add_library(backend_radio_output_mrf_usb STATIC
    bulktransfer.cpp
    controltransfer.cpp
    device.cpp
    devicehandle.cpp
    errors.cpp
    interrupttransfer.cpp
    tbots_libusb.cpp
    misc.cpp
    transfer.cpp
    )
target_include_directories(backend_radio_output_mrf_usb
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    PRIVATE
        ${LIBUSB_1_INCLUDE_DIRS}
    )
target_link_libraries(backend_radio_output_mrf_usb
    PRIVATE
        ${LIBUSB_1_LIBRARIES}
        backend_radio_output_mrf_util
    )
target_compile_definitions(backend_radio_output_mrf_usb
    PRIVATE
        ${LIBUSB_1_DEFINITIONS}
    )
