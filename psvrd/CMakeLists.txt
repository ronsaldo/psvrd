set(PSVRD_SOURCES
    message_queue.c
    message_queue.h
    MadgwickAHRS.c
    MadgwickAHRS.h
    MahonyAHRS.c
    MahonyAHRS.h
    psvrd.c
)

add_executable(psvrd ${PSVRD_SOURCES})
target_link_libraries(psvrd ${LIBUSB_LIB} m)

add_library(psvrd-client SHARED psvrd-client.c message_queue.c)

add_executable(psvrd_control psvrd_control.c)
target_link_libraries(psvrd_control psvrd-client)
