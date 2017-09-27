set(PSVRD_SOURCES
    message_queue.c
    psvrd.c
)

add_executable(psvrd ${PSVRD_SOURCES})
target_link_libraries(psvrd ${LIBUSB_LIB})

add_executable(psvrd_control psvrd_control.c)
