#include <psvrd.h>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libusb.h>

#include <sys/epoll.h>

static int socketHandle;

int main(int argc, const char *argv[])
{
    /* Create the socket. */
    socketHandle = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    if(socketHandle < 0)
    {
       perror("Failed to create an Unix domain socket.\n");
       return 1;
    }

    /* Connect to the server. */
    struct sockaddr_un address;
    memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    strcpy(address.sun_path, PSVR_SOCKET_LOCATION);
    int error = connect(socketHandle, (const struct sockaddr *)&address, sizeof(address));
    if(error)
    {
        perror("Failed to connect to the psvrd daemon");
        return 1;
    }

    /* Close the socket. */
    shutdown(socketHandle, SHUT_RDWR);
    close(socketHandle);
    return 0;
}
