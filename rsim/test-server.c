#include <dbg.h>
#include "rsim_server.h"

static void cb1(const rsim_msg_t *msg, int sock, void *data);

int main(int argc, char *argv[])
{
    rsim_server_t *s;

    s = rsimServerNew();
    rsimServerRegister(s, 1, cb1, NULL);
    rsimServerStart(s, 9876);
    rsimServerDel(s);

    return 0;
}

static void cb1(const rsim_msg_t *msg, int sock, void *data)
{
    fprintf(stderr, "cb1: %lx, type: %d\n", (long)msg, (int)msg->type);
    if (msg->type == RSIM_MSG_PING){
        rsimMsgSendPong(sock);
    }
}
