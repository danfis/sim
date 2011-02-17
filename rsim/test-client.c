#include <stdio.h>
#include <stdlib.h>
#include "rsim.h"

int main(int argc, char *argv[])
{
    rsim_client_t c;
    int ret;
    rsim_msg_t *msg;

    ret = rsimClientConnect(&c, "127.0.0.1", 9876, 1);
    if (ret != 0){
        fprintf(stderr, "Can't connect to server.\n");
        return -1;
    }

    rsimMsgSendPing(c.sock);
    msg = rsimClientNextMsg(&c);
    fprintf(stderr, "type: %d\n", (int)msg->type);
    free(msg);

    rsimClientClose(&c);

    return 0;
}
