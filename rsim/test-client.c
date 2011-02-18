#include <stdio.h>
#include <stdlib.h>
#include "rsim.h"

int main(int argc, char *argv[])
{
    rsim_t c;
    int ret;
    const rsim_msg_t *msg;

    ret = rsimConnect(&c, "127.0.0.1", 9876);
    if (ret != 0){
        fprintf(stderr, "Can't connect to server.\n");
        return -1;
    }

    rsimSendSimple(&c, 10, RSIM_MSG_PING);
    rsimSendSimple(&c, 12, RSIM_MSG_PING);
    rsimSendSimple(&c, 13, RSIM_MSG_PING);
    msg = rsimNextMsg(&c);
    if (msg){
        fprintf(stderr, "id: %d, type: %d\n", (int)msg->id, (int)msg->type);
    }
    msg = rsimNextMsg(&c);
    if (msg){
        fprintf(stderr, "id: %d, type: %d\n", (int)msg->id, (int)msg->type);
    }
    msg = rsimNextMsg(&c);
    if (msg){
        fprintf(stderr, "id: %d, type: %d\n", (int)msg->id, (int)msg->type);
    }


    rsimClose(&c);

    return 0;
}
