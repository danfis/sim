#include <stdio.h>
#include <stdlib.h>
#include "rsim.h"

int main(int argc, char *argv[])
{
    rsim_t c;
    int ret;
    const rsim_msg_t *msg;

    ret = rsimConnect(&c, "127.0.0.1", 9876, 1);
    if (ret != 0){
        fprintf(stderr, "Can't connect to server.\n");
        return -1;
    }

    rsimSendPing(&c);
    msg = rsimNextMsg(&c);
    fprintf(stderr, "type: %d\n", (int)msg->type);

    rsimClose(&c);

    return 0;
}
