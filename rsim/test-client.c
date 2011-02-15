#include <stdio.h>
#include "rsim.h"

int main(int argc, char *argv[])
{
    rsim_client_t c;
    int ret;

    ret = rsimClientConnect(&c, "127.0.0.1", 9876, 1);
    if (ret != 0){
        fprintf(stderr, "Can't connect to server.\n");
        return -1;
    }

    rsimClientClose(&c);

    return 0;
}

