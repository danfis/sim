#include "rsim_server.h"

int main(int argc, char *argv[])
{
    rsim_server_t *s;

    s = rsimServerNew();
    rsimServerStart(s, 9876);
    rsimServerDel(s);

    return 0;
}
