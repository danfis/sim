#include <stdio.h>
#include <unistd.h>


int main(int argc, char *argv[])
{
    int size;
    size = getpagesize();
    fprintf(stdout, "PAGESIZE is %d\n", size);
    return 0;
}
