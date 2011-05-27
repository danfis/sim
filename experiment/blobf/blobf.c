#include <stdlib.h>
#include <stdio.h>
#include "camera.h"
#include "finder.h"
#include "stopwatch.h"

void usage(const char *prog)
{
    fprintf(stderr, "Usage: %s path\n", prog);
    exit(-1);
}

int main(int argc, char *argv[])
{
    camera_t *cam;
    segment_t seg;
    finder_t *finder;
    int running;
    stopwatch_t stopwatch;

    if (argc != 2){
        usage(argv[0]);
    }

    stopwatchStart(&stopwatch);

    cam = cameraNew(argv[1]);
    finder = finderNew(640, 480);

    stopwatchStop(&stopwatch);
    fprintf(stderr, "cameraNew(), finderNew() time: %ld us\n", stopwatchElapsedUs(&stopwatch));


    stopwatchStart(&stopwatch);

    cameraNextImage(cam);

    stopwatchStop(&stopwatch);
    fprintf(stderr, "cameraNextImage() time: %ld us\n", stopwatchElapsedUs(&stopwatch));


    stopwatchStart(&stopwatch);

    finderAddPixel(finder, imageGet(cam->image, 218, 211));

    stopwatchStop(&stopwatch);
    fprintf(stderr, "finderAddPixel() time: %ld us\n", stopwatchElapsedUs(&stopwatch));

    running = 1;
    while (running){
        stopwatchStart(&stopwatch);

        seg = finderFindSegment(finder, cam->image);
        fprintf(stderr, "x: %d, y: %d, size: %d\n", seg.x, seg.y, seg.size);

        stopwatchStop(&stopwatch);
        fprintf(stderr, "finderFindSegment() time: %ld us\n", stopwatchElapsedUs(&stopwatch));

        if (cameraNextImage(cam) != 0)
            running = 0;
    }


    cameraDel(cam);
    finderDel(finder);

    return 0;
}
