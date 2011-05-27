#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "camera.h"

camera_t *cameraNew(const char *dir)
{
    camera_t *cam;
    size_t len, memsize;

    // compute needed memory for all camera members (not image)
    len = strlen(dir);
    memsize  = sizeof(camera_t);
    memsize += len + 1;
    memsize += len + 20 + 1;

    // allocate memory for camera as one big block
    cam = (camera_t *)malloc(memsize);
    cam->dir = ((char *)cam) + sizeof(camera_t);
    cam->buf_path = ((char *)cam->dir) + len + 1;
    cam->next = 0;

    // copy directory to .dir member
    memcpy(cam->dir, dir, len + 1);

    cam->image = imageNew(640, 480);

    return cam;
}

void cameraDel(camera_t *cam)
{
    imageDel(cam->image);
    free(cam);
}

int cameraNextImage(camera_t *cam)
{
    sprintf(cam->buf_path, "%s/%04d.bmp", cam->dir, (int)cam->next);
    cam->next++;
    return imageLoadBmp(cam->image, cam->buf_path);
}
