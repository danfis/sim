#ifndef _CAMERA_H_
#define _CAMERA_H_

#include "image.h"

struct _camera_t {
    size_t next; /*! number of next image in dir */
    char *dir; /*! directory with stored images */
    char *buf_path; /*! buffer for storing path to image */

    image_t *image;
};
typedef struct _camera_t camera_t;

camera_t *cameraNew(const char *dir);
void cameraDel(camera_t *);

static inline image_t *cameraImage(camera_t *c)
{
    return c->image;
}

/**
 * Load next image from camera.
 * Returns 0 on success.
 */
int cameraNextImage(camera_t *);

#endif /* _CAMERA_H_ */
