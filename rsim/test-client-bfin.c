#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include "rsim.h"

// note: 640x480 image
unsigned char rgbHeader[] = {66,77,54,16,14,0,0,0,0,0,54,0,0,0,40,0,0,0,128,2,0,0,224,1,0,0,1,0,24,0,0,0,0,0,0,16,14,0,18,11,0,0,18,11,0,0,0,0,0,0,0,0,0,0};

#define ID 10
#define SLEEPTIME 100000
#define CUBE(x) ((x) * (x))

float findMin(float *f, size_t len)
{
    float m;
    size_t i;

    m = f[0];
    for (i = 1; i < len; i++){
        if (f[i] < m)
            m = f[i];
    }

    return m;
}

int main(int argc, char *argv[])
{
    rsim_t c;
    int ret;
    float initpos[3];
    float dist, minl, minr, w, vl, vr;
    const rsim_msg_t *msg;
    const rsim_msg_float3_t *msgf3;
    const rsim_msg_float4_t *msgf4;
    size_t i, len;
    const rsim_msg_floats_t *msgfs;
#if 0
    const rsim_msg_img_t *msgimg;
    FILE *fout;
    size_t i, j, k;
#endif

    //ret = rsimConnect(&c, "127.0.0.1", 9876);
    ret = rsimConnect(&c, "147.32.85.87", 9876);
    if (ret != 0){
        fprintf(stderr, "Can't connect to server.\n");
        return -1;
    }

    rsimSendSimple(&c, ID, RSIM_MSG_GET_POS);
    msg = rsimNextMsg(&c);
    if (msg && msg->type == RSIM_MSG_POS){
        msgf3 = (rsim_msg_float3_t *)msg;
        initpos[0] = msgf3->f[0];
        initpos[1] = msgf3->f[1];
        initpos[2] = msgf3->f[2];
        fprintf(stderr, "INITPOS: id: %d, type: %d, %g %g %g\n",
                (int)msg->id, (int)msg->type,
                msgf3->f[0], msgf3->f[1], msgf3->f[2]);
    }

    rsimSendFloat(&c, ID, RSIM_MSG_SET_VEL_LEFT, 0.4);
    rsimSendFloat(&c, ID, RSIM_MSG_SET_VEL_RIGHT, 0.4);
    while (1){
        rsimSendSimple(&c, ID, RSIM_MSG_GET_RF);
        msg = rsimNextMsg(&c);
        if (msg && msg->type == RSIM_MSG_RF){
            msgfs = (rsim_msg_floats_t *)msg;

            len = msgfs->flen / 2;
            minr = findMin(msgfs->f, len);
            minl = findMin(msgfs->f + msgfs->flen - len, len);

            fprintf(stderr, "minl, minr: %f %f\n", minl, minr);
        }else{
            fprintf(stderr, "Wrong answer to GET_RF\n");
        }

        w = (minr - minl) / 10;
        vl = -0.8 + w;
        vr = -0.8 - w;
        fprintf(stderr, "w, vl, vr: %f %f %f\n", w, vl, vr);
        rsimSendFloat(&c, ID, RSIM_MSG_SET_VEL_LEFT, vl);
        rsimSendFloat(&c, ID, RSIM_MSG_SET_VEL_RIGHT, vr);

        usleep(SLEEPTIME);
    }

    rsimClose(&c);

    return 0;
}
