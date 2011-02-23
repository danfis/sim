#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "rsim.h"

// note: 640x480 image
unsigned char rgbHeader[] = {66,77,54,16,14,0,0,0,0,0,54,0,0,0,40,0,0,0,128,2,0,0,224,1,0,0,1,0,24,0,0,0,0,0,0,16,14,0,18,11,0,0,18,11,0,0,0,0,0,0,0,0,0,0};


int main(int argc, char *argv[])
{
    rsim_t c;
    int ret;
    size_t i, j, k;
    const rsim_msg_t *msg;
    const rsim_msg_float3_t *msgf3;
    const rsim_msg_float4_t *msgf4;
    const rsim_msg_floats_t *msgfs;
    const rsim_msg_img_t *msgimg;
    FILE *fout;

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

    rsimSendFloat(&c, 10, RSIM_MSG_SET_VEL_LEFT, 1.234);
    rsimSendFloat(&c, 10, RSIM_MSG_SET_VEL_RIGHT, 10.111);

    rsimSendSimple(&c, 10, RSIM_MSG_GET_POS);
    rsimSendSimple(&c, 10, RSIM_MSG_GET_ROT);
    msg = rsimNextMsg(&c);
    if (msg && msg->type == RSIM_MSG_POS){
        msgf3 = (rsim_msg_float3_t *)msg;
        fprintf(stderr, "POS: id: %d, type: %d, %g %g %g\n",
                (int)msg->id, (int)msg->type,
                msgf3->f[0], msgf3->f[1], msgf3->f[2]);
    }

    msg = rsimNextMsg(&c);
    if (msg && msg->type == RSIM_MSG_ROT){
        msgf4 = (rsim_msg_float4_t *)msg;
        fprintf(stderr, "POS: id: %d, type: %d, %g %g %g %g\n",
                (int)msg->id, (int)msg->type,
                msgf4->f[0], msgf4->f[1], msgf4->f[2], msgf4->f[3]);
    }

    rsimSendSimple(&c, 10, RSIM_MSG_GET_RF);
    msg = rsimNextMsg(&c);
    if (msg && msg->type == RSIM_MSG_RF){
        msgfs = (rsim_msg_floats_t *)msg;

        fprintf(stderr, "RF: id: %d, type: %d, dist: ", (int)msgfs->id, (int)msgfs->type);
        for (i = 0; i < msgfs->flen; i++){
            fprintf(stderr, " %g", msgfs->f[i]);
        }
        fprintf(stderr, "\n");
    }else{
        fprintf(stderr, "Wrong answer to GET_RF\n");
    }

    rsimSendSimple(&c, 10, RSIM_MSG_GET_IMG);
    msg = rsimNextMsg(&c);
    if (msg && msg->type == RSIM_MSG_IMG){
        msgimg = (rsim_msg_img_t *)msg;

        fout = fopen("image.bmp", "w");
		fwrite(rgbHeader, 54, 1, fout);

        //fprintf(stderr, "IMG: id: %d, type: %d, data: \n", (int)msgimg->id, (int)msgimg->type);
        k = 0;
        for (i = 0; i < msgimg->width; i++){
            //fprintf(stderr, "[%03d]:", i);
            for (j = 0; j < msgimg->height; j++){
                /*
                fprintf(stderr, " (%d, %d, %d)",
                        (int)msgimg->data[k],
                        (int)msgimg->data[k + 1],
                        (int)msgimg->data[k + 2]);
                */

                fwrite(msgimg->data + k, 1, 3, fout);

                k += 3;
            }
            //fprintf(stderr, "\n");
        }

        fclose(fout);
    }


    rsimClose(&c);

    return 0;
}
