/***
 * Remote sim
 * -----------
 * Copyright (c)2011 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _SIM_RSIM_H_
#define _SIM_RSIM_H_

#include <stdint.h>
#include <netinet/in.h>

#define RSIM_BUFSIZE 4096
//#define RSIM_BUFSIZE 1

#define RSIM_MSG_PING          1
#define RSIM_MSG_PONG          2
#define RSIM_MSG_GET_POS       3
#define RSIM_MSG_POS           4
#define RSIM_MSG_GET_ROT       5
#define RSIM_MSG_ROT           6
#define RSIM_MSG_SET_VEL_LEFT  7
#define RSIM_MSG_SET_VEL_RIGHT 8
#define RSIM_MSG_GET_RF        9
#define RSIM_MSG_RF            10
#define RSIM_MSG_GET_IMG       11
#define RSIM_MSG_IMG           12

struct _rsim_msg_t {
    uint16_t id;
    char type;
};
typedef struct _rsim_msg_t rsim_msg_t;

/**
 * Message with one additional float.
 */
struct _rsim_msg_float_t {
    uint16_t id;
    char type;
    float f;
};
typedef struct _rsim_msg_float_t rsim_msg_float_t;

/**
 * Message with three floats.
 */
struct _rsim_msg_float3_t {
    uint16_t id;
    char type;
    float f[3];
};
typedef struct _rsim_msg_float3_t rsim_msg_float3_t;

/**
 * Message with four floats.
 */
struct _rsim_msg_float4_t {
    uint16_t id;
    char type;
    float f[4];
};
typedef struct _rsim_msg_float4_t rsim_msg_float4_t;

/**
 * Message with any number of floats.
 */
struct _rsim_msg_floats_t {
    uint16_t id;
    char type;
    size_t flen;
    float *f;
};
typedef struct _rsim_msg_floats_t rsim_msg_floats_t;

/**
 * Message with image width * height, in .data are stored triplets rgb.
 */
struct _rsim_msg_img_t {
    uint16_t id;
    char type;
    size_t width, height;
    unsigned char *data;
};
typedef struct _rsim_msg_img_t rsim_msg_img_t;



struct _rsim_t {
    int sock;

    char buf[RSIM_BUFSIZE];
    char *bufstart, *bufend;
    rsim_msg_t *msg;
};
typedef struct _rsim_t rsim_t;

/**
 * Connects client to specified server:port and id of robot.
 * Returns 0 on success, -1 if server is not reachable or other client is
 * already registered to specified robot.
 */
int rsimConnect(rsim_t *, const char *addr, uint16_t port);

/**
 * Close previously established connection to server.
 */
void rsimClose(rsim_t *);

/**
 * Returns true if there are some pending data.
 */
int rsimHaveMsg(rsim_t *);

/**
 * Returns next message sent from server.
 * This is blocking call.
 * Returns NULL if connection failed or was closed or some invalid data
 * were read.
 */
const rsim_msg_t *rsimNextMsg(rsim_t *);


/**
 * Sends simple message.
 */
int rsimSendSimple(rsim_t *, uint16_t id, char type);

/**
 * Sends message with one additional float.
 */
int rsimSendFloat(rsim_t *, uint16_t id, char type, float f);

#endif /* _SIM_RSIM_H_ */
