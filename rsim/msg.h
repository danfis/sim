/***
 * sim
 * ---------------------------------
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

#ifndef _SIM_RSIM_MSG_H_
#define _SIM_RSIM_MSG_H_

#include <stdint.h>

#define RSIM_MSG_PING 0
#define RSIM_MSG_PONG 1

struct _rsim_msg_t {
    uint16_t from;
    uint16_t to;
    char type;
};
typedef struct _rsim_msg_t rsim_msg_t;

#endif /* _SIM_RSIM_MSG_H_ */
