#ifndef PLANE_H
#define PLANE_H

#define PLANE_SIZE 15

#include <sim/math.hpp>
static sim::Vec3 plane10_verts[] = {
sim::Vec3(PLANE_SIZE,PLANE_SIZE,0.000000),
sim::Vec3(-PLANE_SIZE,PLANE_SIZE,0.000000),
sim::Vec3(-PLANE_SIZE,-PLANE_SIZE,0.000000),
sim::Vec3(PLANE_SIZE,PLANE_SIZE,0.000000),
sim::Vec3(-PLANE_SIZE,-PLANE_SIZE,0.000000),
sim::Vec3(PLANE_SIZE,-PLANE_SIZE,0.000000)};

const size_t plane10_verts_len = sizeof(plane10_verts) / sizeof(sim::Vec3);

unsigned int plane10_ids[] = {
0,1,2,
3,4,5};

const size_t plane10_ids_len = sizeof(plane10_ids)/sizeof(unsigned int);

#undef PLANE_SIZE
#endif
