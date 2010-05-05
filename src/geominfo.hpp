#ifndef _SIM_GEOMINFO_HPP_
#define _SIM_GEOMINFO_HPP_

/**
 * This struct can be added to any geom using dGeomSetData() function.
 * If so, some internal functions can work with that - for more info read
 * documentation of struct's members.
 */
struct GeomInfo {
    /**
     * Two geoms are not checked for collision if both have set
     * dont_collide_id to same (non zero) number.
     */
    long dont_collide_id;

    GeomInfo() : dont_collide_id(0) {}
};

#endif /* _SIM_GEOMINFO_HPP_ */
