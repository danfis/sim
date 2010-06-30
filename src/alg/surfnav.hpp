#ifndef _SIM_ALG_SURFNAV_HPP_
#define _SIM_ALG_SURFNAV_HPP_

#include <cv.h>
#include <osg/Image>
#include <sim/math.hpp>

namespace sim {

namespace alg {

class SurfLandmark {
  public:
    float x, y; //!< Position when detected
    float last_x, last_y; //!< Last position of matched Landmark

    float distance;
    float last_distance;

    int laplacian_sign;
    float scale;
    float orientation; //!< Orientation measured anti-clockwise from +ve x-axis

    double desc[64]; //!< 64 description numbers

    int visibility;

    SurfLandmark()
        : x(0), y(0), last_x(0), last_y(0),
          distance(0), last_distance(0),
          laplacian_sign(0), scale(0),
          visibility(0)
    {}
    ~SurfLandmark() {}

    /**
     * Returns distance from other landmark (difference between desriptors).
     */
    double dist(const SurfLandmark &l) const
    {
        double d = 0.;
        for (size_t i = 0; i < 64; i++)
            d += SIM_CUBE(desc[i] - l.desc[i]);
        return std::sqrt(d);
    }
};

class SurfNav {
  public:
    SurfNav();
    ~SurfNav();

    void update(const osg::Image *image, float posx, float posy);
};

} /* namespace alg */

} /* namespace sim */

#endif /* _SIM_ALG_SURFNAV_HPP_ */
