#ifndef _SIM_ALG_SURFNAV_HPP_
#define _SIM_ALG_SURFNAV_HPP_

#include <list>
#include <cmath>
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

    bool enabled;

    SurfLandmark()
        : x(0), y(0), last_x(0), last_y(0),
          distance(0), last_distance(0),
          laplacian_sign(0), scale(0),
          visibility(0),
          enabled(true)
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

class SurfSegment {
  protected:
    float _pos_x, _pos_y; //!< Initial position of segment
    std::list<SurfLandmark> _tracked_lms; //!< List of tracked landmarks
    std::list<SurfLandmark> _learned_lms; //!< List of learned landmarks

  public:
    SurfSegment(float x, float y);
    ~SurfSegment();

    /**
     * Update segment using image and position.
     */
    void update(const osg::Image *image, float posx, float posy);

  protected:
    /**
     * Returns distance from initial position.
     */
    float _dist(float x, float y) const
        { return std::sqrt(SIM_CUBE(_pos_x - x) + SIM_CUBE(_pos_y - y)); }

    /**
     * Obtains and set up all landmarks from given image taken from
     * position posx, posy.
     */
    void _obtainLandmarks(const osg::Image *image, float posx, float posy,
                          std::vector<SurfLandmark> &lms);

    /**
     * Compares current landmarks (_lms) with tracked ones (_tracked_lms)
     * and updates _lms, _tracked_lms and _learned_lms lists.
     */
    void _trackLandmarks(std::vector<SurfLandmark> &lms);

    /**
     * Returns first and second best matching landmarks (with l) from lms
     * list via s1 and s2 pointers.
     * Returns true if two landmarks were found.
     */
    bool _bestMatching(const SurfLandmark &l, std::vector<SurfLandmark> &lms,
                       std::vector<SurfLandmark>::iterator *s1,
                       std::vector<SurfLandmark>::iterator *s2);
};

} /* namespace alg */

} /* namespace sim */

#endif /* _SIM_ALG_SURFNAV_HPP_ */
