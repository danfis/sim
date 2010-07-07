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

/**
 * Histogram of floats.
 */
class SurfHist {
  protected:
    struct _bin_t {
        float *nums;
        size_t size;
        _bin_t *next;

        _bin_t() : nums(0), size(0), next(0){}
    };

    float _binsize;
    float *_nums;
    size_t _nums_size;
    _bin_t *_bins_head, *_bins_tail;

  public:
    SurfHist(float binsize, std::list<float> &nums);
    ~SurfHist();

    /**
     * Returns horizontal difference as average value from highest bin and
     * two surrounding one.
     */
    float hd() const;
};

class SurfSegment {
  protected:
    bool _learning; //!< True if segment is learning
    bool _traversing; //!< True is segment is traversing
    float _pos_x, _pos_y; //!< Initial position of segment
    float _dist; //!< Overall traveled distance
    std::list<SurfLandmark> _tracked_lms; //!< List of tracked landmarks
    std::list<SurfLandmark> _learned_lms; //!< List of learned landmarks

  public:
    SurfSegment() : _learning(false), _traversing(false) {}
    ~SurfSegment();

    /**
     * Returns true if segment is in learning state.
     */
    bool learning() const { return _learning; }

    /**
     * Starts learning on segment.
     */
    void learnStart(float x, float y);

    /**
     * Finishes learning segment.
     */
    void learnFinish();

    /**
     * Saves learned landmarks into file.
     */
    void learnSave(const char *fn);

    /**
     * Loads learned landmarks from file.
     */
    void learnLoad(const char *fn);

    /**
     * Update segment using image and position.
     */
    void learn(const osg::Image *image, float posx, float posy);

    /**
     * Returns true if segment is traversing learned data.
     */
    bool traversing() const { return _traversing; }

    // TODO: comments
    void traverseStart(float x, float y);
    void traverseFinish();
    float traverse(const osg::Image *image, float posx, float posy);

  protected:
    /**
     * Returns distance from initial position.
     */
    float _distFromInit(float x, float y) const
        { return std::sqrt(SIM_CUBE(_pos_x - x) + SIM_CUBE(_pos_y - y)); }

    /**
     * Obtains and set up all landmarks from given image taken from
     * position posx, posy.
     */
    void _obtainLandmarks(const osg::Image *image, float posx, float posy,
                          float dist, std::vector<SurfLandmark> *lms);

    /**
     * Compares current landmarks (_lms) with tracked ones (_tracked_lms)
     * and updates _lms, _tracked_lms and _learned_lms lists.
     */
    void _trackLandmarks(std::vector<SurfLandmark> &lms, float dist);

    /**
     * Returns first and second best matching landmarks (with l) from lms
     * list via s1 and s2 pointers.
     * Returns true if two landmarks were found.
     */
    bool _bestMatching(const SurfLandmark &l, std::vector<SurfLandmark> &lms,
                       std::vector<SurfLandmark>::iterator *s1,
                       std::vector<SurfLandmark>::iterator *s2);

    /**
     * Stores in tracked all learned landmarks that were detected in
     * dist distance.
     */
    void _pickLearned(float dist, std::list<SurfLandmark> *tracked);

    /**
     * Returns histogram of horizontal differences found when
     * matching tracked landmarks with currently obtained landmarks (lms).
     */
    SurfHist *_horizontalDiffsHist(float dist, std::list<SurfLandmark> &tracked,
                                   std::vector<SurfLandmark> &lms);
};

} /* namespace alg */

} /* namespace sim */

#endif /* _SIM_ALG_SURFNAV_HPP_ */
