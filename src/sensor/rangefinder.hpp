#ifndef _SIM_SENSOR_RANGEFINDER_HPP_
#define _SIM_SENSOR_RANGEFINDER_HPP_

#include <sim/sim.hpp>

namespace sim {

namespace sensor {

class RangeFinder : public sim::Component {
  protected:
    sim::Sim *_sim;

    Scalar _max_range;
    size_t _num_beams;
    Scalar _angle_range;

    osg::ref_ptr<osgUtil::IntersectorGroup> _intersectors;
    osg::ref_ptr<osgUtil::IntersectionVisitor> _visitor;

    const sim::Body *_body; //!< Body sensor is attached to
    Vec3 _offset_pos;
    Quat _offset_rot;

    struct {
        bool *detected; //!< Holds info whetever obstacle was detected by beam
        Scalar *dist; //!< Distances of obstacles
        Vec3 *point; //!< Actual points of where obstacles were detected
                     //!< in global coordinates
        Vec3 *local; //!< Same as .points but in local coordinates
    } _data; //!< Measured data

    osg::ref_ptr<osg::Group> _vis;
    bool _vis_enabled;

  public:
    RangeFinder(Scalar max_range, size_t num_beams, Scalar angle_range);
    ~RangeFinder();

    size_t numBeams() const { return _num_beams; }
    Scalar angleRange() const { return _angle_range; }
    void enableVis(bool yes = true) { _vis_enabled = yes; }

    const bool *detected() const { return _data.detected; }
    bool detected(size_t i) const { return _data.detected[i]; }
    const Scalar *distance() const { return _data.dist; }
    Scalar distance(size_t i) const { return _data.dist[i]; }
    const Vec3 *point() const { return _data.point; }
    const Vec3 &point(size_t i) const { return _data.point[i]; }
    const Vec3 *localPoint() const { return _data.local; }
    const Vec3 &localPoint(size_t i) const { return _data.local[i]; }

    void setPosRot(const Vec3 &pos, const Quat &rot = Quat(0., 0., 0., 1.))
        { _offset_pos = pos; _offset_rot = rot; }

    void attachToBody(const sim::Body *b,
                      const Vec3 &pos = Vec3(0., 0., 0.),
                      const Quat &rot = Quat(0., 0., 0., 1.))
        { _body = b; _offset_pos = pos; _offset_rot = rot; }

    void init(sim::Sim *sim);
    void finish();
    void cbPreStep();

  protected:
    void _createIntersectors();
    void _createVis();
    void _updatePosition();
    void _updateVis();
};

} /* namespace sensor */

} /* namespace sim */

#endif /* _SIM_SENSOR_RANGEFINDER_HPP_ */
