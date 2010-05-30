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

  public:
    RangeFinder(Scalar max_range, size_t num_beams, Scalar angle_range);
    ~RangeFinder();

    size_t numBeams() const { return _num_beams; }
    Scalar angleRange() const { return _angle_range; }

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
    void _updatePosition();
};

} /* namespace sensor */

} /* namespace sim */

#endif /* _SIM_SENSOR_RANGEFINDER_HPP_ */
