#ifndef _SIM_SENSOR_CAMERA_HPP_
#define _SIM_SENSOR_CAMERA_HPP_

#include <sim/sim.hpp>
#include <sim/body.hpp>
#include <sim/component.hpp>

namespace sim {

namespace sensor {

class Camera : public sim::Component {
  protected:
    Sim *_sim;

    osg::ref_ptr<osg::Camera> _cam;
    osg::ref_ptr<osg::Image> _image;

    // camera parameters
    Vec3 _eye, _at, _zaxis;
    const sim::Body *_body;

  public:
    Camera();
    ~Camera();

    void init(sim::Sim *sim);
    void finish();
    void cbPreStep();

    // TODO
    //void setViewport
    void setLookAt(const Vec3 &eye, const Vec3 &at, const Vec3 &zaxis);
    void attachToBody(const sim::Body *b);

  protected:
    void _createCamera();
    void _updatePosition();
};

} /* namespace sensor */

} /* namespace sim */

#endif /* _SIM_SENSOR_CAMERA_HPP_ */
