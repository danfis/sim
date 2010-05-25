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

    VisBody *_vis; /*!< Visual representation (for debugging). */

    // camera parameters
    Vec3 _eye, _at, _zaxis;

    // attachement to body
    const sim::Body *_body; /*!< Body camera is attached to. */
    Vec3 _body_offset_pos; /*!< Position offset of camera from body. */
    Quat _body_offset_rot; /*!< Rotation offset from body. */

  public:
    Camera();
    ~Camera();

    void init(sim::Sim *sim);
    void finish();
    void cbPreStep();
    void cbPostStep();

    // TODO
    //void setViewport
    void setLookAt(const Vec3 &eye, const Vec3 &at, const Vec3 &zaxis);

    /**
     * TODO
     */
    void attachToBody(const sim::Body *b,
                      const Vec3 &offset_pos = Vec3(0., 0., 0.),
                      const Quat &offset_rot = Quat(0., 0., 0., 1.));

  protected:
    void _createCamera();
    void _updatePosition();
};

} /* namespace sensor */

} /* namespace sim */

#endif /* _SIM_SENSOR_CAMERA_HPP_ */
