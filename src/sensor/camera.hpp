/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
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

#ifndef _SIM_SENSOR_CAMERA_HPP_
#define _SIM_SENSOR_CAMERA_HPP_

#include <sim/sim.hpp>
#include <sim/body.hpp>
#include <sim/component.hpp>

namespace sim {

namespace sensor {

/**
 * Camera sensor that can be static or attached to any Body.
 * If you want to create static Camera simply use setLookAt() method to set
 * up camera view matrix.
 * If you want to attach camera to Body use attachToBody() method that
 * takes pointer to Body and position and rotation offsets. By default
 * camera is placed at (0, 0, 0) position, camera is facing in (1, 0, 0)
 * direction and z-axis is (0, 0, 1) - all offsets are relative to these
 * defaults.
 */
class Camera : public sim::Component {
  protected:
    Sim *_sim;

    osg::ref_ptr<osg::Camera> _cam; /*!< Camera node */
    osg::ref_ptr<osg::Image> _image; /*!< Image where is stored what camera took */
    VisBody *_vis; /*!< Visual representation (for debugging). */
    bool _vis_enabled; /*!< Is visual representation enabled? (default false) */

    Vec3 _eye; /*!< Position of camera */
    Vec3 _at; /*!< Where is camera pointing */
    Vec3 _zaxis; /*! Where is zaxis of camera */
    osg::Vec4 _bgcolor; /*!< Background color (default blue) */
    int _width, _height; /*!< Width and height of taken pictures (default 100x100) */

    const sim::Body *_body; /*!< Body camera is attached to. */
    Vec3 _body_offset_pos; /*!< Position offset of camera from body. */
    Quat _body_offset_rot; /*!< Rotation offset from body. */

    const char *_dump_prefix; /*!< Prefix of files where camera will dump
                                   all images. If prefix is zero dumping is
                                   disabled */

    osg::ref_ptr<osgViewer::View> _view;
    bool _view_enabled; /*!< By default view is disabled */

  public:
    Camera();
    ~Camera();

    osg::Camera *cam() { return _cam.get(); }
    const osg::Camera *cam() const { return _cam.get(); }
    osg::Image *image() { return _image.get(); }
    const osg::Image *image() const { return _image.get(); }
    VisBody *vis() { return _vis; }
    const VisBody *visBody() const { return _vis; }
    bool visBodyEnabled() const { return _vis_enabled; }

    /**
     * Enables visual representation of camera.
     */
    void visBodyEnable(bool enable = true) { _vis_enabled = enable; }

    const Vec3 &eye() const { return _eye; }
    const Vec3 &at() const { return _at; }
    const Vec3 &zaxis() const { return _zaxis; }
    const osg::Vec4 &bgColor() const { return _bgcolor; }
    int width() const { return _width; }
    int height() const { return _height; }
    const char *dumpPrefix() const { return _dump_prefix; }

    /**
     * Sets view matrix of camera.
     * See doc to gluLookAt() function for better understanding.
     */
    void setLookAt(const Vec3 &eye, const Vec3 &at,
                   const Vec3 &zaxis = Vec3(0., 0., 1.))
        { _eye = eye; _at = at; _zaxis = zaxis; }

    /**
     * Sets background color.
     */
    void setBgColor(const osg::Vec4 &c) { _bgcolor = c; }
    void setBgColor(Scalar r, Scalar g, Scalar b, Scalar a)
        { _bgcolor = osg::Vec4(r, g, b, a); }

    /**
     * Sets width and height of viewport.
     */
    void setWidthHeight(int w, int h) { _width = w; _height = h; }

    /**
     * Enables dumping images to .png files.
     */
    void enableDump(const char *prefix = "") { _dump_prefix = prefix; }
    void disableDump() { _dump_prefix = 0; }

    /**
     * Enables/Disables view window.
     */
    void enableView(bool enable = true) { _view_enabled = enable; }


    /**
     * Attach camera to body with specified position and rotation offsets.
     * If attached to body camera is moving with body.
     * By default camera is on position (0., 0., 0.) facing in direction
     * (1., 0., 0.) and z-axis is (0., 0., 1.) - both offsets are relative
     * to these values.
     */
    void attachToBody(const sim::Body *b,
                      const Vec3 &offset_pos = Vec3(0., 0., 0.),
                      const Quat &offset_rot = Quat(0., 0., 0., 1.));
    void dettachFromBody() { _body = 0; }
    const sim::Body *attachedBody() const { return _body; }

    /**
     * Position offset of attached body.
     */
    const Vec3 &attachedBodyPosOffset() const { return _body_offset_pos; }

    /**
     * Rotation offset of attached body.
     */
    const Quat &attachedBodyRotOffset() const { return _body_offset_rot; }

    void init(sim::Sim *sim);
    void finish();
    void cbPreStep();
    void cbPostStep();

  protected:
    /**
     * Creates camera according to previously set parameters.
     */
    void _createCamera();

    /**
     * Creates view.
     */
    void _createView();

    /**
     * Update position of camera (and visual representation).
     * If camera is attached to body all transformations are taken from
     * body and _eye, _at, _zaxis is set properly.
     */
    void _updatePosition();
};

} /* namespace sensor */

} /* namespace sim */

#endif /* _SIM_SENSOR_CAMERA_HPP_ */
