#ifndef _SIM_VIS_WORLD_HPP_
#define _SIM_VIS_WORLD_HPP_

#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>

#include "visbody.hpp"

namespace sim {

/**
 * Visual representation of world.
 * Based on openscenegraph.
 */
class VisWorld {
    osg::ref_ptr<osgViewer::Viewer> _viewer;
    osg::ref_ptr<osg::StateSet> _state_set;
    osg::ref_ptr<osg::Group> _root; /*!< Root of scene graph */
    osg::ref_ptr<osg::Group> _lights;
    bool _window; /*!< Show a window? */

  public:
    osg::ref_ptr<osg::Group> _r;
    osg::ref_ptr<osg::Camera> _cam;
    osg::ref_ptr<osg::Image> _image;
  public:
    VisWorld();
    virtual ~VisWorld();

    osgViewer::Viewer *viewer() { return _viewer.get(); }
    const osgViewer::Viewer *viewer() const { return _viewer.get(); }
    bool window() const { return _window; }
    void setWindow(bool yes = true) { _window = yes; }

    /**
     * Returns root of scene graph.
     */
    //osg::Node *sceneRoot() { return _root; }
    osg::Node *sceneRoot() { return _root; }

    /**
     * Adds visualisable object.
     */
    void addBody(VisBody *obj);

    /**
     * Initializes world.
     */
    void init();

    /**
     * Finalize world.
     */
    void finish();

    /**
     * Performes one step.
     */
    void step();

    bool done();

    void toggleWireframe();

  protected:
    void _setUpStateSet();
    void _setUpLights();
};

}

#endif /* _SIM_VIS_WORLD_HPP_ */
