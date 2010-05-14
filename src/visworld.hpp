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
    VisWorld();
    virtual ~VisWorld();

    bool window() const { return _window; }
    void setWindow(bool yes = true) { _window = yes; }

    /**
     * Returns root of scene graph.
     */
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

  protected:
    void _setUpStateSet();
    void _setUpLights();
};

}

#endif /* _SIM_VIS_WORLD_HPP_ */
