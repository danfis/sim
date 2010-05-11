#ifndef _SIM_VIS_WORLD_HPP_
#define _SIM_VIS_WORLD_HPP_

#include <osgViewer/Viewer>

#include "visbody.hpp"

namespace sim {

/**
 * Visual representation of world.
 * Based on openscenegraph.
 */
class VisWorld {
    osgViewer::Viewer *_viewer;
    osg::Group *_root; /*!< Root of scene graph */
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
};

}

#endif /* _SIM_VIS_WORLD_HPP_ */
