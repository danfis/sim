#ifndef _SIM_VIS_WORLD_HPP_
#define _SIM_VIS_WORLD_HPP_

#include <osgViewer/Viewer>

#include "visobj.hpp"

namespace sim {

/**
 * Visualised world.
 * Based on openscenegraph.
 */
class VisWorld {
    osgViewer::Viewer *_viewer;
    osg::Group *_root; /*! root of scene graph */

  public:
    VisWorld();
    virtual ~VisWorld();

    /**
     * Adds visualisable object.
     */
    void addObj(VisObj *obj);

    /**
     * Returns root of scene graph.
     */
    osg::Node *sceneRoot() { return _root; }

    /**
     * Initializes world.
     */
    void init();

    /**
     * Destroys world.
     */
    void destroy();

    /**
     * Performes one step.
     */
    void step();

    bool done();
};

}

#endif /* _SIM_VIS_WORLD_HPP_ */
