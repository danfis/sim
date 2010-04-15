#ifndef _SIM_OBJECT_HPP_
#define _SIM_OBJECT_HPP_

#include <ode/ode.h>
#include <osg/Node>
#include <osg/Group>
#include <osg/PositionAttitudeTransform>

#include "msg.hpp"

namespace sim {

/**
 * Base virtual class of all objects.
 * Simulator knows only methods from this class.
 */
class Object {
  public:
    Object() {}
    virtual ~Object() {}

    /**
     * Returns osg root node of scene graph.
     */
    virtual osg::Node *osgRootNode() = 0;

    /**
     * This function is called right after added to simulator.
     * It should build whole object (osg part and ode part).
     */
    virtual void build(dWorldID, dSpaceID) = 0;

    /**
     * This function is called on the end of ODE simulation step.
     * It should apply all informations from ODE model to osg model.
     */
    virtual void applyODE() = 0;

    /**
     * This functio is called before each simulation step.
     * If object should be moving or anything - it should be set up here.
     */
    virtual void preODE() = 0;
};


/**
 * Simple object consisting of one body and root node of scene graph is
 * PositionAttitudeTransform node which holds transformations of whole osg
 * representation.
 * You can set osg node, body and geom by methods _setOsgNode(),
 * _setODEBody() and _setODEGeom().
 * Method applyODE() is already finished to apply transformation from
 * single body to root osg node (transformation node).
 */
class ObjectSimple : public Object {
  protected:
    osg::PositionAttitudeTransform *_osg_root;
    osg::Node *_osg_node;
    dBodyID _ode_body;
    dGeomID _ode_geom;

  public:
    ObjectSimple();
    ~ObjectSimple();

    osg::Node *osgRootNode() { return static_cast<osg::Node *>(_osg_root); }
    void build(dWorldID, dSpaceID) {}
    void applyODE();
    void preODE() {}

    osg::Node *osgNode() { return _osg_node; }
    dBodyID odeBody() { return _ode_body; }
    dGeomID odeGeom() { return _ode_geom; }

  protected:
    void _setOsgNode(osg::Node *n);
    void _setODEBody(dBodyID b) { _ode_body = b; }
    void _setODEGeom(dGeomID g) { _ode_geom = g; }
};


}

#endif /* _SIM_OBJECT_HPP */
