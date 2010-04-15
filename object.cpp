#include "object.hpp"


namespace sim {

ObjectSimple::ObjectSimple()
        : Object(),
          _osg_root(0),
          _osg_node(0),
          _ode_body(0),
          _ode_geom(0)
{
    _osg_root = new osg::PositionAttitudeTransform();
}

ObjectSimple::~ObjectSimple()
{
}

void ObjectSimple::applyODE()
{
    const dReal *pos, *rot;
    dBodyID body = odeBody();
    osg::PositionAttitudeTransform *root = (osg::PositionAttitudeTransform *)osgRootNode();

    if (!body || !root)
        return;

    pos = dBodyGetPosition(body);
    rot = dBodyGetQuaternion(body);

    root->setPosition(osg::Vec3(pos[0], pos[1], pos[2]));
    root->setAttitude(osg::Quat(rot[1], rot[2], rot[3], rot[0]));
}


void ObjectSimple::_setOsgNode(osg::Node *n)
{
    _osg_node = n;
    _osg_root->addChild(n);
}

}
