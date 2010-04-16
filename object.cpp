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



ObjectGroup::ObjectGroup()
    : Object()
{
    _osg_root = new osg::Group();
}

ObjectGroup::~ObjectGroup()
{
}

void ObjectGroup::applyODE()
{
    const dReal *pos, *rot;
    std::list<ode_osg>::iterator it, it_end;

    it = _objs.begin();
    it_end = _objs.end();
    for (; it != it_end; ++it){
        if (it->ode_body && it->osg_node){
            pos = dBodyGetPosition(it->ode_body);
            rot = dBodyGetQuaternion(it->ode_body);

            it->osg_node->setPosition(osg::Vec3(pos[0], pos[1], pos[2]));
            it->osg_node->setAttitude(osg::Quat(rot[1], rot[2], rot[3], rot[0]));
        }
    }
}

ObjectGroup::ode_osg &ObjectGroup::_odeOsg(size_t pos)
{
    size_t i;
    std::list<ode_osg>::iterator it, it_end;

    it = _objs.begin();
    it_end = _objs.end();
    for (i = 0; it != it_end; ++it){
        if (i == pos)
            return *it;
    }

    // it shouldn't get here
    return _objs.front();
}

osg::Node *ObjectGroup::osgNode(size_t i)
{
    if (objsSize() <= i)
        return 0;

    ode_osg &oo = _odeOsg(i);
    return oo.osg_node;
}

dBodyID ObjectGroup::odeBody(size_t i)
{
    if (objsSize() <= i)
        return 0;

    ode_osg &oo = _odeOsg(i);
    return oo.ode_body;
}

dGeomID ObjectGroup::odeGeom(size_t i)
{
    if (objsSize() <= i)
        return 0;

    ode_osg &oo = _odeOsg(i);
    return oo.ode_geom;
}


void ObjectGroup::_addObj(dBodyID b, dGeomID g, osg::Node *n)
{
    ode_osg st;

    if (n){
        st.osg_node = new osg::PositionAttitudeTransform();
        st.osg_node->addChild(n);
        _osg_root->addChild(st.osg_node);
    }else{
        st.osg_node = 0;
    }
    
    st.ode_body = b;
    st.ode_geom = g;

    _objs.push_back(st);
}


}
