#include <osgGA/TrackballManipulator>

#include "sim.hpp"
#include "geominfo.hpp"
#include "msg.hpp"

namespace sim {

class SimKeyboard : public osgGA::GUIEventHandler {
    Sim *_sim;
  public:
    SimKeyboard(Sim *sim) : _sim(sim){}
    virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);
    virtual void accept(osgGA::GUIEventHandlerVisitor& v) { v.visit(*this); };
};

bool SimKeyboard::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    int key;

    if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN){
        key = ea.getKey();
        return _sim->pressedKey(key);
    }
    return false;
}

Sim::Sim()
    : _sim_time(0.0001)
{
    _viewer = new osgViewer::Viewer();
    _root = new osg::Group();

    dInitODE2(0);

    _world = dWorldCreate();
    _space = dSimpleSpaceCreate(0);
    _coll_contacts = dJointGroupCreate(0);

    dWorldSetERP(_world, 0.5);
    dWorldSetCFM(_world, 0.001);
    dWorldSetGravity(_world, 0., 0., -9.81);

    _default_contact.surface.slip1 = 0.7;
    _default_contact.surface.slip2 = 0.7;
    _default_contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactSlip1 | dContactSlip2;
    _default_contact.surface.mu = 10.0; // was: dInfinity
    _default_contact.surface.soft_erp = 0.96;
    _default_contact.surface.soft_cfm = 0.04;
}

Sim::~Sim()
{
    std::list<Object *>::iterator it, it_end;

    if (_viewer)
        delete _viewer;

    if (_world)
        dWorldDestroy(_world);
    if (_space)
        dSpaceDestroy(_space);
    if (_coll_contacts)
        dJointGroupDestroy(_coll_contacts);

    it = _objs.begin();
    it_end = _objs.end();
    for (; it != it_end; ++it){
        delete *it;
    }
    _objs.clear();
}


void Sim::addObject(Object *o)
{
    osg::Node *node;

    _objs.push_back(o);

    o->build(_world, _space);

    node = o->osgRootNode();
    if (node){
        _root->addChild(node);
        o->applyODE();
    }
}

void Sim::run()
{
    size_t counter = 0, steps = 5, i;
    double stepsize = 0.01, ss;
    std::list<Object *>::iterator it, it_end;

    _viewer->setSceneData(_root);

    if (!_viewer->getCameraManipulator()){
        DBG("Adding camera manipulator");
        _viewer->setCameraManipulator(new osgGA::TrackballManipulator());
    }

    // register keyboard listener
    SimKeyboard *keyboard = new SimKeyboard(this);
    _viewer->addEventHandler(keyboard);

    _viewer->realize();
    while (!_viewer->done()){
        if (counter % 1 == 0){
            //DBG("Simulate");
            it_end = _objs.end();

            ss = stepsize / steps;
            for (i = 0; i < steps; i++){
                it = _objs.begin();
                for (; it != it_end; ++it){
                    (*it)->preODE();
                }

                dSpaceCollide(_space, this, __ode_near_collision);
                dWorldStep(_world, ss);
                dJointGroupEmpty(_coll_contacts);
            }

            it = _objs.begin();
            for (; it != it_end; ++it){
                (*it)->applyODE();
            }
        }


        _viewer->frame(_sim_time);

        counter++;
    }
}

void __ode_near_collision(void *data, dGeomID o1, dGeomID o2)
{
    Sim *sim = (Sim *)data;
    GeomInfo *info1, *info2;

    info1 = (GeomInfo *)dGeomGetData(o1);
    info2 = (GeomInfo *)dGeomGetData(o2);

    if (info1 && info2
            && info1->dont_collide_id
            && info2->dont_collide_id
            && info1->dont_collide_id == info2->dont_collide_id){
        return;
    }

    if (dGeomIsSpace(o1) || dGeomIsSpace(o2)){
        dSpaceCollide2(o1, o2, data, &__ode_near_collision);

        if (dGeomIsSpace (o1)) dSpaceCollide((dSpaceID)o1, data, &__ode_near_collision);
        if (dGeomIsSpace (o2)) dSpaceCollide((dSpaceID)o2, data, &__ode_near_collision);

        return;
    }

    const int N = 32;
    dContact contact[N];
    dJointID joint;
    dContactGeom geoms[N];

    // store contacts in geoms array
    int n = dCollide(o1, o2, N, geoms, sizeof(dContactGeom));

    if (n > 0){
        //DBG("Collides: " << n << " - " << o1 << " " << o2);
        for (int i=0; i<n; i++){
            // copy default contact setting
            contact[i] = sim->_default_contact;
            // apply geom (pair of geoms)
            contact[i].geom = geoms[i];

            // create joint
            joint = dJointCreateContact (sim->_world, sim->_coll_contacts, &contact[i]);
            dJointAttach (joint, dGeomGetBody(contact[i].geom.g1),
                                 dGeomGetBody(contact[i].geom.g2));
        }
    }
}

}
