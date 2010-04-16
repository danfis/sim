#include <osg/Geode>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/Geometry>

#include "sim.hpp"
#include "geominfo.hpp"
#include "msg.hpp"


static double randRange(double from, double to)
{
    double n = (double)rand() / (double)RAND_MAX;
    return from + n * (to - from);
}

class OPlane : public sim::ObjectSimple {
  public:
    OPlane() { }

    void build(dWorldID world, dSpaceID space)
    {
        osg::Geode *geode = new osg::Geode();
        osg::Geometry *plane = new osg::Geometry();

        osg::Vec3 myCoords[] = {
            osg::Vec3(-10, -10, 0.),
            osg::Vec3(10, -10, 0.),
            osg::Vec3(10, 10, 0.),
            osg::Vec3(10, 10, 0.),
            osg::Vec3(-10, 10, 0.),
            osg::Vec3(-10, -10, 0.),
        };
        int numCoords = sizeof(myCoords)/sizeof(osg::Vec3);
        osg::Vec3Array* vertices = new osg::Vec3Array(numCoords,myCoords);
        plane->setVertexArray(vertices);
        plane->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES,0,6));

        geode->addDrawable(plane);

        _setOsgNode(geode);

        _setODEGeom(dCreatePlane(space, 0., 0., 1., 0.));
    }
};


class OBase : public sim::ObjectSimple {
  protected:
    float _x, _y, _z;
    int _dir[3];
    int _move[3];

  public:
    OBase(float x = 0., float y = 0., float z = 3.)
        : _x(x), _y(y), _z(z)
    {
        _dir[0] = _dir[1] = _dir[2] = 1;
        _move[0] = _move[1] = _move[2] = 0;

        if (randRange(-1., 1.) < 0.)
            _move[0] = 1;
        if (randRange(-1., 1.) < 0.)
            _move[1] = 1;
        if (randRange(-1., 1.) < 0.)
            _move[2] = 1;

        if (randRange(-1., 1.) < 0.)
            _dir[0] = -1;
        if (randRange(-1., 1.) < 0.)
            _dir[1] = -1;
    }
};

class OBaseGroup : public sim::ObjectGroup {
  protected:
    float _x, _y, _z;
    int _dir[3];
    int _move[3];

  public:
    OBaseGroup(float x = 0., float y = 0., float z = 3.)
        : _x(x), _y(y), _z(z)
    {
        _dir[0] = _dir[1] = _dir[2] = 1;
        _move[0] = _move[1] = _move[2] = 0;

        if (randRange(-1., 1.) < 0.)
            _move[0] = 1;
        if (randRange(-1., 1.) < 0.)
            _move[1] = 1;
        if (randRange(-1., 1.) < 0.)
            _move[2] = 1;

        if (randRange(-1., 1.) < 0.)
            _dir[0] = -1;
        if (randRange(-1., 1.) < 0.)
            _dir[1] = -1;
    }
};

class O1 : public OBase {
  public:
    O1(float x = 0., float y = 0., float z = 3.)
        : OBase(x, y, z){}

    void build(dWorldID world, dSpaceID space)
    {
        osg::Geode *geode = new osg::Geode();
        osg::Box *box = new osg::Box(osg::Vec3(0., 0., 0.), 1.);

        geode->addDrawable(new osg::ShapeDrawable(box));
        _setOsgNode(geode);


        dBodyID body;
        dGeomID geom;

        body = dBodyCreate(world);

        dMass mass;
        dMassSetBox(&mass, 1., 1., 1., 1.);
        dBodySetMass(body, &mass);

        geom = dCreateBox(space, 1., 1., 1.);
        dGeomSetBody(geom, body);

        dBodySetPosition(body, _x, _y, _z);

        _setODEBody(body);
        _setODEGeom(geom);
    }

    void preODE()
    {
        double step = 2.;
        dBodyAddRelForce(odeBody(),
                          step * _dir[0] * _move[0],
                          step * _dir[1] * _move[1], 0.);
    }
};

class O2 : public OBaseGroup {
  public:
    O2(float x = 0., float y = 0., float z = 0.)
        : OBaseGroup(x, y, z) {}

    void build(dWorldID world, dSpaceID space)
    {
        osg::Box *box[2];
        osg::Geode *geode;
        dJointID joint;
        dBodyID body[2];
        dGeomID geom[2];
        dMass mass;

        // create bodies
        body[0] = dBodyCreate(world);
        body[1] = dBodyCreate(world);

        // set shape and weight
        dMassSetBox(&mass, 1., 1., 1., 1.);
        dBodySetMass(body[0], &mass);
        dBodySetMass(body[1], &mass);

        // create geoms and assign them to bodies
        geom[0] = dCreateBox(space, 1., 1., 1.);
        geom[1] = dCreateBox(space, 1., 1., 1.);
        dGeomSetBody(geom[0], body[0]);
        dGeomSetBody(geom[1], body[1]);

        // create graphical representation (remember that initial position
        // should be 0, 0, 0)
        box[0] = new osg::Box(osg::Vec3(0., 0., 0.), 1.);
        box[1] = new osg::Box(osg::Vec3(0., 0., 0.), 1.);

        // and finally group bodies, geoms and osg nodes together
        for (size_t i = 0; i < 2; i++){
            geode = new osg::Geode();
            geode->addDrawable(new osg::ShapeDrawable(box[i]));
            _addObj(body[i], geom[i], geode);
        }

        // position bodies in space
        dBodySetPosition(body[0], _x, _y, _z);
        dBodySetPosition(body[1], _x + 1., _y + 1., _z);

        // create joint
        joint = dJointCreateFixed(world, 0);
        dJointAttach(joint, body[0], body[1]);

        // fix joint on current relative position of bodies
        dJointSetFixed(joint);

    }

    void preODE()
    {
        double step = 2.;
        dBodyAddRelForce(odeBody(0),
                          step * _dir[0] * _move[0],
                          step * _dir[1] * _move[1], 0.);
    }
};

class O3 : public OBaseGroup {
    dJointID _motor;
    GeomInfo *_geom_info;

  public:
    O3(float x = 0., float y = 0., float z = 0.)
        : OBaseGroup(x, y, z), _geom_info(0) {}

    ~O3()
    {
        if (_geom_info)
            delete _geom_info;
    }

    void build(dWorldID world, dSpaceID space)
    {
        osg::Box *box;
        osg::Sphere *sphere;
        osg::Geode *geode;
        dBodyID body[4];
        dGeomID geom[4];
        dJointID joint[3];
        dMass mass;
        const dReal *pos;
        size_t i;
        //double length = 0.7, width = 0.5, height = 0.2, radius = 0.18;
        double length = 1.4, width = 1., height = 0.4, radius = 0.36;

        // create bodies
        for (i = 0; i < 4; i++){
            body[i] = dBodyCreate(world);
        }

        // chassis
        dMassSetBox(&mass, 1., length, width, height);
        dBodySetMass(body[0], &mass);
        geom[0] = dCreateBox(space, length, width, height);


        // wheels
        for (i = 1; i < 4; i++){
            dQuaternion q;
            dQFromAxisAndAngle (q, 1., 0., 0., M_PI * 0.5);
            dBodySetQuaternion (body[i], q);
            dMassSetSphere (&mass, 1., radius);
            dBodySetMass (body[i], &mass);
            geom[i] = dCreateSphere(space, radius);
        }

        // set initial positions
        dBodySetPosition(body[0], _x, _y, _z);
        dBodySetPosition(body[1], _x + 0.5 * length ,_y + 0, _z - height * 0.5);
        dBodySetPosition(body[2], _x - 0.5 * length, _y + width * 0.5, _z - height * 0.5);
        dBodySetPosition(body[3], _x - 0.5 * length, _y - width * 0.5, _z - height * 0.5);

        // assign geoms to bodies
        for (i = 0; i < 4; i++){
            dGeomSetBody(geom[i], body[i]);
        }

        // create joint
        for (i = 0; i < 3; i++){
            joint[i] = dJointCreateHinge2(world, 0);
            dJointAttach(joint[i], body[0], body[i + 1]);
            pos = dBodyGetPosition(body[i + 1]);
            dJointSetHinge2Anchor(joint[i], pos[0], pos[1], pos[2]);
            dJointSetHinge2Axis1(joint[i], 0., 0., 1.);
            dJointSetHinge2Axis2(joint[i], 0., 1., 0.);

            // set suspension
            dJointSetHinge2Param(joint[i], dParamSuspensionERP, 0.8);
            dJointSetHinge2Param(joint[i], dParamSuspensionCFM, 0.2);

            // lock back wheels along the steering axis
            // set stops to make sure wheels always stay in alignment
            dJointSetHinge2Param(joint[i], dParamLoStop, 0.);
            dJointSetHinge2Param(joint[i], dParamHiStop, 0.);
        }

        // set wheels to dont collide with chassis
        // create GeomInfo struct (don't forget to free memory in destructor)
        _geom_info = new GeomInfo();
        // set dont_collide_id to some unique number (non zero)
        _geom_info->dont_collide_id = (long)this;
        // assign this struct to all parts of robot
        for (i = 0; i < 4; i++){
            dGeomSetData(geom[i], _geom_info);
        }

        // set motor
        _motor = joint[0];
            

        // create graphical representation
        // chassis
        box = new osg::Box(osg::Vec3(0., 0., 0.), length, width, height);
        geode = new osg::Geode();
        geode->addDrawable(new osg::ShapeDrawable(box));
        _addObj(body[0], geom[0], geode);

        // wheels
        for (i = 1; i < 4; i++){
            sphere = new osg::Sphere(osg::Vec3(0., 0., 0.), radius);
            geode = new osg::Geode();
            geode->addDrawable(new osg::ShapeDrawable(sphere));
            _addObj(body[i], geom[i], geode);
        }
    }

    void preODE()
    {
		dJointSetHinge2Param(_motor, dParamVel2, -2);
		dJointSetHinge2Param(_motor, dParamFMax2, 2);

		dJointSetHinge2Param(_motor, dParamVel, 0);
		dJointSetHinge2Param(_motor, dParamFMax, 0.2);
		dJointSetHinge2Param(_motor, dParamLoStop, -0.75);
		dJointSetHinge2Param(_motor, dParamHiStop, 0.75);
		dJointSetHinge2Param(_motor, dParamFudgeFactor, 0.1);
    }
};


class MySim : public sim::Sim {
  public:
    bool pressedKey(int key)
    {
        sim::Object *obj;
        float x, y, z;

        //DBG("Key down " << (char)key);

        if (key == '1'){
            x = randRange(-9, 9);
            y = randRange(-9, 9);
            z = randRange(0.5, 1.);
            O1 *obj = new O1(x, y, z);

            addObject(obj);
            return true;
        }else if (key == '2'){
            x = randRange(-9, 9);
            y = randRange(-9, 9);
            z = randRange(0.5, 1.);
            O2 *obj = new O2(x, y, z);

            addObject(obj);
            return true;
        }else if (key == '3'){
            x = randRange(-9, 9);
            y = randRange(-9, 9);
            z = randRange(0.5, 1.);
            O3 *obj = new O3(x, y, z);

            addObject(obj);
            return true;
        }else if (key == 'p'){
            obj = new OPlane();
            addObject(obj);
            return true;
        }

        return false;
    }
};

int main(int argc, char *argv[])
{
    MySim s;

    s.addObject(new OPlane());

    s.run();
}
