#include <osg/Geode>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/Geometry>

#include "sim.hpp"
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

        DBG(": " << _x << " " << _y << " " << _z);

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

        DBG("position: " << _x << " " << _y << " " << _z);
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
            DBG("1: " << x << " " << y << " " << z);
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
