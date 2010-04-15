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


class O1 : public sim::ObjectSimple {
    float _x, _y, _z;
    int _dir[3];
    int _move[3];

  public:
    O1(float x = 0., float y = 0., float z = 3.)
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

    DBG(&s);
    s.addObject(new O1());
    s.addObject(new OPlane());

    s.run();
}
