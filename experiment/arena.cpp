#include <osg/Geometry>
#include <osg/Array>
#include <osgUtil/SmoothingVisitor>
#include "arena.hpp"

#define ICE_LEN 6.
#define ICE_ANGLE (M_PI / 100)

static const Vec3 ps_coords[] = {
    Vec3(.5, -.5, .5),
    Vec3(-.5, -.5, .5),
    Vec3(-.5, -.5, -.5),
    Vec3(.5, -.5, -.5),
    Vec3(.5, .5, .5),
    Vec3(-.5, .5, .5),
    Vec3(-.5, .5, -.5),
    Vec3(.5, .5, -.5)
};
static const unsigned int ps_ids[] = {
    0, 1, 2, 3,
    4, 5, 6, 7,
    0, 1, 5, 4,
    2, 3, 7, 6,
    0, 3, 7, 4,
    1, 2, 6, 5
};

class PowerSourceVis : public sim::VisBody {
  public:
    PowerSourceVis()
        : VisBody()
    {
        size_t i;
        osg::ref_ptr<osg::Geode> g = new osg::Geode;
        osg::ref_ptr<osg::Vec3Array> vert = new osg::Vec3Array;
        osg::ref_ptr<osg::DrawElementsUInt> faces;

        for (i = 0; i < 8; i++){
            vert->push_back(ps_coords[i].toOsg());
        }

        osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
        geom->setVertexArray(vert);
        faces = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
        faces->push_back(ps_ids[0]);
        faces->push_back(ps_ids[1]);
        faces->push_back(ps_ids[2]);
        faces->push_back(ps_ids[3]);
        geom->addPrimitiveSet(faces);
        osgUtil::SmoothingVisitor::smooth(*geom.get());
        {
            osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
            color->push_back(osg::Vec4(0.7, 0.7, 0.1, 1.));
            geom->setColorArray(color);
            geom->setColorBinding(osg::Geometry::BIND_OVERALL);
        }
        g->addDrawable(geom);


        geom = new osg::Geometry;
        geom->setVertexArray(vert);

        for (i = 4; i < 24; i += 4){
            faces = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
            faces->push_back(ps_ids[i]);
            faces->push_back(ps_ids[i + 1]);
            faces->push_back(ps_ids[i + 2]);
            faces->push_back(ps_ids[i + 3]);
            geom->addPrimitiveSet(faces);
        }

        osgUtil::SmoothingVisitor::smooth(*geom.get());
        {
            osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
            color->push_back(osg::Vec4(0.7, 0.1, 0.1, 1.));
            geom->setColorArray(color);
            geom->setColorBinding(osg::Geometry::BIND_OVERALL);
        }

        g->addDrawable(geom);
        _setNode(g);
    }

    void toPovrayObject(std::ostream &os) const
    {
        int i;

        os << "#declare object_" << id() << "_1 = object {" << std::endl;

        os << "mesh {" << std::endl;
        os << "triangle {" << std::endl;
        os << "<" << ps_coords[ps_ids[0]].x() << ","
                  << ps_coords[ps_ids[0]].y() << ","
                  << ps_coords[ps_ids[0]].z() << ">, ";
        os << "<" << ps_coords[ps_ids[1]].x() << ","
                  << ps_coords[ps_ids[1]].y() << ","
                  << ps_coords[ps_ids[1]].z() << ">, ";
        os << "<" << ps_coords[ps_ids[2]].x() << ","
                  << ps_coords[ps_ids[2]].y() << ","
                  << ps_coords[ps_ids[2]].z() << ">}" << std::endl;
        os << "triangle {" << std::endl;
        os << "<" << ps_coords[ps_ids[2]].x() << ","
                  << ps_coords[ps_ids[2]].y() << ","
                  << ps_coords[ps_ids[2]].z() << ">, ";
        os << "<" << ps_coords[ps_ids[3]].x() << ","
                  << ps_coords[ps_ids[3]].y() << ","
                  << ps_coords[ps_ids[3]].z() << ">, ";
        os << "<" << ps_coords[ps_ids[0]].x() << ","
                  << ps_coords[ps_ids[0]].y() << ","
                  << ps_coords[ps_ids[0]].z() << ">}" << std::endl;
        os << "}" << std::endl; // mesh

        os << "pigment {";
        povColor(os, osg::Vec4(0.7, 0.7, 0.1, 1.));
        os << "}\n";

        os << "}" << std::endl; // object

        os << "#declare object_" << id() << "_2 = object {" << std::endl;
        os << "mesh {" << std::endl;
        for (i = 4; i < 24; i += 4){
            os << "triangle {" << std::endl;
            os << "<" << ps_coords[ps_ids[i + 0]].x() << ","
                      << ps_coords[ps_ids[i + 0]].y() << ","
                      << ps_coords[ps_ids[i + 0]].z() << ">, ";
            os << "<" << ps_coords[ps_ids[i + 1]].x() << ","
                      << ps_coords[ps_ids[i + 1]].y() << ","
                      << ps_coords[ps_ids[i + 1]].z() << ">, ";
            os << "<" << ps_coords[ps_ids[i + 2]].x() << ","
                      << ps_coords[ps_ids[i + 2]].y() << ","
                      << ps_coords[ps_ids[i + 2]].z() << ">}" << std::endl;
            os << "triangle {" << std::endl;
            os << "<" << ps_coords[ps_ids[i + 2]].x() << ","
                      << ps_coords[ps_ids[i + 2]].y() << ","
                      << ps_coords[ps_ids[i + 2]].z() << ">, ";
            os << "<" << ps_coords[ps_ids[i + 3]].x() << ","
                      << ps_coords[ps_ids[i + 3]].y() << ","
                      << ps_coords[ps_ids[i + 3]].z() << ">, ";
            os << "<" << ps_coords[ps_ids[i + 0]].x() << ","
                      << ps_coords[ps_ids[i + 0]].y() << ","
                      << ps_coords[ps_ids[i + 0]].z() << ">}" << std::endl;
        }
        os << "}" << std::endl; // mesh

        os << "pigment {";
        povColor(os, osg::Vec4(0.7, 0.1, 0.1, 1.));
        os << "}\n";

        os << "}" << std::endl; // object
    }

    void toPovrayTr(std::ostream &os) const
    {
        os << "object { object_" << id() << "_1" << std::endl;
        povTransformation(os, pos(), rot());
        os << "}" << std::endl; // object
        os << "object { object_" << id() << "_2" << std::endl;
        povTransformation(os, pos(), rot());
        os << "}" << std::endl; // object
    }
};

PowerSource::PowerSource(sim::Sim *sim, const Vec3 &pos, const Quat &rot)
    : sim::Component()
{
    PowerSourceVis *vis;

    vis = new PowerSourceVis();
    _body = sim->world()->createBodyCompound();
    _body->addBox(Vec3(.5, .5, .5), vis);

    _body->setPos(pos);
    _body->setRot(rot);

    _body->collSetDontCollideId(100);

    _body->activate();
}

void PowerSource::init(sim::Sim *sim)
{
}


Arena::Arena(sim::Sim *sim)
    : sim::Component(), _frame(0)
{
    osg::Vec4 color(0., 0.7, 0.1, 1.);
    osg::Vec4 color2(0., 0.1, 0.7, 1.);
    int id;

    _frame = sim->world()->createBodyCompound();
    id = _frame->addBox(Vec3(30 - ICE_LEN, 30, 0.1), SIM_BODY_DEFAULT_VIS,
                        Vec3(ICE_LEN / 2., 0, 0));
    _frame->visBody(id)->setColor(color);
    _frame->visBody(id)->setTexture("wood.ppm");

    id = _frame->addBox(Vec3(0.1, 30., 2.), SIM_BODY_DEFAULT_VIS, Vec3(-15, 0., .75));
    _frame->visBody(id)->setColor(color);
    _frame->visBody(id)->setTexture("wood.ppm");

    id = _frame->addBox(Vec3(0.1, 30., 2.), SIM_BODY_DEFAULT_VIS, Vec3(15, 0., .75));
    _frame->visBody(id)->setColor(color);
    _frame->visBody(id)->setTexture("wood.ppm");

    id = _frame->addBox(Vec3(30., .1, 2.), SIM_BODY_DEFAULT_VIS, Vec3(0., 15, .75));
    _frame->visBody(id)->setColor(color);
    _frame->visBody(id)->setTexture("wood.ppm");

    id = _frame->addBox(Vec3(30., .1, 2.), SIM_BODY_DEFAULT_VIS, Vec3(0., -15, .75));
    _frame->visBody(id)->setColor(color);
    _frame->visBody(id)->setTexture("wood.ppm");

    _frame->collSetDontCollideId(100);
    _frame->activate();


    _ice = sim->world()->createBodyCompound();
    id = _ice->addBox(Vec3(ICE_LEN, 30, 0.1), SIM_BODY_DEFAULT_VIS,
                      Vec3(-15 + ICE_LEN / 2., 0, (ICE_LEN / 2.) * sin(ICE_ANGLE)),
                      Quat(Vec3(0, 1, 0), ICE_ANGLE));
    _ice->visBody(id)->setColor(color2);
    _ice->visBody(id)->setTexture("wood.ppm");

    _ice->collSetFriction(0.0001);
    _ice->collSetDontCollideId(100);
    _ice->activate();
}

void Arena::init(sim::Sim *sim)
{
    PowerSource *ps;

    ps = new PowerSource(sim, Vec3(0, 14.7, .7));
    _pw_sources.push_back(ps);

    ps = new PowerSource(sim, Vec3(0, -14.7, .7), Quat(Vec3(0, 0, 1), M_PI));
    _pw_sources.push_back(ps);

    ps = new PowerSource(sim, Vec3(14.7, 0, .7), Quat(Vec3(0, 0, 1), -M_PI_2));
    _pw_sources.push_back(ps);

    ps = new PowerSource(sim, Vec3(-14.7, 0, .7), Quat(Vec3(0, 0, 1), M_PI_2));
    _pw_sources.push_back(ps);

    //ps = new PowerSource(sim, Vec3(10, 14.7, 1.4));
    //_pw_sources.push_back(ps);

    ps = new PowerSource(sim, Vec3(10, -14.7, 1.4), Quat(Vec3(0, 0, 1), M_PI));
    _pw_sources.push_back(ps);

    ps = new PowerSource(sim, Vec3(14.7, 10, 1.4), Quat(Vec3(0, 0, 1), -M_PI_2));
    _pw_sources.push_back(ps);

    ps = new PowerSource(sim, Vec3(-14.7, 10, 1.4), Quat(Vec3(0, 0, 1), M_PI_2));
    _pw_sources.push_back(ps);
}
