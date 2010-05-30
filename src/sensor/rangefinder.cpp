#include "rangefinder.hpp"
#include "sim/msg.hpp"

namespace sim {
namespace sensor {


RangeFinder::RangeFinder(Scalar max_range, size_t num_beams, Scalar angle_range)
    : sim::Component(),
      _max_range(max_range), _num_beams(num_beams), _angle_range(angle_range),
      _offset_pos(0., 0., 0.), _offset_rot(0., 0., 0., 1.)
{
    _intersectors = new osgUtil::IntersectorGroup;
    _visitor = new osgUtil::IntersectionVisitor(_intersectors);

    _data.detected = new bool[_num_beams];
    _data.dist = new Scalar[_num_beams];
    _data.point = new Vec3[_num_beams];
    _data.local = new Vec3[_num_beams];

    for (size_t i = 0; i < _num_beams; i++){
        _data.detected[i] = false;
    }
}

RangeFinder::~RangeFinder()
{
    delete [] _data.detected;
    delete [] _data.dist;
    delete [] _data.point;
    delete [] _data.local;
}

void RangeFinder::init(sim::Sim *sim)
{
    _sim = sim;

    _sim->regPreStep(this);

    {
        sim::Body *b = _sim->world()->createBodyCube(0.03, 0.);
        DBG(b);
        b->setPos(_offset_pos + Vec3(-0.1, 0., 0.));
        b->activate();
    }

    _createIntersectors();
    _updatePosition();
}

void RangeFinder::finish()
{
}

void RangeFinder::cbPreStep()
{
    static int __C = 0;
    osg::Node *root;

    if (__C != 0)
        return;
    __C++;

    if (_body)
        _updatePosition();

    // get root of scene
    root = _sim->visWorld()->sceneRoot();

    // traverse scene
    root->accept(*_visitor.get());

    DBG("");

    // gather measured data
    osgUtil::LineSegmentIntersector *is;
    osgUtil::LineSegmentIntersector::Intersection inter;
    osgUtil::IntersectorGroup::Intersectors &its = _intersectors->getIntersectors();

    for (size_t i = 0; i < _num_beams; i++){
        is = (osgUtil::LineSegmentIntersector *)its[i].get();

        if (is->containsIntersections()){
            inter = is->getFirstIntersection();

            _data.detected[i] = true;
            _data.point[i] = inter.getWorldIntersectPoint();
            _data.local[i] = inter.getLocalIntersectPoint();
            _data.dist[i] = _data.local[i].length();

            std::cerr << _data.detected[i] << ":" << _data.dist[i] << " ";
            {
            Vec3 pos = is->getFirstIntersection().getWorldIntersectPoint();
            sim::Body *b = _sim->world()->createBodyCube(0.01, 0.);
            b->visBody()->setColor(1., 0., 0., 1.);
            b->setPos(pos);
            b->activate();
            }
        }else{
            _data.detected[i] = false;
            _data.dist[i] = _max_range;
            _data.point[i].set(0., 0., 0.);
            _data.local[i].set(0., 0., 0.);
        }
        //DBG(i << ": " << is->containsIntersections() << " " << is->getIntersections().size());

        // clear list of intersections
        is->getIntersections().clear();
    }

    std::cerr << std::endl;
}


void RangeFinder::_createIntersectors()
{
    osgUtil::LineSegmentIntersector *beam;

    for (size_t i = 0; i < _num_beams; i++){
        beam = new osgUtil::LineSegmentIntersector(Vec3(0., 0., 0.), Vec3(0., 0., 0.));
        _intersectors->addIntersector(beam);
    }
}


void RangeFinder::_updatePosition()
{
    Vec3 pos;
    Quat rot;

    if (_body){
        pos = _offset_pos + _body->pos();
        rot = _offset_rot * _body->rot();
    }else{
        pos = _offset_pos;
        rot = _offset_rot;
    }

    Scalar angle, angle_step;
    Vec3 from(0., 0., 0.), to, to_dir(_max_range, 0., 0.);

    angle = -_angle_range / 2.;
    angle_step = _angle_range / (Scalar)(_num_beams - 1);

    osgUtil::LineSegmentIntersector *is;
    osgUtil::IntersectorGroup::Intersectors &its = _intersectors->getIntersectors();
    size_t i, len = its.size();
    for (i = 0; i < len; i++){
        is = (osgUtil::LineSegmentIntersector *)its[i].get();

        from = Vec3(0., 0., 0.);
        to = Quat(Vec3(0., 0., 1.), angle) * to_dir;

        from += pos;
        from = rot * from;
        to += pos;
        to = rot * to;

        is->setStart(from);
        is->setEnd(to);

        angle += angle_step;
    }
}


}
}
