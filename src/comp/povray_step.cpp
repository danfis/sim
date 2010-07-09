#include "sim/comp/povray_step.hpp"
#include "sim/common.hpp"

namespace sim {
namespace comp {

PovrayStep::PovrayStep(const char *dir)
    : _sim(0), _frame(0), _dir(dir), _last_id(0)
{
}

PovrayStep::~PovrayStep()
{
}

void PovrayStep::init(sim::Sim *sim)
{
    _sim = sim;

    // create camera and lights files
    _cameraFile();
    _lightsFile();

    // register Component to be called after each step
    _sim->regPostStep(this);
}

void PovrayStep::finish()
{
}

void PovrayStep::cbPostStep()
{
    char fn[200];
    char fn_object[200];
    VisWorld *vw;

    if (!_sim)
        return;
    if (!(vw = _sim->visWorld()))
        return;

    if (_last_id < VisBody::lastId()){
        _updateObjects();
        _last_id = VisBody::lastId();
    }

    snprintf(fn, 200, "%s/frame_%010ld.pov", _dir, _frame);

    std::ofstream fout(fn);
    if (fout.good()){
        const std::list<VisBody *> &bodies = vw->bodies();

        fout << "#include \"camera.inc\"" << std::endl;
        fout << "#include \"lights.inc\"" << std::endl;

        for_each(std::list<VisBody *>::const_iterator, bodies){
            snprintf(fn_object, 200, "#include \"object_%010ld.inc\"", (*it)->id());
            fout << fn_object << std::endl;
            (*it)->toPovrayTr(fout);
        }

        fout.close();
    }

    _frame++;
}

void PovrayStep::_cameraFile()
{
    char fn[200];

    snprintf(fn, 200, "%s/camera.inc", _dir);
    std::ofstream fout(fn);
    if (fout.good()){
        fout << "camera {\n"
	            "    location <6,-6,4>\n"
	            "    sky <0,0,1>\n"
	            "    look_at <1,0,3>\n"
                "}\n";
        fout.close();
    }
}

void PovrayStep::_lightsFile()
{
    char fn[200];

    snprintf(fn, 200, "%s/lights.inc", _dir);
    std::ofstream fout(fn);
    if (fout.good()){
        fout << "#include \"colors.inc\"\n";
        fout << "light_source { <20,-5,20> color White }\n"
	            "light_source { <-20,-5,20> color White }\n"
	            "light_source { <0,-15,10> color White }\n";
        fout.close();
    }
}

void PovrayStep::_updateObjects()
{
    char fn[200];
    // this method is called from cbPostStep() _after_ check of _sim and
    // _sim->visWorld() so no more checks are needed
    VisWorld *vw = _sim->visWorld();
    const std::list<VisBody *> &bodies = vw->bodies();


    for_each(std::list<VisBody *>::const_iterator, bodies){
        if ((*it)->id() > _last_id){
            snprintf(fn, 200, "%s/object_%010ld.inc", _dir, (*it)->id());
            std::ofstream fout(fn);
            if (fout.good()){
                (*it)->toPovrayObject(fout);
                fout.close();
            }
        }
    }
}

} /* namespace comp */
} /* namespace sim */

