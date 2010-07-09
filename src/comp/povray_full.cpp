#include "sim/comp/povray_full.hpp"
#include "sim/common.hpp"

namespace sim {
namespace comp {

PovrayFull::PovrayFull(const char *dir)
    : _sim(0), _frame(0), _dir(dir)
{
}

PovrayFull::~PovrayFull()
{
}

void PovrayFull::init(sim::Sim *sim)
{
    _sim = sim;

    // create camera and lights files
    _cameraFile();
    _lightsFile();

    // register Component to be called after each step
    _sim->regPostStep(this);
}

void PovrayFull::finish()
{
}

void PovrayFull::cbPostStep()
{
    char fn[200];
    VisWorld *vw;

    if (!_sim)
        return;
    if (!(vw = _sim->visWorld()))
        return;

    snprintf(fn, 200, "%s/frame_%010ld.pov", _dir, _frame);

    std::ofstream fout(fn);
    if (fout.good()){
        const std::list<VisBody *> &bodies = vw->bodies();

        fout << "#include \"camera.inc\"" << std::endl;
        fout << "#include \"lights.inc\"" << std::endl;

        for_each(std::list<VisBody *>::const_iterator, bodies){
            (*it)->toPovrayFull(fout);
        }

        fout.close();
    }

    _frame++;
}

void PovrayFull::_cameraFile()
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

void PovrayFull::_lightsFile()
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

} /* namespace comp */
} /* namespace sim */
