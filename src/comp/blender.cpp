#include "blender.hpp"
#include "sim/common.hpp"

namespace sim {
namespace comp {

static void printLoadingIpoFunc(std::ofstream &ofs) {
    
    ofs << "def loadIpo(filename, objectName):\n";
    ofs << "    print \"Loading IPO curve for \",objectName\n";
    ofs << "    fin = open(filename,'r')\n";
    ofs << "    count = 1\n";
    ofs << "    obj = Blender.Object.Get(objectName)\n";
    ofs << "    ipo = Ipo.New('Object','objectIpo')\n";
    ofs << "    ipo.addCurve('LocX')\n";
    ofs << "    ipo.addCurve('LocY')\n";
    ofs << "    ipo.addCurve('LocZ')\n";
    ofs << "    ipo.addCurve('RotX')\n";
    ofs << "    ipo.addCurve('RotY')\n";
    ofs << "    ipo.addCurve('RotZ')\n";
    ofs << "    x = ipo[Ipo.OB_LOCX]\n";
    ofs << "    y = ipo[Ipo.OB_LOCY]\n";
    ofs << "    z = ipo[Ipo.OB_LOCZ]\n";
    ofs << "    rx = ipo[Ipo.OB_ROTX]\n";
    ofs << "    ry = ipo[Ipo.OB_ROTY]\n";
    ofs << "    rz = ipo[Ipo.OB_ROTZ]\n";
    ofs << "    x.setInterpolation('Constant')\n";
    ofs << "    y.setInterpolation('Constant')\n";
    ofs << "    z.setInterpolation('Constant')\n";
    ofs << "    rx.setInterpolation('Constant')\n";
    ofs << "    ry.setInterpolation('Constant')\n";
    ofs << "    rz.setInterpolation('Constant')\n";
    ofs << "    for line in fin:\n";
    ofs << "        nums = [float(_) for _ in line.split()]\n";
    ofs << "        x[count] = nums[0]\n";
    ofs << "        y[count] = nums[1]\n";
    ofs << "        z[count] = nums[2]\n";
    ofs << "        q = Mathutils.Quaternion([nums[3],nums[4],nums[5],nums[6]])\n";
    ofs << "        obj.setEuler(q.toEuler())\n";
    ofs << "        euler = obj.getEuler()\n";
    ofs << "        rx[count] = euler[0]/10\n";
    ofs << "        ry[count] = euler[1]/10\n";
    ofs << "        rz[count] = euler[2]/10\n";
    ofs << "        count+=1\n";
    ofs << "    obj.setIpo(ipo)\n\n";
}


Blender::Blender(const char *dir, const sim::Time &frame_duration)
    : _sim(0), _dir(dir), _frame_duration(frame_duration),
      _last_id(0)
{
}

Blender::~Blender()
{
}

void Blender::init(sim::Sim *sim){
	char name[200];

	_sim = sim;

	sprintf(name, "%s/objects.py", _dir);
	std::ofstream ofs(name);
    
    if (ofs.good()){
        ofs << "#definition of objects from Sim created by Blender component\n";
        ofs << "import Blender\nfrom Blender import *\n\n";
        ofs << "sc = Scene.GetCurrent()\n";

        printLoadingIpoFunc(ofs);
        ofs.close();

        _sim->regPostStep(this);
    }

    _timer.start();
}

void Blender::finish()
{
}

void Blender::cbPostStep()
{
    sim::VisWorld *vw = _sim->visWorld();

    if (!vw)
        return;

    if (_timer.stop() >= _frame_duration){
        char fn[200];
        const std::list<VisBody *> &bodies = vw->bodies();

        if (_last_id < VisBody::lastId()){
            _updateObjects();
            _last_id = VisBody::lastId();
        }

        for_each(std::list<VisBody *>::const_iterator, bodies){
            if (*it) {
                sprintf(fn, "%s/object_%ld.ipo.dat", _dir, (*it)->id());
                const Vec3 pos((*it)->pos());
                const Quat rot((*it)->rot());
                std::ofstream ofs(fn, std::ios_base::app);
                ofs << pos[0] << " " << pos[1] << " " << pos[2] << " " << rot.w() << " " << rot.x() << " " << rot.y() << " " << rot.z() << "\n";
                ofs.close();
            }
        }

        _timer.start();
    }
}

void Blender::_updateObjects()
{
    char fn[200];
    // this method is called from cbPostStep() _after_ check of _sim and
    // _sim->visWorld() so no more checks are needed
    VisWorld *vw = _sim->visWorld();
    const std::list<VisBody *> &bodies = vw->bodies();


	sprintf(fn, "%s/objects.py", _dir);
	std::ofstream fout(fn, std::ios_base::app);
    if (fout.good()){
        for_each(std::list<VisBody *>::const_iterator, bodies){
            if ((*it)->id() > _last_id){
                (*it)->toBlender(fout);
                fout << "loadIpo(\"object_" << (*it)->id() << ".ipo.dat\",\"object_" << (*it)->id() << "\")\n";
            }
        }

        fout.close();
    }
}


}
}
