/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Vojta Vonasek <vonasek@labe.felk.cvut.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "blender.hpp"
#include "sim/common.hpp"

namespace sim {

namespace comp {

Blender::Blender(const char *prefix, const double frameTime)
    : sim::Component(), _frame(0), _prefix(prefix),_frameTime(frameTime)
{
}

Blender::~Blender()
{
}

static void printLoadingIpoFunc(std::ofstream &ofs) {
    
    ofs << "def loadIpo(filename, objectName):\n";
    ofs << "    fin = open(filename,'r');\n";
    ofs << "    count = 1;\n";
    ofs << "    obj = Blender.Object.Get(objectName);\n";
    ofs << "    ipo = Ipo.New('Object','objectIpo');\n";
    ofs << "    ipo.addCurve('LocX');\n";
    ofs << "    ipo.addCurve('LocY');\n";
    ofs << "    ipo.addCurve('LocZ');\n";
    ofs << "    ipo.addCurve('RotX');\n";
    ofs << "    ipo.addCurve('RotY');\n";
    ofs << "    ipo.addCurve('RotZ');\n";
    ofs << "    x = ipo[Ipo.OB_LOCX];\n";
    ofs << "    y = ipo[Ipo.OB_LOCY];\n";
    ofs << "    z = ipo[Ipo.OB_LOCZ];\n";
    ofs << "    rx = ipo[Ipo.OB_ROTX];\n";
    ofs << "    ry = ipo[Ipo.OB_ROTY];\n";
    ofs << "    rz = ipo[Ipo.OB_ROTZ];\n";
    ofs << "    for line in fin:\n";
    ofs << "        nums = [float(_) for _ in line.split()]\n";
    ofs << "        x[count] = nums[0];\n";
    ofs << "        y[count] = nums[1];\n";
    ofs << "        z[count] = nums[2];\n";
    ofs << "        q = Mathutils.Quaternion([nums[3],nums[4],nums[5],nums[6]]);\n";
    ofs << "        obj.setEuler(q.toEuler());\n";
    ofs << "        euler = obj.getEuler();\n";
    ofs << "        rx[count] = euler[0]/10;\n";
    ofs << "        ry[count] = euler[1]/10;\n";
    ofs << "        rz[count] = euler[2]/10;\n";
    ofs << "        count+=1;\n";
    ofs << "    obj.setIpo(ipo);\n\n";
}


void Blender::init(sim::Sim *sim){
	char name[200];

	_sim = sim;

	sprintf(name, "%sobjects.py", _prefix);
	std::ofstream ofs(name);
    
    ofs << "#definition of objects from Sim created by Blender component\n";
    ofs << "import Blender\nfrom Blender import *\n\n";
    ofs << "sc = Scene.GetCurrent()\n";

    printLoadingIpoFunc(ofs);

    const std::list<VisBody *> &bodies = _sim->visWorld()->bodies();
    int i = 0;
    for_each(std::list<VisBody *>::const_iterator, bodies){
        if (*it){
            (*it)->exportToBlender(ofs, i);
            i++;
        }
    }
    ofs << "\n\n\n";

    i = 0;
    for_each(std::list<VisBody *>::const_iterator, bodies){
        if (*it){
            ofs << "loadIpo(\"object_" << i << ".ipo.dat\",\"object_" << i << "\")\n";
            i++;
        }
    }
    ofs << "\n\n\n";
    ofs << "Window.RedrawAll()\n\n";
    ofs.close();
 

    _sim->regPostStep(this);

    _lastFrameTime = _sim->timeSimulated();

}

void Blender::finish(){
}

void Blender::cbPostStep(){


    sim::Time current = _sim->timeSimulated();
    const double timeDiff = current.inSF() - _lastFrameTime.inSF();

    if (timeDiff > _frameTime) {

        char name[200];
        const std::list<VisBody *> &bodies = _sim->visWorld()->bodies();

        int i = 0;
        for_each(std::list<VisBody *>::const_iterator, bodies){
            if (*it) {
                sprintf(name,"%sobject_%d.ipo.dat",_prefix,i);
                const Vec3 pos((*it)->pos());
                const Quat rot((*it)->rot());
                std::ofstream ofs(name,std::ios::app);
                ofs << pos[0] << " " << pos[1] << " " << pos[2] << " " << rot.w() << " " << rot.x() << " " << rot.y() << " " << rot.z() << "\n";
                ofs.close();
                i++;
            }
        }

        _frame++;
        _lastFrameTime = current;
    }

}

}

}
