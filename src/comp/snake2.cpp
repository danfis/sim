/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Vojta Vonasek <vonasek@labe.felk.cvut.cz>
 *                   Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "snake2.hpp"
#include <sim/msg.hpp>
#include <osgGA/GUIEventAdapter>
#include <vector>

namespace sim {

namespace comp {


SnakeBody::SnakeBody(const std::vector<sim::robot::SSSA *> &snakeBodies) {
	_snakeBodies = snakeBodies;
	selectedRobot = 0;
    oldColor = snakeBodies[selectedRobot]->chasisColor();
    snakeBodies[selectedRobot]->setChasisColor(osg::Vec4(0.8,0.5,0.9,1));
}

SnakeBody::~SnakeBody(){
	_snakeBodies.clear();
}

void SnakeBody::init(sim::Sim *sim) {
	_sim = sim;
	for(int i=0;i<(int)_snakeBodies.size();i++) {
		vel1.push_back(0);
		vel2.push_back(0);
	}
    sim->regMessage(this,sim::MessageKeyPressed::Type);

}

void SnakeBody::finish(){
}

void SnakeBody::cbPreStep() {

}

void SnakeBody::processMessage(const Message &msg) {

	const int step = 1;
	if (msg.type() == MessageKeyPressed::Type) {
		const MessageKeyPressed &m = (const MessageKeyPressed &)msg;
		if (m.key() == 'n') {
			// 'n'
            _snakeBodies[selectedRobot]->setChasisColor(oldColor);

			selectedRobot = (selectedRobot + 1) % _snakeBodies.size();
            
            oldColor = _snakeBodies[selectedRobot]->chasisColor();
            _snakeBodies[selectedRobot]->setChasisColor(osg::Vec4(0.8,0.5,0.9,1));
			DBG("Body " << selectedRobot << " is selected");
		} else if (m.key() == 'm') {
			// 'm'
            _snakeBodies[selectedRobot]->setChasisColor(oldColor);
			selectedRobot--;
			if (selectedRobot < 0) {
				selectedRobot = (int)_snakeBodies.size()-1;
			}
            _snakeBodies[selectedRobot]->setChasisColor(osg::Vec4(0.8,0.5,0.9,1));
			DBG("Body " << selectedRobot << " is selected");

		} else if (m.key() == osgGA::GUIEventAdapter::KEY_Up) {
			// up
			vel1[selectedRobot]+= step;
		} else if (m.key() == osgGA::GUIEventAdapter::KEY_Down) {
			// down
			vel1[selectedRobot]-= step;
		} else if (m.key() == osgGA::GUIEventAdapter::KEY_Left) {
			// left
			vel2[selectedRobot]+= step;
		} else if (m.key() == osgGA::GUIEventAdapter::KEY_Right) {
			// right 
			vel2[selectedRobot]-= step;
		} else {
			DBG("keyboard mesage:" << m.key());
		}

		for(int i=0;i<(int)_snakeBodies.size();i++) {
            _snakeBodies[i]->setVelArm(vel1[i]);
			DBG("Joint " << i << ", vel1=" << vel1[i] << ", vel2=" << vel2[i]);	
		}
        DBG("");
	}	
}


}

}
