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

#include "snake.hpp"
#include <sim/msg.hpp>
#include <osgGA/GUIEventAdapter>


namespace sim {

namespace comp {


Snake::Snake(const std::vector<sim::Joint *> &snakeBodies) {
	_snakeBodies = snakeBodies;
	selectedRobot = 0;
//	oldColor = snakeBodies[0]->getColor();
}

Snake::~Snake(){
	_snakeBodies.clear();
}

void Snake::init(sim::Sim *sim) {
	_sim = sim;
	for(int i=0;i<(int)_snakeBodies.size();i++) {
		vel1.push_back(0);
		vel2.push_back(0);
		_snakeBodies[i]->setParamVel(0);			
		_snakeBodies[i]->setParamVel2(0);			
	}

}

void Snake::finish(){
}

void Snake::cbPreStep() {

}

void Snake::processMessage(const Message &msg) {

	const int step = 1;

	if (msg.type() == MessageKeyPressed::Type) {
		const MessageKeyPressed &m = (const MessageKeyPressed &)msg;
		if (m.key() == 'n') {
			// 'n'
			selectedRobot=++selectedRobot % _snakeBodies.size();
			DBG("Body " << selectedRobot << " is selected");
		} else if (m.key() == 'm') {
			// 'm'
			selectedRobot--;
			if (selectedRobot < 0) {
				selectedRobot = (int)_snakeBodies.size()-1;
			}
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
			_snakeBodies[i]->setParamVel(vel1[i]);			
			_snakeBodies[i]->setParamVel2(vel2[i]);	
			_snakeBodies[i]->setParamFMax(40);	
			_snakeBodies[i]->setParamFMax2(40);	
			DBG("Joint " << i << ", vel1=" << vel1[i] << ", vel2=" << vel2[i]);	
		}
	}	
}


}

}
