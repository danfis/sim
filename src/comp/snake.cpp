#include "snake.hpp"
#include <sim/msg.hpp>

namespace sim {

namespace comp {


Snake::Snake(const std::vector<sim::Body *> &snakeBodies) {
	_snakeBodies = snakeBodies;
}

Snake::~Snake(){
	_snakeBodies.clear();
}

void Snake::init(sim::Sim *sim) {
	_sim = sim;
}

void Snake::finish(){
}

void Snake::cbPreStep() {

}

void Snake::processMessage(const Message &msg) {
//	DBG("keypressed: " << (Meymsg.key());	
	if (msg.type() == MessageKeyPressed::Type) {
		const MessageKeyPressed &m = (const MessageKeyPressed &)msg;
		DBG("keyboard mesage:" << m.key());


	}	
}


}

}
