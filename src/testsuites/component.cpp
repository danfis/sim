#include <iostream>
using std::cout;
using std::endl;
using std::cerr;

#include "cu.h"
#include "sim/sim.hpp"

class Comp : public sim::Component {
  public:
    void init(sim::Sim *_s)
    {
        cout <<"Comp::init" << endl;
    }

    void finish()
    {
        cout << "Comp::finish" << endl;
    }

    void cbPreStep()
    {
        cout << "Comp::cbPreStep" << endl;
    }

    void cbPostStep()
    {
        cout << "Comp::cbPostStep" << endl;
    }
};

TEST(compSetUp)
{
}

TEST(compTearDown)
{
}

TEST(compPrePostStep)
{
    sim::Sim s;
    Comp *c = new Comp();
    s.addComponent(c);
    s.regPreStep(c);
    s.regPostStep(c);

    s.init();
    s.step();
    s.step();
    s.step();

    s.unregPreStep(c);
    s.step();
    s.step();

    s.regPreStep(c);
    s.unregPostStep(c);
    s.step();
    s.step();

    s.finish();

    s.rmComponent(c);
    delete c;

    cout << endl;
}


class M1 : public sim::Message {
    SIM_MESSAGE_INIT(10)
};

class M2 : public sim::Message {
    SIM_MESSAGE_INIT(11)
};

class M3 : public sim::Message {
    SIM_MESSAGE_INIT(12)
  public:
    std::string msg;
    M3(const std::string &m = "") : msg(m) {}
};

class C1 : public sim::Component {
  public:
    void processMessage(const sim::Message &msg)
    {
        cout << "C1::processMessage - prio:" << prio() << " type: " << msg.type() << endl;
        if (msg.type() == M3::Type){
            cout << "    M3::msg: " << ((const M3 &)msg).msg << endl;
        }
    }
};

class C2 : public sim::Component {
  public:
    C2() : sim::Component(sim::Component::PRIO_HIGHEST) {}

    void cbPreStep()
    {
        cout << "C2::preStep()" << endl;
    }

    void processMessage(const sim::Message &msg)
    {
        cout << "C2::processMessage - prio:" << prio() << " type: " << msg.type() << endl;
        if (msg.type() == M3::Type){
            cout << "    M3::msg: " << ((const M3 &)msg).msg << endl;
        }
    }
};

class C3 : public sim::Component {
  public:
    C3() : sim::Component(sim::Component::PRIO_LOWER) {}

    void cbPostStep()
    {
        cout << "C3::postStep()" << endl;
    }

    void processMessage(const sim::Message &msg)
    {
        cout << "C3::processMessage - prio:" << prio() << " type: " << msg.type() << endl;
        if (msg.type() == M3::Type){
            cout << "    M3::msg: " << ((const M3 &)msg).msg << endl;
        }
    }
};

TEST(compMsg)
{
    cerr << endl;
    cout << endl << "--- compMsg() ---" << endl;

    sim::Sim s;
    C1 *c1 = new C1();
    C2 *c2 = new C2();
    C3 *c3 = new C3();
    s.addComponent(c1);
    s.addComponent(c2);
    s.addComponent(c3);
    s.regMessage(c1, M1::Type);
    s.regMessage(c2, M1::Type);
    s.regPreStep(c2);
    s.regPostStep(c3);

    s.init();

    s.sendMessage(new M1());
    s.step();
    cout << endl;

    s.step();
    cout << endl;

    s.sendMessage(new M2());
    s.sendMessage(new M1());
    s.step();
    cout << endl;

    s.regMessage(c2, M2::Type);
    s.regMessage(c3, M2::Type);
    s.unregMessage(c2, M1::Type);
    s.sendMessage(new M2());
    s.sendMessage(new M1());
    s.step();
    cout << endl;

    s.step();
    cout << endl;

    s.sendMessage(new M3());
    s.step();
    cout << endl;

    s.regMessage(c1, M3::Type);
    s.regMessage(c2, M3::Type);
    s.regMessage(c3, M3::Type);

    s.sendMessage(new M3("1"));
    s.sendMessage(new M3("2"));
    s.sendMessage(new M1());

    s.step();
    cout << endl;

    s.finish();

    cout << endl << "--- compMsg() ---" << endl;
    cout << endl;
}
