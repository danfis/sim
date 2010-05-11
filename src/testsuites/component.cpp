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
    s.visWorld()->setWindow(false);

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
  public:
    M1() : sim::Message(sim::Message::PRIO_LOWEST) {}
};

class M2 : public sim::Message {
    SIM_MESSAGE_INIT(11)
  public:
    M2() : sim::Message(sim::Message::PRIO_NORMAL) {}
};

class M3 : public sim::Message {
    SIM_MESSAGE_INIT(12)
  public:
    std::string msg;
    M3(const std::string &m = "") : sim::Message(sim::Message::PRIO_HIGHEST), msg(m) {}
};

class C1 : public sim::Component {
  public:
    void processMessage(const sim::Message &msg)
    {
        cout << "C1::processMessage - prio:" << prio();
        cout << " M(prio: " << msg.prio() << ", type: " << msg.type() << ")" << endl;
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
        cout << "C2::processMessage - prio:" << prio();
        cout << " M(prio: " << msg.prio() << ", type: " << msg.type() << ")" << endl;
        if (msg.type() == M3::Type){
            cout << "    M3::msg: " << ((const M3 &)msg).msg << endl;
        }
    }
};

class C22 : public C2 {
    sim::Sim *sim;
    void init(sim::Sim *s) { sim = s; }
    void processMessage(const sim::Message &msg)
    {
        cout << "C22::processMessage - prio:" << prio();
        cout << " M(prio: " << msg.prio() << ", type: " << msg.type() << ")" << endl;
        if (msg.type() == M3::Type){
            cout << "    M3::msg: " << ((const M3 &)msg).msg << endl;
        }

        if (msg.type() == M3::Type && ((const M3 &)msg).msg == "C22 send"){
            cout << "  C22 -> sending M1" << endl;
            sim->sendMessage(new M1());
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
        cout << "C3::processMessage - prio:" << prio();
        cout << " M(prio: " << msg.prio() << ", type: " << msg.type() << ")" << endl;
        if (msg.type() == M3::Type){
            cout << "    M3::msg: " << ((const M3 &)msg).msg << endl;
        }
    }
};

TEST(compMsg)
{
    cout << endl << "--- compMsg() ---" << endl;

    sim::Sim s;
    s.visWorld()->setWindow(false);

    C1 *c1 = new C1();
    C2 *c2 = new C2();
    C2 *c22 = new C22();
    C3 *c3 = new C3();
    s.addComponent(c1);
    s.addComponent(c2);
    s.addComponent(c3);
    s.regMessage(c1, M1::Type);
    s.regMessage(c2, M1::Type);
    s.regPreStep(c2);
    s.regPostStep(c3);

    s.init();

    cout << "Sending M1" << endl;
    s.sendMessage(new M1());
    s.step();
    cout << endl;

    s.step();
    cout << endl;

    cout << "Sending M2 M1" << endl;
    s.sendMessage(new M2());
    s.sendMessage(new M1());
    s.step();
    cout << endl;

    s.regMessage(c2, M2::Type);
    s.regMessage(c3, M2::Type);
    s.unregMessage(c2, M1::Type);
    cout << "Sending M2, M1" << endl;
    s.sendMessage(new M2());
    s.sendMessage(new M1());
    s.step();
    cout << endl;

    s.step();
    cout << endl;

    cout << "Sending M3" << endl;
    s.sendMessage(new M3());
    s.step();
    cout << endl;

    s.regMessage(c1, M3::Type);
    s.regMessage(c2, M3::Type);
    s.regMessage(c3, M3::Type);

    cout << "Sending M3(1), M3(2), M1" << endl;
    s.sendMessage(new M3("1"));
    s.sendMessage(new M3("2"));
    s.sendMessage(new M1());
    s.step();
    cout << endl;

    s.regMessage(c2, M2::Type);
    s.regMessage(c2, M1::Type);
    cout << "Sending M3(1), M1, M2, M3(2)" << endl;
    s.sendMessage(new M3("1"));
    s.sendMessage(new M1());
    s.sendMessage(new M2());
    s.sendMessage(new M3("2"));
    s.step();
    cout << endl;

    s.rmComponent(c2);
    cout << "Sending M3(1)" << endl;
    s.sendMessage(new M3("1"));
    s.step();
    cout << endl;

    s.addComponent(c2);
    s.addComponent(c22);
    s.regMessage(c2, M3::Type);
    s.regMessage(c2, M2::Type);
    s.regMessage(c22, M3::Type);
    s.regMessage(c22, M2::Type);

    cout << "Sending M3(1), M2, M3(3)" << endl;
    s.sendMessage(new M3("1"));
    s.sendMessage(new M2());
    s.sendMessage(new M3("3"));

    s.step();
    cout << endl;

    s.regMessage(c2, M3::Type);
    cout << "Sending M3(1)" << endl;
    s.sendMessage(new M3("1"));
    s.step();
    cout << endl;

    cout << "Sending M3(C22 send)" << endl;
    s.sendMessage(new M3("C22 send"));
    s.step();
    cout << endl;

    s.step();
    cout << endl;

    s.step();
    cout << endl;

    s.step();
    cout << endl;


    s.regMessage(c2, M1::Type);
    s.regMessage(c22, M1::Type);
    cout << "Sending M1 with highest, M2" << endl;
    s.sendMessage(new M1(), sim::Message::PRIO_HIGHEST);
    s.sendMessage(new M2());
    s.sendMessage(new M3());
    s.step();
    cout << endl;

    s.step();
    cout << endl;

    s.finish();

    cout << "--- compMsg() ---" << endl;
    cout << endl;
}
