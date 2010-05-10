#include <stdio.h>
#include <iostream>

#include "cu.h"
#include "sim/sim.hpp"

class Comp : public sim::Component {
  public:
    void init(sim::Sim *_s)
    {
        printf("Comp::init\n");
    }

    void finish()
    {
        printf("Comp::finish\n");
    }

    void cbPreStep()
    {
        printf("Comp::cbPreStep\n");
    }

    void cbPostStep()
    {
        printf("Comp::cbPostStep\n");
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
    s.finish();

    s.rmComponent(c);
    delete c;
}

