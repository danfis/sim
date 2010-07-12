#ifndef _SIM_COMP_BLENDER_HPP_
#define _SIM_COMP_BLENDER_HPP_

#include <vector>
#include <string>
#include <sim/sim.hpp>
#include <sim/time.hpp>

namespace sim {

namespace comp {

    
class Blender : public sim::Component {
  protected:
    sim::Sim *_sim;
    const char *_dir;
    sim::Timer _timer;
    sim::Time _frame_duration;
    unsigned long _last_id;

  public:
    /** prefix specifies directory for output puictures. if the directory does not exist, no pictures wil
      * be created. FrameTime specifies time of one frame. this can be used .e.g for porducing movie, where one frame
      * is assumed to be 1/24 s long. if frameTime==-1, all simulation frames are outputted */
    Blender(const char *dir = "blender/",
            const sim::Time &frame_duration = sim::Time::fromMs((1./24.) * 1000L));
    ~Blender();

    void init(sim::Sim *sim);
    void finish();
    void cbPostStep();

  protected:
    void _updateObjects();
};

} /* namespace comp */

} /* namespace sim */

#endif /* _SIM_COMP_BLENDER_HPP_ */
