#include "motion_completion.hpp"
#include <cassert>
#include <chrono>

int main() {
    using namespace std::chrono_literals;
    using Joints = MotionCompletion::Joints;
    const auto t = MotionCompletion::Clock::time_point{};
    Joints initial{{90,0,90,0,90,90}};
    Joints nearby=initial; nearby[0]+=0.1;

    // An idle controller and an already-close pose do not prove an async move started.
    MotionCompletion delayed(initial,nearby,1,0,false);
    assert(!delayed.observe(0,false,true,0,2,initial,t+1s));
    assert(!delayed.observe(0,false,true,0,3,initial,t+2s));
    assert(!delayed.started());

    Joints target=initial; target[0]+=1;
    MotionCompletion moving(initial,target,10,5,false);
    assert(!moving.observe(1,false,true,5,11,initial,t));
    assert(!moving.started());
    assert(!moving.observe(2,true,false,5,12,initial,t+100ms));
    assert(moving.started());
    assert(!moving.observe(0,false,true,6,13,target,t+200ms));
    assert(!moving.observe(0,false,true,6,14,target,t+400ms));
    assert(moving.observe(0,false,true,6,15,target,t+700ms));

    // Repeated polling of one feedback frame is not multiple stable samples.
    MotionCompletion same_frame(initial,target,10,0,true);
    assert(!same_frame.observe(0,false,true,0,11,target,t));
    assert(!same_frame.observe(0,false,true,0,11,target,t+1s));
    assert(!same_frame.observe(0,false,true,0,12,target,t+1100ms));
    assert(same_frame.observe(0,false,true,0,13,target,t+1200ms));

    // A change larger than the stationary threshold restarts the stability window.
    MotionCompletion unstable(initial,target,10,0,true);
    Joints shifted=target; shifted[0]+=0.15;
    assert(!unstable.observe(0,false,true,0,11,target,t));
    assert(!unstable.observe(0,false,true,0,12,shifted,t+300ms));
    assert(!unstable.observe(0,false,true,0,13,shifted,t+600ms));
    assert(unstable.observe(0,false,true,0,14,shifted,t+800ms));
}
