#include "admittance_x.hpp"

#include <cassert>
#include <cmath>
#include <stdexcept>

int main() {
    admittance::FreeX controller;
    controller.reset(0.4);
    for (int i = 0; i < 500; ++i) {
        const auto output = controller.update(0.2, 0.02);
        assert(std::abs(output.target_x_m - 0.4) < 1e-10);
    }
    double previous = 0.4;
    for (int i = 0; i < 100; ++i) {
        const auto output = controller.update(5.0, 0.02);
        assert(output.target_x_m >= previous);
        assert(output.target_x_m - previous <= 0.000601);
        assert(output.velocity_m_s <= 0.030001);
        previous = output.target_x_m;
    }
    assert(previous > 0.43);
    for (int i = 0; i < 150; ++i) previous = controller.update(0.0, 0.02).target_x_m;
    const double stopped = previous;
    for (int i = 0; i < 150; ++i) controller.update(0.0, 0.02);
    assert(std::abs(controller.update(0.0, 0.02).target_x_m - stopped) < 0.0001);
    for (int i = 0; i < 300; ++i) {
        const auto output = controller.update(-5.0, 0.02);
        if (i == 299) assert(output.target_x_m < stopped);
    }
    bool rejected = false;
    try { controller.update(1.0, 0.2); }
    catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
}
