#include "joint_steps.hpp"
#include <cassert>
#include <limits>
#include <stdexcept>

int main() {
    JointStepAudit::Joints start{{0,0,0,0,0,0}};
    JointStepAudit audit(start,360);
    JointStepAudit::Joints at_limit=start;at_limit[5]=360;
    audit.add(at_limit,"start -> home");
    assert(audit.largest()==360 && audit.joint()==6 && audit.segment()=="start -> home");
    auto above=start;above[5]=-0.01;
    bool rejected=false;
    try {audit.add(above,"home -> pose 1");} catch(const std::runtime_error&) {rejected=true;}
    assert(rejected);
    rejected=false;above=start;above[0]=std::numeric_limits<double>::quiet_NaN();
    try {audit.add(above,"invalid");} catch(const std::runtime_error&) {rejected=true;}
    assert(rejected);
}
