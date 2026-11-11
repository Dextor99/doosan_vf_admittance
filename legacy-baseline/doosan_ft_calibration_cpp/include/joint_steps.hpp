#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

class JointStepAudit {
public:
    using Joints = std::array<double,6>;
    JointStepAudit(const Joints& initial,double limit) : previous_(initial),limit_(limit) {
        if(!std::isfinite(limit_) || limit_<=0)throw std::runtime_error("Invalid joint step limit");
        for(double value:initial)if(!std::isfinite(value))throw std::runtime_error("Invalid start joint angle");
    }
    void add(const Joints& target,const std::string& segment) {
        for(std::size_t i=0;i<target.size();++i) {
            if(!std::isfinite(target[i]))throw std::runtime_error("Invalid target joint angle");
            const double delta=std::abs(target[i]-previous_[i]);
            if(delta>limit_)
                throw std::runtime_error(segment+": J"+std::to_string(i+1)+" exceeds "+std::to_string(limit_)+" deg in one move");
            if(delta>largest_) {largest_=delta;joint_=i+1;segment_=segment;}
        }
        previous_=target;
    }
    double largest() const {return largest_;}
    std::size_t joint() const {return joint_;}
    const std::string& segment() const {return segment_;}
private:
    Joints previous_{};
    double limit_=0,largest_=0;
    std::size_t joint_=0;
    std::string segment_;
};
