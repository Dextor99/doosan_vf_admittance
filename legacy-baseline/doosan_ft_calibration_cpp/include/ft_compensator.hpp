#pragma once
// Portable static gravity + constant bias compensator for a later Doosan C++ loop.
// No ROS, no SDK dependencies. Input raw wrench and q_BS must be synchronized.
#include <array>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <algorithm>

namespace ftcal {
using Vec3=std::array<double,3>;
using Quat=std::array<double,4>; // qx, qy, qz, qw, SENSOR -> BASE
using Wrench=std::array<double,6>; // Fx Fy Fz Mx My Mz (N, Nm)

struct Parameters {
    double signed_mass = 0;  // signed force convention, kg
    Vec3 signed_first_moment{}; // kg*m times sign
    Wrench bias{};
};

inline Vec3 cross(const Vec3& a,const Vec3& b) {
    return {a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]};
}

inline Vec3 gravity_in_sensor(Quat q) {
    double n=0;for(double v:q){if(!std::isfinite(v))throw std::runtime_error("Invalid quaternion");n+=v*v;}
    n=std::sqrt(n);
    if(n<0.98 || n>1.02)throw std::runtime_error("q_BS is not a unit quaternion");
    for(double& v:q)v/=n;
    const Vec3 xyz={q[0],q[1],q[2]};
    const Vec3 g={0,0,-9.80665};
    const auto c1=cross(xyz,g),c2=cross(xyz,c1);
    return {g[0]-2*q[3]*c1[0]+2*c2[0],
            g[1]-2*q[3]*c1[1]+2*c2[1],
            g[2]-2*q[3]*c1[2]+2*c2[2]};
}

inline Parameters load_parameters(const std::string& path) {
    std::ifstream in(path);
    if(!in)throw std::runtime_error("Cannot open calibration parameters: "+path);
    const std::array<std::string,10> keys={"signed_mass","hx","hy","hz","bfx","bfy","bfz","bmx","bmy","bmz"};
    std::array<double,10> v{};std::array<bool,10> seen{};
    std::string line;
    while(std::getline(in,line)){
        if(!line.empty() && line.back()=='\r')line.pop_back();
        if(line.empty()||line[0]=='#')continue;
        auto eq=line.find('=');if(eq==std::string::npos)throw std::runtime_error("Invalid parameter line");
        auto key=line.substr(0,eq);auto it=std::find(keys.begin(),keys.end(),key);
        if(it==keys.end())throw std::runtime_error("Unknown key: "+key);
        int idx=static_cast<int>(it-keys.begin());
        if(seen[idx])throw std::runtime_error("Duplicate key: "+key);
        seen[idx]=true;
        std::string val=line.substr(eq+1);std::size_t end=0;
        try{v[idx]=std::stod(val,&end);}catch(...){throw std::runtime_error("Bad parameter: "+key);}
        if(end!=val.size()||!std::isfinite(v[idx]))throw std::runtime_error("Bad parameter: "+key);
    }
    for(bool ok:seen)if(!ok)throw std::runtime_error("Missing parameter");
    if(std::abs(v[0])<1e-6)throw std::runtime_error("Invalid signed mass");
    Parameters p;p.signed_mass=v[0];
    for(int i=0;i<3;++i)p.signed_first_moment[i]=v[1+i];
    for(int i=0;i<6;++i)p.bias[i]=v[4+i];
    return p;
}

class Compensator {
public:
    explicit Compensator(Parameters p):p_(p){}
    Wrench predicted_no_contact(Quat q_BS) const {
        auto g=gravity_in_sensor(q_BS);
        auto moment=cross(p_.signed_first_moment,g);
        Wrench pred{};
        for(int j=0;j<3;++j){
            pred[j]=p_.signed_mass*g[j]+p_.bias[j];
            pred[3+j]=moment[j]+p_.bias[3+j];
        }
        return pred;
    }
    Wrench compensate(Wrench raw,Quat q_BS) const {
        for(double v:raw) if(!std::isfinite(v))throw std::runtime_error("Invalid wrench");
        const auto pred=predicted_no_contact(q_BS);
        for(int i=0;i<6;++i)raw[i]-=pred[i];
        return raw;
    }
private:
    Parameters p_;
};
} // namespace ftcal
