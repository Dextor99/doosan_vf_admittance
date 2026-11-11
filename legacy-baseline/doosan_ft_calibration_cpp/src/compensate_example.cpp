// This executable shows the API to use in a future Doosan + sensor control loop.
// It does NOT send robot commands or read physical sensors.
#include "ft_compensator.hpp"
#include <iostream>
#include <iomanip>
int main(int argc,char** argv){
    if(argc!=2){std::cerr<<"Usage: ./ft_compensate_example <model.txt>\n";return 2;}
    try{
        const auto params=ftcal::load_parameters(argv[1]);
        const ftcal::Compensator compensation(params);
        // Replace with synchronized, real, calibrated hardware readings.
        ftcal::Quat q_BS={0,0,0,1}; // sensor axes aligned with robot base
        ftcal::Wrench raw=compensation.predicted_no_contact(q_BS); // demo no-contact sample
        const auto contact=compensation.compensate(raw,q_BS);
        std::cout<<"Compensated no-contact wrench [Fx,Fy,Fz,Mx,My,Mz]:\n";
        for(double v:contact)std::cout<<std::setprecision(8)<<v<<" ";
        std::cout<<"\n";
        return 0;
    }catch(const std::exception& e){std::cerr<<"ERROR: "<<e.what()<<"\n";return 1;}
}
