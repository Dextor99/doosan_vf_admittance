// Motion-capable calibration: display the live path and require Enter before control.
// Controller safety functions and an independent emergency stop remain essential.
#include "ft_acquisition.hpp"
#include "joint_steps.hpp"
#include "motion_completion.hpp"
#include "servo_enable_policy.hpp"
#include "session_status.hpp"
#include <sys/wait.h>
#include <cstring>
#include <map>
#include <set>
#include <tuple>
#include <vector>

namespace {
using Joints = std::array<double, 6>;
using TaskPose = std::array<double, 6>;
struct RecordedPose { Joints joints{}; TaskPose task{}; };
struct Settings {
    std::string ip, device;
    Joints low{}, high{}, center{}, offsets{};
    int version=1;
    double j4=0, j5=0, velocity=0, acceleration=0, settle=0, max_step=5;
};
struct Waypoint { std::string split; Joints joints{}; TaskPose task{}; int solution=2; };
struct Plan { Settings s; Joints home{}; TaskPose home_task{}; int home_solution=2; std::vector<Waypoint> points; };

std::atomic<int> access_state{-1};
std::atomic<int> robot_state{STATE_LAST};
std::atomic<int> safety_state{SAFETY_STATE_LAST};
std::atomic<std::uint64_t> moving_events{0};
std::atomic<std::uint64_t> warning_events{0};
struct DiagnosticEvent { Clock::time_point at{}; int kind=0; int value=0; };
std::mutex diagnostics_mutex;
std::array<DiagnosticEvent,64> diagnostics{};
std::size_t diagnostic_count=0;
std::array<LOG_ALARM,8> callback_alarms{};
std::size_t callback_alarm_count=0;
void remember_event(int kind,int value) {
    std::lock_guard<std::mutex> lock(diagnostics_mutex);
    diagnostics[diagnostic_count%diagnostics.size()]={Clock::now(),kind,value};
    ++diagnostic_count;
}
void on_access(const MONITORING_ACCESS_CONTROL state) {
    access_state.store(static_cast<int>(state));remember_event(1,static_cast<int>(state));
}
void on_state(const ROBOT_STATE state) {
    if(state==STATE_MOVING)moving_events.fetch_add(1);
    robot_state.store(static_cast<int>(state));remember_event(0,static_cast<int>(state));
}
void on_safety_state(const SAFETY_STATE state) {
    safety_state.store(static_cast<int>(state));remember_event(3,static_cast<int>(state));
}
void on_alarm(LPLOG_ALARM alarm) {
    if(!alarm)return;
    std::lock_guard<std::mutex> lock(diagnostics_mutex);
    if(alarm->_iLevel>=LOG_LEVEL_SYSWARN)warning_events.fetch_add(1);
    callback_alarms[callback_alarm_count%callback_alarms.size()]=*alarm;
    ++callback_alarm_count;
    diagnostics[diagnostic_count%diagnostics.size()]={Clock::now(),2,static_cast<int>(alarm->_iIndex)};
    ++diagnostic_count;
}
const char* state_name(int state) {
    switch(state) {
        case STATE_INITIALIZING:return "INITIALIZING";case STATE_STANDBY:return "STANDBY";
        case STATE_MOVING:return "MOVING";case STATE_SAFE_OFF:return "SAFE_OFF";
        case STATE_TEACHING:return "TEACHING";case STATE_SAFE_STOP:return "SAFE_STOP";
        case STATE_EMERGENCY_STOP:return "EMERGENCY_STOP";case STATE_HOMMING:return "HOMMING";
        case STATE_RECOVERY:return "RECOVERY";case STATE_SAFE_STOP2:return "SAFE_STOP2";
        case STATE_SAFE_OFF2:return "SAFE_OFF2";case STATE_NOT_READY:return "NOT_READY";
        default:return "UNKNOWN";
    }
}
const char* safety_state_name(int state) {
    switch(state) {
        case SAFETY_STATE_BP_START:return "BP_START";case SAFETY_STATE_BP_INIT:return "BP_INIT";
        case SAFETY_STATE_VD_STO:return "VD_STO";case SAFETY_STATE_VD_SOS:return "VD_SOS";
        case SAFETY_STATE_JH_SOS:return "JH_SOS";case SAFETY_STATE_JH_MOVE:return "JH_MOVE";
        case SAFETY_STATE_HG_MOVE:return "HG_MOVE";case SAFETY_STATE_RV_SOS:return "RV_SOS";
        case SAFETY_STATE_RV_MOVE:return "RV_MOVE";case SAFETY_STATE_RV_BACK:return "RV_BACK";
        case SAFETY_STATE_RV_HG_MOVE:return "RV_HG_MOVE";case SAFETY_STATE_SW_SOS:return "SW_SOS";
        case SAFETY_STATE_SW_RUN:return "SW_RUN";case SAFETY_STATE_CW_SOS:return "CW_SOS";
        case SAFETY_STATE_CW_RUN:return "CW_RUN";case SAFETY_STATE_CM_RUN:return "CM_RUN";
        case SAFETY_STATE_AM_RUN:return "AM_RUN";case SAFETY_STATE_DRL_JH_SOS:return "DRL_JH_SOS";
        case SAFETY_STATE_DRL_HG_MOVE:return "DRL_HG_MOVE";default:return "UNKNOWN";
    }
}
void print_diagnostics() {
    std::array<DiagnosticEvent,64> events{};std::size_t count=0;
    std::array<LOG_ALARM,8> alarms{};std::size_t alarm_count=0;
    {
        std::lock_guard<std::mutex> lock(diagnostics_mutex);
        events=diagnostics;count=diagnostic_count;alarms=callback_alarms;alarm_count=callback_alarm_count;
    }
    const auto now=Clock::now();
    const int current_state=robot_state.load();
    std::cerr<<"Diagnostic snapshot: state="<<state_name(current_state)<<" ("<<current_state
             <<"), access="<<access_state.load()<<", disconnected="<<robot_disconnected.load()<<"\n";
    const auto pose_snapshot=snapshot();
    if(pose_snapshot.received){
        std::cerr<<"Last joint positions [deg]:";
        for(double value:pose_snapshot.joints)std::cerr<<' '<<value;
        std::cerr<<"; age="<<std::chrono::duration_cast<std::chrono::milliseconds>(now-pose_snapshot.received_at).count()<<" ms\n";
    }
    const auto first=count>events.size()?count-events.size():0;
    for(std::size_t i=first;i<count;++i){
        const auto& event=events[i%events.size()];
        const auto age=std::chrono::duration_cast<std::chrono::milliseconds>(now-event.at).count();
        std::cerr<<"Event "<<age<<" ms ago: ";
        if(event.kind==0)std::cerr<<"robot state "<<state_name(event.value)<<" ("<<event.value<<')';
        else if(event.kind==1)std::cerr<<"control-right event "<<event.value;
        else if(event.kind==2)std::cerr<<"alarm index "<<event.value;
        else std::cerr<<"controller safety state "<<safety_state_name(event.value)<<" ("<<event.value<<')';
        std::cerr<<'\n';
    }
    if(alarm_count){
        const auto alarm_first=alarm_count>alarms.size()?alarm_count-alarms.size():0;
        for(std::size_t j=alarm_first;j<alarm_count;++j){
            const auto& alarm=alarms[j%alarms.size()];
            std::cerr<<"Alarm callback: level="<<static_cast<unsigned>(alarm._iLevel)
                     <<" group="<<static_cast<unsigned>(alarm._iGroup)<<" index="<<alarm._iIndex<<'\n';
            for(int i=0;i<3;++i)
                std::cerr<<"  param"<<i<<'='<<std::string(alarm._szParam[i],strnlen(alarm._szParam[i],sizeof(alarm._szParam[i])))<<'\n';
        }
    } else std::cerr<<"No alarm callback was received on this connection.\n";
}

std::string trim(const std::string& text) {
    const auto begin=text.find_first_not_of(" \t\r\n");
    if(begin==std::string::npos) return {};
    const auto end=text.find_last_not_of(" \t\r\n");
    return text.substr(begin,end-begin+1);
}
std::vector<std::string> fields(const std::string& text) {
    std::vector<std::string> result;
    std::stringstream in(text); std::string item;
    while(std::getline(in,item,',')) result.push_back(trim(item));
    if(!text.empty() && text.back()==',') result.emplace_back();
    return result;
}
Joints joints(const std::string& text) {
    const auto parts=fields(text);
    if(parts.size()!=6) throw std::runtime_error("Expected six comma-separated joint angles: "+text);
    Joints out{}; for(int i=0;i<6;++i) out[i]=number(parts[i].c_str());
    return out;
}
std::string joint_text(const Joints& v) {
    std::ostringstream out; out<<std::setprecision(17);
    for(int i=0;i<6;++i){if(i)out<<',';out<<v[i];}
    return out.str();
}
std::map<std::string,std::string> read_properties(const std::string& text) {
    std::map<std::string,std::string> out;
    std::istringstream in(text); std::string line;
    while(std::getline(in,line)) {
        line=trim(line); if(line.empty()||line[0]=='#') continue;
        const auto eq=line.find('=');
        if(eq==std::string::npos) throw std::runtime_error("Expected key=value: "+line);
        const auto key=trim(line.substr(0,eq)), value=trim(line.substr(eq+1));
        if(key.empty()||value.empty()||!out.emplace(key,value).second)
            throw std::runtime_error("Empty or duplicate setting: "+key);
    }
    return out;
}
std::string slurp(const std::string& path) {
    std::ifstream in(path,std::ios::binary); if(!in) throw std::runtime_error("Cannot open: "+path);
    std::ostringstream out; out<<in.rdbuf(); if(in.bad())throw std::runtime_error("Read failed: "+path);
    return out.str();
}
std::string required(const std::map<std::string,std::string>& m,const std::string& k) {
    const auto it=m.find(k); if(it==m.end())throw std::runtime_error("Missing setting: "+k);
    return it->second;
}
void validate_settings(const Settings& s) {
    in_addr a{}; if(inet_pton(AF_INET,s.ip.c_str(),&a)!=1) throw std::runtime_error("Invalid robot IPv4 address");
    if(s.device.empty() || s.device[0]!='/')throw std::runtime_error("Serial device must be an absolute path");
    if(s.version==1 && (s.j4<1 || s.j4>5 || s.j5<1 || s.j5>5))
        throw std::runtime_error("J4 and J5 offsets must be 1..5 degrees; these caps do not guarantee collision clearance");
    if(s.version>=2){
        bool any=false;
        for(double offset:s.offsets){
            if(!std::isfinite(offset) || offset<0 || offset>(s.version>=3?180:5))
                throw std::runtime_error("Invalid recorded-plan joint offset");
            if(offset>0)any=true;
        }
        if(!any)throw std::runtime_error("At least one allowed offset must be positive");
    }
    if(!std::isfinite(s.max_step) || s.max_step<=0 || s.max_step>(s.version>=3?180:5))
        throw std::runtime_error("Invalid maximum per-joint step");
    if(s.velocity<=0 || s.velocity>5 || s.acceleration<=0 || s.acceleration>10)
        throw std::runtime_error("Velocity must be >0..5 deg/s and acceleration >0..10 deg/s^2; select site-reviewed values");
    if(s.settle<2 || s.settle>30)throw std::runtime_error("Settling time must be 2..30 seconds");
    for(int i=0;i<6;++i) {
        if(!std::isfinite(s.low[i]) || !std::isfinite(s.high[i]) || s.low[i]>=s.high[i])
            throw std::runtime_error("Invalid site joint limits for J"+std::to_string(i+1));
        if(s.low[i]<-720 || s.high[i]>720) throw std::runtime_error("Joint limits outside sanity range (-720..720 deg)");
    }
}
Settings parse_settings(const std::string& path, bool recorded=false) {
    auto m=read_properties(slurp(path));
    std::set<std::string> expected={"robot_ip","serial_device","joint_min_deg","joint_max_deg","velocity_deg_s","acceleration_deg_s2","settle_seconds"};
    if(recorded)expected.insert("joint_max_offset_deg");
    else {expected.insert("offset_j4_deg");expected.insert("offset_j5_deg");}
    for(const auto& p:m) if(!expected.count(p.first))throw std::runtime_error("Unknown setting: "+p.first);
    Settings s;s.version=recorded?2:1;s.ip=required(m,"robot_ip");s.device=required(m,"serial_device");
    s.low=joints(required(m,"joint_min_deg"));s.high=joints(required(m,"joint_max_deg"));
    if(recorded)s.offsets=joints(required(m,"joint_max_offset_deg"));
    else {s.j4=number(required(m,"offset_j4_deg").c_str());s.j5=number(required(m,"offset_j5_deg").c_str());}
    s.velocity=number(required(m,"velocity_deg_s").c_str());
    s.acceleration=number(required(m,"acceleration_deg_s2").c_str());
    s.settle=number(required(m,"settle_seconds").c_str());
    validate_settings(s);return s;
}
void check_limits(const Joints& j,const Settings& s,double tolerance=0) {
    for(int i=0;i<6;++i)
        if(!std::isfinite(j[i]) || j[i]<s.low[i]-tolerance || j[i]>s.high[i]+tolerance)
            throw std::runtime_error("J"+std::to_string(i+1)+" is outside site limits");
}
std::string plan_text(const Plan& p) {
    const auto& s=p.s;std::ostringstream out;out<<std::setprecision(17);
    out<<"# FT auto plan; review every path segment before executing\n"
       <<"version="<<s.version<<"\nrobot_ip="<<s.ip<<"\nserial_device="<<s.device
       <<"\njoint_min_deg="<<joint_text(s.low)<<"\njoint_max_deg="<<joint_text(s.high)
       <<"\ncenter_deg="<<joint_text(s.center);
    if(s.version==1)out<<"\noffset_j4_deg="<<s.j4<<"\noffset_j5_deg="<<s.j5;
    else out<<"\njoint_max_offset_deg="<<joint_text(s.offsets);
    if(s.version>=3)out<<"\nmax_joint_step_deg="<<s.max_step;
    if(s.version>=4)out<<"\nhome_deg="<<joint_text(p.home);
    if(s.version>=5)out<<"\nhome_task="<<joint_text(p.home_task)<<"\nhome_solution="<<p.home_solution;
    out<<"\nvelocity_deg_s="<<s.velocity<<"\nacceleration_deg_s2="<<s.acceleration<<"\nsettle_seconds="<<s.settle<<'\n';
    for(std::size_t i=0;i<p.points.size();++i){
        std::ostringstream k;k<<"pose"<<std::setw(2)<<std::setfill('0')<<i+1;
        out<<k.str()<<'='<<p.points[i].split<<','<<joint_text(p.points[i].joints)<<'\n';
        if(s.version>=5)out<<k.str()<<"_task="<<joint_text(p.points[i].task)<<'\n'<<k.str()<<"_solution="<<p.points[i].solution<<'\n';
    }
    return out.str();
}
std::string fingerprint(const std::string& data) {
    uint64_t h=14695981039346656037ULL;
    for(unsigned char b:data){h^=b;h*=1099511628211ULL;}
    std::ostringstream out;out<<std::hex<<std::setfill('0')<<std::setw(16)<<h;return out.str();
}
Plan parse_plan(const std::string& text) {
    auto m=read_properties(text);const auto version=required(m,"version");
    if(version!="1"&&version!="2"&&version!="3"&&version!="4"&&version!="5")throw std::runtime_error("Unsupported plan version");
    std::set<std::string> base={"version","robot_ip","serial_device","joint_min_deg","joint_max_deg","center_deg","velocity_deg_s","acceleration_deg_s2","settle_seconds"};
    if(version=="1"){base.insert("offset_j4_deg");base.insert("offset_j5_deg");}
    else base.insert("joint_max_offset_deg");
    if(version=="3"||version=="4"||version=="5")base.insert("max_joint_step_deg");
    if(version=="4"||version=="5")base.insert("home_deg");
    if(version=="5"){base.insert("home_task");base.insert("home_solution");}
    std::set<std::string> allowed=base;
    for(int i=1;i<=20;++i){std::ostringstream k;k<<"pose"<<std::setw(2)<<std::setfill('0')<<i;allowed.insert(k.str());if(version=="5"){allowed.insert(k.str()+"_task");allowed.insert(k.str()+"_solution");}}
    for(const auto& kv:m)if(!allowed.count(kv.first))throw std::runtime_error("Unknown plan key: "+kv.first);
    Plan p;auto& s=p.s;s.version=std::stoi(version);
    s.ip=required(m,"robot_ip");s.device=required(m,"serial_device");
    s.low=joints(required(m,"joint_min_deg"));s.high=joints(required(m,"joint_max_deg"));s.center=joints(required(m,"center_deg"));
    if(s.version==1){s.j4=number(required(m,"offset_j4_deg").c_str());s.j5=number(required(m,"offset_j5_deg").c_str());}
    else s.offsets=joints(required(m,"joint_max_offset_deg"));
    if(s.version>=3)s.max_step=number(required(m,"max_joint_step_deg").c_str());
    s.velocity=number(required(m,"velocity_deg_s").c_str());s.acceleration=number(required(m,"acceleration_deg_s2").c_str());
    s.settle=number(required(m,"settle_seconds").c_str());validate_settings(s);check_limits(s.center,s);
    int train=0,valid=0;Joints previous=s.center;
    if(s.version>=4){
        p.home=joints(required(m,"home_deg"));check_limits(p.home,s);
        for(int j=0;j<6;++j){
            if(std::abs(p.home[j]-s.center[j])>s.offsets[j]+1e-7 || std::abs(p.home[j]-s.center[j])>s.max_step+1e-6)
                throw std::runtime_error("Home exceeds start offset or maximum joint step");
        }
        previous=p.home;
    }
    if(s.version==5){
        p.home_task=joints(required(m,"home_task"));
        p.home_solution=static_cast<int>(number(required(m,"home_solution").c_str()));
        if(p.home_solution<0||p.home_solution>7)throw std::runtime_error("Invalid home IK solution");
    }
    for(int i=1;i<=20;++i) {
        std::ostringstream k;k<<"pose"<<std::setw(2)<<std::setfill('0')<<i;
        const auto v=fields(required(m,k.str()));if(v.size()!=7)throw std::runtime_error("Bad waypoint: "+k.str());
        Waypoint w;w.split=v[0];if(w.split=="train")++train;else if(w.split=="valid")++valid;else throw std::runtime_error("Bad split: "+w.split);
        for(int j=0;j<6;++j) w.joints[j]=number(v[j+1].c_str());
        if(s.version==5){w.task=joints(required(m,k.str()+"_task"));w.solution=static_cast<int>(number(required(m,k.str()+"_solution").c_str()));if(w.solution<0||w.solution>7)throw std::runtime_error("Invalid waypoint IK solution: "+k.str());}
        check_limits(w.joints,s);
        for(int j=0;j<6;++j) {
            const double max_offset=(s.version==1?(j==3?s.j4:(j==4?s.j5:0)):s.offsets[j]);
            if(std::abs(w.joints[j]-s.center[j])>max_offset+1e-7)throw std::runtime_error("Waypoint exceeds configured offset: "+k.str());
            if(std::abs(w.joints[j]-previous[j])>s.max_step+1e-6)throw std::runtime_error("Waypoint exceeds configured maximum joint step: "+k.str());
        }
        previous=w.joints;p.points.push_back(w);
    }
    if(s.version>=4)for(int j=0;j<6;++j)
        if(std::abs(p.home[j]-previous[j])>s.max_step+1e-6)throw std::runtime_error("Return to home exceeds maximum joint step");
    if(train!=15||valid!=5)throw std::runtime_error("Plan must have 15 train and 5 valid poses");
    for(std::size_t i=0;i<p.points.size();++i)for(std::size_t j=0;j<i;++j){
        bool same=true;for(int k=0;k<6;++k)if(std::abs(p.points[i].joints[k]-p.points[j].joints[k])>1e-6)same=false;
        if(same)throw std::runtime_error("Plan contains duplicate joint poses");
    }
    return p;
}
void print_plan(const Plan& p,const std::string& hash) {
    if(!hash.empty())std::cout<<"Plan check code: "<<hash<<'\n';
    std::cout<<"Robot "<<p.s.ip<<"  serial "<<p.s.device
             <<"\nSpeed "<<p.s.velocity<<" deg/s; acceleration "<<p.s.acceleration
             <<" deg/s^2; settle "<<p.s.settle<<" s; max joint step "<<p.s.max_step<<" deg.\nCenter: "<<joint_text(p.s.center)<<'\n';
    for(std::size_t i=0;i<p.points.size();++i)
        std::cout<<std::setw(2)<<i+1<<' '<<p.points[i].split<<"  "<<joint_text(p.points[i].joints)<<'\n';
    std::cout<<"Review EVERY segment, including the path from the initial pose. Joint limits and small angles do not check collisions.\n";
    if(p.s.version>=4)std::cout<<"Home before pose 1 and after pose 20: "<<joint_text(p.home)<<"\nReview start -> home, home -> pose 1 and pose 20 -> home.\n";
    if(p.s.version>=5){
        std::cout<<"MoveJX home pose [mm,deg]: "<<joint_text(p.home_task)<<" solution="<<p.home_solution<<"\n";
        for(std::size_t i=0;i<p.points.size();++i)
            std::cout<<"MoveJX pose "<<i+1<<" [mm,deg]: "<<joint_text(p.points[i].task)<<" solution="<<p.points[i].solution<<"\n";
    }
}

void check_controller_limits(DRAFramework::CDRFLEx& robot,const Plan& plan) {
    const auto* config=robot.get_safety_configuration();
    if(!config) throw std::runtime_error("Cannot read controller joint safety configuration");
    const std::array<JOINT_RANGE,2> ranges{{config->_tJointRange._Normal,config->_tJointRange._Reduced}};
    for(const auto& range:ranges) {
        for(int i=0;i<6;++i) {
            const double low=range._fMinRange[i],high=range._fMaxRange[i],speed=range._fMaxVelocity[i];
            if(!std::isfinite(low)||!std::isfinite(high)||!std::isfinite(speed)||low>=high||speed<=0)
                throw std::runtime_error("Controller joint limits are unavailable or invalid for J"+std::to_string(i+1));
            if(plan.s.center[i]<low+0.2||plan.s.center[i]>high-0.2)
                throw std::runtime_error("Plan center is too close to a controller limit for J"+std::to_string(i+1));
            if(plan.s.version>=4 && (plan.home[i]<low+0.2||plan.home[i]>high-0.2 ||
                (std::abs(plan.home[i]-plan.s.center[i])>1e-6 && plan.s.velocity>speed)))
                throw std::runtime_error("Home violates controller joint limit or speed for J"+std::to_string(i+1));
            for(const auto& point:plan.points) {
                if(point.joints[i]<low+0.2||point.joints[i]>high-0.2)
                    throw std::runtime_error("Plan target is too close to a controller limit for J"+std::to_string(i+1));
                if(std::abs(point.joints[i]-plan.s.center[i])>1e-6 && plan.s.velocity>speed)
                    throw std::runtime_error("Requested speed exceeds a controller joint limit for J"+std::to_string(i+1));
            }
        }
    }
}
void initialize_direct_sequence(Plan& plan,DRAFramework::CDRFLEx& robot) {
    auto& s=plan.s;
    s.center=snapshot().joints;
    const auto* config=robot.get_safety_configuration();
    if(!config)throw std::runtime_error("Cannot read controller joint limits for direct replay");
    for(int i=0;i<6;++i) {
        s.low[i]=std::max(config->_tJointRange._Normal._fMinRange[i],
                          config->_tJointRange._Reduced._fMinRange[i]);
        s.high[i]=std::min(config->_tJointRange._Normal._fMaxRange[i],
                           config->_tJointRange._Reduced._fMaxRange[i]);
        if(!std::isfinite(s.low[i])||!std::isfinite(s.high[i])||s.low[i]>=s.high[i])
            throw std::runtime_error("Invalid controller joint limits for J"+std::to_string(i+1));
    }
    constexpr double max_step_deg=360.0;
    JointStepAudit steps(s.center,max_step_deg);
    steps.add(plan.home,"current -> home");
    for(std::size_t i=0;i<plan.points.size();++i)
        steps.add(plan.points[i].joints,(i==0?"home":"pose "+std::to_string(i))+" -> pose "+std::to_string(i+1));
    steps.add(plan.home,"pose 20 -> home");
    s.max_step=max_step_deg;
    std::cout<<"Largest single-joint change: "<<steps.largest()<<" deg";
    if(steps.joint())std::cout<<" (J"<<steps.joint()<<", "<<steps.segment()<<")";
    std::cout<<"; limit 360 deg.\n";
    std::cout<<"Current -> home delta [deg]:";
    for(int i=0;i<6;++i)std::cout<<' '<<plan.home[i]-s.center[i];
    std::cout<<'\n';
}

void setup_robot(DRAFramework::CDRFLEx& robot) {
    robot.set_on_monitoring_data_ex(on_robot_data_ex);robot.set_on_disconnected(on_robot_disconnected);
    robot.set_on_monitoring_access_control(on_access);robot.set_on_monitoring_state(on_state);
    robot.set_on_log_alarm(on_alarm);
    robot.set_on_monitoring_safety_state(on_safety_state);
}
void wait_for_pose() {
    const auto deadline=Clock::now()+std::chrono::seconds(5);
    while(!stop_requested&&!snapshot().received&&!robot_disconnected.load()&&Clock::now()<deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if(stop_requested)throw std::runtime_error("Interrupted");
    const auto current=snapshot();
    if(!current.received && !current.invalid && !robot_disconnected.load())
        throw std::runtime_error("No version 1 robot monitoring pose received within 5 seconds");
    require_fresh_pose(current);
}
struct Connection {
    DRAFramework::CDRFLEx& robot; bool open=false;
    explicit Connection(DRAFramework::CDRFLEx& r):robot(r){}
    ~Connection(){if(open)robot.close_connection();}
};
void open_robot(DRAFramework::CDRFLEx& robot,Connection& connection,const std::string& ip,bool allow_safe_off=false) {
    setup_robot(robot);if(!robot.open_connection(ip.c_str(),12345))throw std::runtime_error("Robot connection failed");
    connection.open=true;
    if(!robot.setup_monitoring_version(1))
        throw std::runtime_error("Failed to configure Doosan monitoring data version 1");
    wait_for_pose();
    auto state=robot.get_robot_state();robot_state.store(static_cast<int>(state));
    const auto deadline=Clock::now()+std::chrono::seconds(15);
    while((state==STATE_INITIALIZING || state==STATE_NOT_READY) && !stop_requested && Clock::now()<deadline){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        state=robot.get_robot_state();robot_state.store(static_cast<int>(state));
    }
    if(state==STATE_SAFE_OFF && !allow_safe_off)
        throw std::runtime_error("Robot is SAFE_OFF. Automatic servo reset is disabled; inspect and clear the cause on the teach pendant first.");
    if(state!=STATE_STANDBY && !(allow_safe_off && state==STATE_SAFE_OFF))
        throw std::runtime_error("Robot is not STANDBY/eligible SAFE_OFF; inspect the teach pendant (state="+std::to_string(static_cast<int>(state))+")");
    const auto mode=robot.get_robot_mode();
    if(mode!=ROBOT_MODE_MANUAL && mode!=ROBOT_MODE_AUTONOMOUS)throw std::runtime_error("Unsupported robot operating mode");
    std::cout<<"Robot connected; mode="<<(mode==ROBOT_MODE_MANUAL?"MANUAL":"AUTONOMOUS")
             <<", state="<<(state==STATE_STANDBY?"STANDBY":"SAFE_OFF")<<".\n";
}
void inspect_robot(const std::string& ip) {
    in_addr address{};
    if(inet_pton(AF_INET,ip.c_str(),&address)!=1) throw std::runtime_error("Invalid robot IPv4 address");
    mounting={0,0,0,1};DRAFramework::CDRFLEx robot;Connection connection(robot);
    setup_robot(robot);
    if(!robot.open_connection(ip.c_str(),12345))throw std::runtime_error("Robot connection failed");
    connection.open=true;
    if(!robot.setup_monitoring_version(1))
        throw std::runtime_error("Failed to configure Doosan monitoring data version 1");
    wait_for_pose();
    const auto state=robot.get_robot_state();
    const auto mode=robot.get_robot_mode();
    std::cout<<"Robot state="<<state_name(static_cast<int>(state))<<" ("<<static_cast<int>(state)<<"); mode code="
             <<static_cast<int>(mode)<<" (MANUAL=0, AUTONOMOUS=1).\n";
    const auto system=robot.get_robot_system();
    std::cout<<"Robot system code="<<static_cast<int>(system)<<" (REAL=0, VIRTUAL=1).\n";
    std::cout<<"Current joint angles [deg]: "<<joint_text(snapshot().joints)<<'\n';
    print_axes_in_base(snapshot().q);
    const auto* alarm=robot.get_last_alarm();
    if(alarm && (alarm->_iLevel!=0 || alarm->_iGroup!=0 || alarm->_iIndex!=0)) {
        std::cout<<"SDK last alarm (may be historical): level="<<static_cast<unsigned>(alarm->_iLevel)
                 <<" group="<<static_cast<unsigned>(alarm->_iGroup)<<" index="<<alarm->_iIndex<<'\n';
        for(int i=0;i<3;++i)
            std::cout<<"  param"<<i<<'='<<std::string(alarm->_szParam[i],strnlen(alarm->_szParam[i],sizeof(alarm->_szParam[i])))<<'\n';
    } else std::cout<<"SDK returned no usable last-alarm details; this does not prove no fault occurred.\n";
    const auto* config=robot.get_safety_configuration();
    if(!config) { std::cerr<<"Controller safety configuration unavailable.\n"; return; }
    const auto print_range=[](const char* name,const JOINT_RANGE& range){
        std::cout<<name<<" controller joint settings [deg; deg/s]:\n";
        for(int i=0;i<6;++i)
            std::cout<<"  J"<<i+1<<"  min="<<range._fMinRange[i]<<"  max="<<range._fMaxRange[i]
                     <<"  max_velocity="<<range._fMaxVelocity[i]<<'\n';
    };
    print_range("Normal",config->_tJointRange._Normal);
    print_range("Reduced",config->_tJointRange._Reduced);
    std::cout<<"Controller limits do not establish collision-free paths or account for unmodelled equipment.\n";
}
void make_plan(const std::string& config,const std::string& file) {
    Settings s=parse_settings(config);
    if(std::filesystem::exists(file))throw std::runtime_error("Plan file exists; choose a new name");
    mounting={0,0,0,1};DRAFramework::CDRFLEx robot;Connection connection(robot);
    open_robot(robot,connection,s.ip);s.center=snapshot().joints;check_limits(s.center,s);
    Plan plan;plan.s=s;
    const std::array<double,4> x{{-1,-1.0/3,1.0/3,1}};
    const std::array<double,5> y{{-1,-0.5,0,0.5,1}};
    const std::set<std::pair<int,int>> validation{{0,1},{1,3},{2,0},{3,2},{3,4}};
    for(int i=0;i<4;++i)for(int seq=0;seq<5;++seq){
        const int j=(i%2==0?seq:4-seq);Waypoint w;
        w.split=validation.count({i,j})?"valid":"train";w.joints=s.center;
        w.joints[3]+=x[i]*s.j4;w.joints[4]+=y[j]*s.j5;plan.points.push_back(w);
    }
    const auto text=plan_text(plan);const auto checked=parse_plan(text);
    check_controller_limits(robot,checked);
    std::ofstream out(file,std::ios::binary);if(!out || !(out<<text) || !(out.flush()))throw std::runtime_error("Failed to write plan");
    print_axes_in_base(snapshot().q);print_plan(checked,fingerprint(text));
    std::cout<<"Plan saved to "<<file<<". Review the entire plan before running. No motion was commanded.\n";
}
std::vector<RecordedPose> load_recorded_poses(const std::string& path,const std::string& robot_ip) {
    std::ifstream in(path);if(!in)throw std::runtime_error("Cannot open recorded poses: "+path);
    std::string line;
    if(!std::getline(in,line))throw std::runtime_error("Recorded pose CSV is empty");
    if(!line.empty() && line.back()=='\r')line.pop_back();
    const std::string header="index,stamp_s,robot_ip,j1_deg,j2_deg,j3_deg,j4_deg,j5_deg,j6_deg,flange_x_mm,flange_y_mm,flange_z_mm,flange_a_deg,flange_b_deg,flange_c_deg";
    if(line!=header)throw std::runtime_error("Invalid recorded pose CSV header");
    std::vector<RecordedPose> result;
    while(std::getline(in,line)) {
        if(!line.empty() && line.back()=='\r')line.pop_back();
        if(line.empty())continue;
        const auto col=fields(line);
        if(col.size()!=15)throw std::runtime_error("Invalid recorded pose CSV row");
        if(number(col[0].c_str())!=static_cast<double>(result.size()+1))
            throw std::runtime_error("Recorded pose indices must be consecutive, starting at 1");
        (void)number(col[1].c_str());
        if(col[2]!=robot_ip)throw std::runtime_error("Recorded robot IP differs from configured robot IP");
        RecordedPose pose;
        for(int i=0;i<6;++i){pose.joints[i]=number(col[3+i].c_str());pose.task[i]=number(col[9+i].c_str());}
        result.push_back(pose);
        if(result.size()>20)throw std::runtime_error("Recorded CSV must contain exactly 20 poses");
    }
    if(in.bad())throw std::runtime_error("Failed reading recorded poses");
    if(result.size()!=20)throw std::runtime_error("Recorded CSV must contain exactly 20 poses; found "+std::to_string(result.size()));
    return result;
}
int solution_for_pose(DRAFramework::CDRFLEx& robot,const TaskPose& task,const Joints& expected) {
    float target[6];for(int i=0;i<6;++i)target[i]=static_cast<float>(task[i]);
    for(int solution=0;solution<8;++solution){
        auto* inverse=robot.ikin(target,static_cast<unsigned char>(solution),COORDINATE_SYSTEM_BASE);
        if(!inverse)continue;
        bool matches=true;
        for(int j=0;j<6;++j)
            if(std::abs(std::remainder(static_cast<double>(inverse->_fPosition[j])-expected[j],360.0))>0.5)matches=false;
        if(matches)return solution;
    }
    throw std::runtime_error("Recorded Cartesian pose has no IK solution matching its taught joint pose");
}
void make_recorded_plan(const std::string& config,const std::string& poses_path,const std::string& file) {
    Settings s=parse_settings(config,true);
    const auto recorded=load_recorded_poses(poses_path,s.ip);
    if(std::filesystem::exists(file))throw std::runtime_error("Plan file exists; choose a new name");
    s.center=recorded.front().joints;Plan plan;plan.s=s;
    for(std::size_t i=0;i<recorded.size();++i){
        Waypoint w;w.joints=recorded[i].joints;w.task=recorded[i].task;w.split=((i+1)%4==0?"valid":"train");plan.points.push_back(w);
    }
    const auto text=plan_text(plan);const auto checked=parse_plan(text);
    mounting={0,0,0,1};DRAFramework::CDRFLEx robot;Connection connection(robot);
    open_robot(robot,connection,s.ip);check_controller_limits(robot,checked);
    const auto current=snapshot().joints;
    bool near_start=true;for(int i=0;i<6;++i)if(std::abs(current[i]-s.center[i])>0.2)near_start=false;
    if(!near_start)std::cout<<"Current joints differ from recorded pose 1; execution requires returning to pose 1 safely.\n";
    std::ofstream out(file,std::ios::binary);
    if(!out || !(out<<text) || !(out.flush()))throw std::runtime_error("Failed to write plan");
    print_plan(checked,fingerprint(text));
    std::cout<<"Plan saved from recorded poses to "<<file<<". Review the entire path; no motion was commanded.\n";
}
std::string make_confirmed_plan(const std::string& poses_path,const std::string& file,bool use_home=false,
                                bool require_standby=false) {
    if(!file.empty() && std::filesystem::exists(file))throw std::runtime_error("Plan file exists; choose a new name");
    Settings s;s.version=use_home?5:3;s.ip="192.168.123.156";
    s.device="/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00TFOX-if00-port0";
    s.velocity=2;s.acceleration=4;s.settle=3;
    const auto recorded=load_recorded_poses(poses_path,s.ip);
    mounting={0,0,0,1};DRAFramework::CDRFLEx robot;Connection connection(robot);
    // Planning may inspect ordinary SAFE_OFF without enabling the servo.
    open_robot(robot,connection,s.ip,!require_standby);s.center=snapshot().joints;
    if(robot.get_robot_system()!=ROBOT_SYSTEM_REAL)
        throw std::runtime_error("Confirmed real-sensor plan requires REAL robot system; mode is never switched automatically");
    s.low=s.high=s.center;
    const Joints home{{90,0,90,0,90,90}};
    TaskPose home_task{};int home_solution=2;
    if(use_home){
        float home_joints[6];for(int i=0;i<6;++i)home_joints[i]=static_cast<float>(home[i]);
        const auto* forward=robot.fkin(home_joints,COORDINATE_SYSTEM_BASE);
        if(!forward)throw std::runtime_error("SDK forward kinematics failed for the fixed home pose");
        for(int i=0;i<6;++i)home_task[i]=forward->_fPosition[i];
        home_solution=solution_for_pose(robot,home_task,home);
    }
    double largest_step=0;Joints previous=s.center;
    if(use_home){
        for(int i=0;i<6;++i){
            s.low[i]=std::min(s.low[i],home[i]);s.high[i]=std::max(s.high[i],home[i]);
            s.offsets[i]=std::max(s.offsets[i],std::abs(home[i]-s.center[i]));
            largest_step=std::max(largest_step,std::abs(home[i]-s.center[i]));
        }
        previous=home;
    }
    for(const auto& point:recorded){
        for(int i=0;i<6;++i){
            s.low[i]=std::min(s.low[i],point.joints[i]);s.high[i]=std::max(s.high[i],point.joints[i]);
            s.offsets[i]=std::max(s.offsets[i],std::abs(point.joints[i]-s.center[i]));
            largest_step=std::max(largest_step,std::abs(point.joints[i]-previous[i]));
        }
        previous=point.joints;
    }
    if(use_home)for(int i=0;i<6;++i)largest_step=std::max(largest_step,std::abs(home[i]-previous[i]));
    for(int i=0;i<6;++i){s.low[i]-=0.5;s.high[i]+=0.5;s.offsets[i]+=0.5;}
    s.max_step=largest_step+0.5;
    Plan plan;plan.s=s;plan.home=home;plan.home_task=home_task;plan.home_solution=home_solution;
    for(std::size_t i=0;i<recorded.size();++i){
        Waypoint w;w.joints=recorded[i].joints;w.task=recorded[i].task;
        w.solution=solution_for_pose(robot,w.task,w.joints);
        w.split=((i+1)%4==0?"valid":"train");plan.points.push_back(w);
    }
    const auto text=plan_text(plan);const auto checked=parse_plan(text);check_controller_limits(robot,checked);
    if(!file.empty()){
        std::ofstream out(file,std::ios::binary);
        if(!out || !(out<<text) || !(out.flush()))throw std::runtime_error("Failed to write plan");
    }
    std::cout<<"Initial MoveJ delta [deg]: ";
    for(int i=0;i<6;++i)std::cout<<(use_home?home[i]:recorded.front().joints[i])-s.center[i]<<(i==5?'\n':' ');
    if(use_home){
        std::cout<<"Home -> pose 1 delta [deg]: ";for(int i=0;i<6;++i)std::cout<<recorded.front().joints[i]-home[i]<<(i==5?'\n':' ');
        std::cout<<"Pose 20 -> home delta [deg]: ";for(int i=0;i<6;++i)std::cout<<home[i]-recorded.back().joints[i]<<(i==5?'\n':' ');
    }
    std::cout<<"Joint envelope is derived from these targets plus 0.5 deg; it is not a collision-free zone.\n";
    if(!file.empty()){
        print_plan(checked,fingerprint(text));
        std::cout<<"Plan saved to "<<file<<"; no motion or servo command was sent.\n";
    }
    return text;
}
void safety_check(const Settings& s,bool require_standby) {
    if(stop_requested)throw std::runtime_error("Interrupted by operator");
    if(access_state.load()!=MONITORING_ACCESS_CONTROL_GRANT)throw std::runtime_error("Robot control right lost or transfer requested");
    const auto st=robot_state.load();
    if(require_standby ? st!=STATE_STANDBY : (st!=STATE_STANDBY && st!=STATE_MOVING))
        throw std::runtime_error(std::string("Robot left the expected operating state: ")+state_name(st)+" ("+std::to_string(st)+")");
    const auto snap=snapshot();require_fresh_pose(snap);check_limits(snap.joints,s,0.05);
}
bool autonomous_run_safety_state(int state) {
    return state==SAFETY_STATE_SW_RUN || state==SAFETY_STATE_CW_RUN;
}
void print_motion_context(DRAFramework::CDRFLEx& robot,const char* label) {
    const auto state=robot.get_robot_state();
    const auto mode=robot.get_robot_mode();
    std::cout<<label<<": mode="<<(mode==ROBOT_MODE_AUTONOMOUS?"AUTONOMOUS":
                                  mode==ROBOT_MODE_MANUAL?"MANUAL":"OTHER")
             <<", robot_state="<<state_name(static_cast<int>(state))
             <<", safety_state="<<safety_state_name(safety_state.load())
             <<", control_access="<<access_state.load()
             <<", disconnected="<<robot_disconnected.load()
             <<", joints=["<<joint_text(snapshot().joints)<<"]\n"<<std::flush;
}
void require_autonomous_move_safety(DRAFramework::CDRFLEx& robot,const Settings& s,
                                    std::uint64_t confirmed_warnings,bool request_move_event=false) {
    const auto check_conditions=[&] {
        if(stop_requested || robot_disconnected.load() ||
           access_state.load()!=MONITORING_ACCESS_CONTROL_GRANT)
            throw std::runtime_error("Interrupted, disconnected, or control right lost before motion");
        if(warning_events.load()!=confirmed_warnings) {
            print_diagnostics();
            throw std::runtime_error("A controller warning/alarm arrived before motion; inspect the teach pendant");
        }
        if(robot.get_robot_state()!=STATE_STANDBY || robot_state.load()!=STATE_STANDBY)
            throw std::runtime_error("Robot is not confirmed in STANDBY before motion");
        if(robot.get_robot_mode()!=ROBOT_MODE_AUTONOMOUS)
            throw std::runtime_error("Robot mode is not confirmed AUTONOMOUS before motion");
        const auto current=snapshot();require_fresh_pose(current);check_limits(current.joints,s);
    };
    check_conditions();
    int current_safety=safety_state.load();
    if(request_move_event || !autonomous_run_safety_state(current_safety)) {
        std::cout<<"Safety state is "<<safety_state_name(current_safety)
                 <<"; requesting AUTONOMOUS / EVENT_MOVE and waiting for SW_RUN or CW_RUN.\n"<<std::flush;
        if(!robot.set_safety_mode(SAFETY_MODE_AUTONOMOUS,SAFETY_MODE_EVENT_MOVE)) {
            print_diagnostics();
            throw std::runtime_error("Controller rejected AUTONOMOUS / EVENT_MOVE; no motion was sent");
        }
    }
    const auto deadline=Clock::now()+std::chrono::seconds(5);
    while(Clock::now()<deadline) {
        check_conditions();
        current_safety=safety_state.load();
        if(autonomous_run_safety_state(current_safety)) {
            std::cout<<"Motion safety state ready: "<<safety_state_name(current_safety)<<".\n"<<std::flush;
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    print_motion_context(robot,"Autonomous motion preflight failed");
    print_diagnostics();
    throw std::runtime_error("Safety state did not enter SW_RUN/CW_RUN; no motion was sent");
}
void configure_autonomous_motion(DRAFramework::CDRFLEx& robot,const Settings& s,
                                 std::uint64_t confirmed_warnings) {
    print_motion_context(robot,"Before autonomous mode setup");
    if(robot.get_robot_state()!=STATE_STANDBY || robot_state.load()!=STATE_STANDBY)
        throw std::runtime_error("Robot must remain in STANDBY before changing to AUTONOMOUS mode");
    if(robot.get_robot_mode()!=ROBOT_MODE_AUTONOMOUS) {
        if(!robot.set_robot_mode(ROBOT_MODE_AUTONOMOUS)) {
            print_diagnostics();
            throw std::runtime_error("Controller rejected ROBOT_MODE_AUTONOMOUS; no motion was sent");
        }
    }
    if(robot.get_robot_mode()!=ROBOT_MODE_AUTONOMOUS) {
        print_diagnostics();
        throw std::runtime_error("ROBOT_MODE_AUTONOMOUS was not confirmed; no motion was sent");
    }
    require_autonomous_move_safety(robot,s,confirmed_warnings,true);
    print_motion_context(robot,"Autonomous motion mode ready");
}
bool at_target(const Joints& actual,const Joints& target,double tolerance=0.2) {
    for(int i=0;i<6;++i)if(std::abs(actual[i]-target[i])>tolerance)return false;
    return true;
}
void request_control(DRAFramework::CDRFLEx& robot) {
    // Preserve a grant that the SDK may already have reported on this connection.
    // A concurrent callback must not be overwritten while clearing an older event.
    int observed=access_state.load();
    while(observed!=MONITORING_ACCESS_CONTROL_GRANT && !access_state.compare_exchange_weak(observed,-1)) {}
    if(observed==MONITORING_ACCESS_CONTROL_GRANT){
        if(stop_requested || robot_disconnected.load())throw std::runtime_error("Interrupted or disconnected before using control right");
        require_fresh_pose(snapshot());
        std::cout<<"控制器已向当前连接授予控制权。\n"<<std::flush;
        return;
    }
    if(!robot.manage_access_control(MANAGE_ACCESS_CONTROL_REQUEST))throw std::runtime_error("Control request could not be sent");
    std::cout<<"已请求控制权：请在示教器上同意/允许移交控制权，最多等待 60 秒。\n"<<std::flush;
    const auto deadline=Clock::now()+std::chrono::seconds(60);
    while(Clock::now()<deadline&&!stop_requested&&!robot_disconnected.load()) {
        const int a=access_state.load();if(a==MONITORING_ACCESS_CONTROL_GRANT)return;
        if(a==MONITORING_ACCESS_CONTROL_DENY)throw std::runtime_error("Control-right request was denied");
        if(a==MONITORING_ACCESS_CONTROL_LOSS)throw std::runtime_error("Control right was lost");
        require_fresh_pose(snapshot());std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    throw std::runtime_error("No control-right grant before timeout or interruption");
}
void require_servo_ready(DRAFramework::CDRFLEx& robot,const Settings& s,
                         bool safe_off_confirmed,std::uint64_t confirmed_warnings) {
    ServoEnablePolicy policy(safe_off_confirmed);
    auto servo_deadline=Clock::time_point::max();
    auto callback_deadline=Clock::time_point::max();
    for(;;) {
        if(stop_requested || robot_disconnected.load() || access_state.load()!=MONITORING_ACCESS_CONTROL_GRANT)
            throw std::runtime_error("Interrupted, disconnected, or control right lost during servo preparation");
        if(warning_events.load()!=confirmed_warnings) {
            print_diagnostics();
            throw std::runtime_error("A new controller warning/alarm arrived after confirmation; inspect the teach pendant");
        }
        const auto current=snapshot();require_fresh_pose(current);check_limits(current.joints,s);
        if(!at_target(current.joints,s.center,0.2))
            throw std::runtime_error("Robot moved from the reviewed start during servo preparation");
        const auto state=robot.get_robot_state();
        const auto mapped=state==STATE_STANDBY?ServoState::Standby:
                          state==STATE_SAFE_OFF?ServoState::SafeOff:ServoState::Other;
        const auto action=policy.observe(mapped,true);
        if(action==ServoAction::Ready) {
            if(robot_state.load()!=STATE_STANDBY) {
                if(callback_deadline==Clock::time_point::max())callback_deadline=Clock::now()+std::chrono::seconds(3);
                if(Clock::now()>=callback_deadline) {
                    print_diagnostics();
                    throw std::runtime_error("STANDBY was not confirmed by the monitoring callback");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            if(robot.check_motion()!=0) {
                const bool stopped=robot.stop(STOP_TYPE_QUICK);
                std::cerr<<"Controller is not idle after servo preparation; QUICK stop "
                         <<(stopped?"accepted":"not confirmed")<<".\n";
                print_diagnostics();
                throw std::runtime_error("Controller is not idle after servo preparation");
            }
            std::cout<<"Servo ready: controller is STANDBY.\n"<<std::flush;
            return;
        }
        callback_deadline=Clock::time_point::max();
        if(action==ServoAction::NeedConfirmation) {
            std::cout<<"控制权移交后状态变为 SAFE_OFF。请检查示教器并确认无急停或保护报警；"
                     <<"若确认可以上伺服，按回车尝试一次，输入其他字符取消： "<<std::flush;
            std::string command;
            if(!read_command(command)||!command.empty()||stop_requested)
                throw std::runtime_error("Servo-on cancelled by operator");
            policy.confirm_safe_off();
            continue;
        }
        if(action==ServoAction::EnableOnce) {
            if(stop_requested || robot_disconnected.load() ||
               access_state.load()!=MONITORING_ACCESS_CONTROL_GRANT ||
               warning_events.load()!=confirmed_warnings || robot.get_robot_state()!=STATE_SAFE_OFF) {
                print_diagnostics();
                throw std::runtime_error("Servo-on cancelled because controller conditions changed");
            }
            std::cout<<"Requesting CONTROL_SERVO_ON once for confirmed SAFE_OFF...\n"<<std::flush;
            if(!robot.set_robot_control(CONTROL_SERVO_ON)) {
                print_diagnostics();
                throw std::runtime_error("Controller rejected servo-on; inspect the teach pendant");
            }
            servo_deadline=Clock::now()+std::chrono::seconds(15);
        } else if(action==ServoAction::Reject) {
            if(state==STATE_MOVING) {
                const bool stopped=robot.stop(STOP_TYPE_QUICK);
                std::cerr<<"Unexpected motion during servo preparation; QUICK stop "
                         <<(stopped?"accepted":"not confirmed")<<".\n";
            }
            print_diagnostics();
            throw std::runtime_error(std::string("Servo preparation stopped in state ")+state_name(static_cast<int>(state)));
        } else if(action==ServoAction::Wait && Clock::now()>=servo_deadline) {
            print_diagnostics();
            throw std::runtime_error("Servo-on did not reach STANDBY within 15 seconds; no retry was sent");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
void move_and_wait(DRAFramework::CDRFLEx& robot,const Settings& s,const Joints& expected_start,const Joints& target,
                   std::uint64_t confirmed_warnings,const TaskPose* task=nullptr,int solution=2) {
    safety_check(s,true);auto initial_snapshot=snapshot();auto initial=initial_snapshot.joints;
    if(!at_target(initial,expected_start,0.2))
        throw std::runtime_error("Actual joints differ from the reviewed segment start by >0.2 deg");
    if(robot.check_motion()!=0)throw std::runtime_error("Controller is not idle before the next move");
    require_autonomous_move_safety(robot,s,confirmed_warnings);
    initial_snapshot=snapshot();require_fresh_pose(initial_snapshot);initial=initial_snapshot.joints;
    if(!at_target(initial,expected_start,0.2))
        throw std::runtime_error("Actual joints changed from the reviewed segment start during safety-mode preparation");
    if(robot.check_motion()!=0)throw std::runtime_error("Controller is not idle after motion safety preparation");
    if(at_target(initial,target,0.2)){
        std::cout<<"Already within 0.2 deg of target; no move sent.\n"<<std::flush;
        return;
    }
    float dest[6];double largest=0;
    for(int i=0;i<6;++i){dest[i]=static_cast<float>(target[i]);largest=std::max(largest,std::abs(target[i]-initial[i]));}
    if(largest>s.max_step)throw std::runtime_error("Actual motion start is too far from the target");
    std::cout<<(task?"MoveJX":"MoveJ")<<" current joints [deg]: "<<joint_text(initial)
             <<"; target joints [deg]: "<<joint_text(target)<<"; velocity="<<s.velocity
             <<" deg/s, acceleration="<<s.acceleration<<" deg/s^2; mode=AUTONOMOUS; safety="
             <<safety_state_name(safety_state.load())<<".\n"<<std::flush;
    bool active=false;
    try {
        const auto initial_moving_events=moving_events.load();
        // Mark active before the request: even a failed/ambiguous response may have started motion.
        active=true;
        bool accepted=false;
        if(task){
            float task_dest[6];for(int i=0;i<6;++i)task_dest[i]=static_cast<float>((*task)[i]);
            accepted=robot.movejx(task_dest,static_cast<unsigned char>(solution),
                                  static_cast<float>(s.velocity),static_cast<float>(s.acceleration),0.f,
                                  MOVE_MODE_ABSOLUTE,MOVE_REFERENCE_BASE);
        }else accepted=robot.amovej(dest,static_cast<float>(s.velocity),static_cast<float>(s.acceleration),0.f,MOVE_MODE_ABSOLUTE);
        if(!accepted)throw std::runtime_error("Robot rejected or did not acknowledge the move");
        MotionCompletion completion(initial,target,initial_snapshot.seq,initial_moving_events,task!=nullptr);
        const auto start_deadline=Clock::now()+std::chrono::seconds(10);
        // The controller's 50% operation-speed override also slows this move.
        const double seconds=20+4*largest/(s.velocity*0.5);
        const auto deadline=Clock::now()+std::chrono::milliseconds(static_cast<long long>(1000*seconds));
        while(Clock::now()<deadline) {
            safety_check(s,false);
            const int state=robot.check_motion();
            if(state<0 || state>2)throw std::runtime_error("Unexpected motion status");
            const auto actual=snapshot();require_fresh_pose(actual);
            const int operating_state=robot_state.load();
            if(completion.observe(state,operating_state==STATE_MOVING,operating_state==STATE_STANDBY,
                                  moving_events.load(),actual.seq,actual.joints,Clock::now())) {
                active=false;
                std::cout<<"Move complete; joint feedback is stable.\n"<<std::flush;
                return;
            }
            if(!completion.started() && Clock::now()>=start_deadline)
                throw std::runtime_error("Move was acknowledged but no motion start was observed within 10 seconds");
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
        throw std::runtime_error("Motion timed out or target was not reached");
    } catch (...) {
        if(active){
            const int state_before=robot_state.load();
            const int access_before=access_state.load();
            const bool disconnected_before=robot_disconnected.load();
            // Send the stop request before printing diagnostics, which could block.
            const bool stop_accepted=!disconnected_before && robot.stop(STOP_TYPE_QUICK);
            std::cerr<<"Motion interrupted: requesting QUICK stop. Use the independent emergency stop if needed.\n";
            std::cerr<<"Before stop: state="<<state_name(state_before)<<" ("<<state_before
                     <<"), access="<<access_before<<", disconnected="<<disconnected_before<<"\n";
            print_diagnostics();
            if(disconnected_before)
                std::cerr<<"QUICK stop was not sent because the SDK reported a disconnection.\n";
            else if(!stop_accepted)
                std::cerr<<"QUICK stop request returned failure; verify the robot on the teach pendant.\n";
            else std::cerr<<"QUICK stop request was accepted; verify the robot has stopped.\n";
        }
        throw;
    }
}
void settle(DRAFramework::CDRFLEx& robot,const Settings& s,const Joints& target) {
    const auto end=Clock::now()+std::chrono::milliseconds(static_cast<long long>(1000*s.settle));
    while(Clock::now()<end){
        safety_check(s,true);if(robot.check_motion()!=0 || !at_target(snapshot().joints,target))
            throw std::runtime_error("Robot moved during settling");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
void stop_if_sampling_motion(DRAFramework::CDRFLEx& robot) {
    if(robot_disconnected.load()) {
        std::cerr<<"Robot disconnected during settling/capture; SDK stop unavailable. Verify robot is stopped.\n";
        return;
    }
    bool stop_needed=robot_state.load()!=STATE_STANDBY;
    if(!stop_needed) {
        try {stop_needed=robot.check_motion()!=0;} catch(...) {stop_needed=true;}
    }
    if(!stop_needed)return;
    bool accepted=false;
    try {accepted=robot.stop(STOP_TYPE_QUICK);} catch(...) {}
    std::cerr<<"Unexpected robot state during settling/capture; QUICK stop "
             <<(accepted?"accepted":"not confirmed")<<". Verify robot is stopped.\n";
    print_diagnostics();
}
int run_fit(const std::filesystem::path& executable,const std::vector<std::string>& args) {
    std::vector<std::string> all{executable.string()};all.insert(all.end(),args.begin(),args.end());
    std::vector<char*> argv;for(auto& s:all)argv.push_back(&s[0]);argv.push_back(nullptr);
    const pid_t child=fork();if(child<0)throw std::runtime_error(system_error("fork failed"));
    if(child==0){execv(argv[0],argv.data());_exit(127);}
    int status=0;while(waitpid(child,&status,0)<0){if(errno==EINTR)continue;throw std::runtime_error(system_error("waitpid failed"));}
    return WIFEXITED(status)?WEXITSTATUS(status):1;
}
void execute_sequence(Plan plan,const std::string& csv,const std::string& model,
                      bool direct_recorded=false,const std::string& plan_code="",
                      ftcal::Quat sensor_mount={0,0,0,1}) {
    auto& s=plan.s;
    if(!direct_recorded)print_plan(plan,plan_code);
    else {
        std::cout<<"Direct replay of "<<plan.points.size()<<" recorded joint poses.\n"
                 <<"Home [deg]: "<<joint_text(plan.home)<<"; speed "<<s.velocity
                 <<" deg/s; acceleration "<<s.acceleration<<" deg/s^2; settle "<<s.settle<<" s.\n";
        for(std::size_t i=0;i<plan.points.size();++i)
            std::cout<<std::setw(2)<<i+1<<"  "<<joint_text(plan.points[i].joints)<<'\n';
    }
    const auto fit=std::filesystem::canonical("/proc/self/exe").parent_path()/"ft_calibration";
    if(!std::filesystem::exists(fit))throw std::runtime_error("Missing ft_calibration executable next to ft_auto");
    const std::string compensated=model+".compensated.csv";
    const std::array<std::string,6> output_paths{{csv,csv+".meta",csv+".status",model,model+".meta",compensated}};
    if(std::set<std::string>(output_paths.begin(),output_paths.end()).size()!=output_paths.size())
        throw std::runtime_error("Output paths overlap");
    for(const auto& path:output_paths)
        if(std::filesystem::exists(path))
            throw std::runtime_error("Output already exists: "+path+"; choose a new filename");
    mounting=ftcal::normalize_quaternion(sensor_mount);
    std::cout<<"Sensor-to-flange quaternion [qx,qy,qz,qw]: "
             <<mounting[0]<<' '<<mounting[1]<<' '<<mounting[2]<<' '<<mounting[3]<<'\n';
    std::signal(SIGINT,on_signal);std::signal(SIGTERM,on_signal);
    DRAFramework::CDRFLEx robot;Connection connection(robot);open_robot(robot,connection,s.ip,true);
    if(robot.get_robot_system()!=ROBOT_SYSTEM_REAL)
        throw std::runtime_error("Real-sensor calibration requires REAL robot system; mode is never switched automatically.");
    if(direct_recorded)initialize_direct_sequence(plan,robot);
    check_controller_limits(robot,plan);
    if(!at_target(snapshot().joints,s.center,0.2))throw std::runtime_error("Current joints changed by >0.2 deg; restart and review the actual path");
    check_limits(snapshot().joints,s);
    std::string error;SerialPort serial;if(!serial.connect(s.device,error))throw std::runtime_error(error);
    const auto first=read_observation(serial);
    std::cout<<"Preflight sensor values (candidate): ";for(double v:first.wrench)std::cout<<v<<' ';std::cout<<'\n';
    print_axes_in_base(snapshot().q);
    print_motion_context(robot,"Preflight motion context");
    const auto state_before_confirmation=robot.get_robot_state();
    if(state_before_confirmation!=STATE_STANDBY && state_before_confirmation!=STATE_SAFE_OFF)
        throw std::runtime_error(std::string("Robot state before confirmation: ")+state_name(static_cast<int>(state_before_confirmation)));
    const auto warnings_before_confirmation=warning_events.load();
    if(state_before_confirmation==STATE_SAFE_OFF)
        std::cout<<"当前为 SAFE_OFF：确认只是正常伺服关闭且示教器无报警后，取得控制权将尝试上伺服一次。\n";
    std::cout<<"Confirm all paths were reviewed with the installed load and cables.\n";
    std::cout
             <<"Keep the independent emergency stop accessible.\n"
             <<"按回车后程序会申请控制权；若当前为普通 SAFE_OFF，会尝试伺服上电一次，然后请求 AUTONOMOUS / EVENT_MOVE。"
             <<"只有控制器确认 AUTONOMOUS 且安全状态进入 SW_RUN/CW_RUN 后才会发送运动。\n"
             <<"请确认路径安全、人员现场监护、示教器无急停或保护报警；按回车开始，输入其他字符取消： "<<std::flush;
    std::string command;if(!read_command(command)||!command.empty()||stop_requested)throw std::runtime_error("Start cancelled");
    // Acquire control after the operator starts; do not create output files on a denied request.
    request_control(robot);
    require_servo_ready(robot,s,state_before_confirmation==STATE_SAFE_OFF,warnings_before_confirmation);safety_check(s,true);
    configure_autonomous_motion(robot,s,warnings_before_confirmation);
    safety_check(s,true);
    if(!robot.change_operation_speed(50.f))throw std::runtime_error("Controller rejected the 50% operation-speed override");
    check_controller_limits(robot,plan);
    if(!at_target(snapshot().joints,s.center,0.2))throw std::runtime_error("Robot start pose changed while waiting for control");
    CalibrationSessionStatus session(csv+".status",plan.points.size());
    try {
        prepare_csv(csv,s.device,mounting);
        safety_check(s,true);
        if(!at_target(snapshot().joints,s.center,0.2))throw std::runtime_error("Robot start pose changed before the first move");
        Joints expected_start=s.center;
        if(s.version==4){
            std::cout<<"Moving to reviewed home pose before sampling.\n"<<std::flush;
            move_and_wait(robot,s,expected_start,plan.home,warnings_before_confirmation);expected_start=plan.home;
        }
        if(s.version>=5){
            std::cout<<"Moving to home with the recorded Cartesian MoveJX path.\n"<<std::flush;
            move_and_wait(robot,s,expected_start,plan.home,warnings_before_confirmation,&plan.home_task,plan.home_solution);expected_start=plan.home;
        }
        for(std::size_t i=0;i<plan.points.size();++i){
            const auto& w=plan.points[i];std::cout<<"Pose "<<i+1<<"/20 ("<<w.split<<")\n"<<std::flush;
            move_and_wait(robot,s,expected_start,w.joints,warnings_before_confirmation,
                          s.version>=5?&w.task:nullptr,w.solution);
            try {
                settle(robot,s,w.joints);safety_check(s,true);
                const auto capture_guard=[&] {
                    safety_check(s,true);
                    if(!at_target(snapshot().joints,w.joints))
                        throw std::runtime_error("Robot left target while sampling");
                };
                capture_pose(serial,csv,w.split,capture_guard);
                session.pose_captured(i+1);capture_guard();
            } catch(...) {
                stop_if_sampling_motion(robot);
                throw;
            }
            expected_start=w.joints;
        }
        if(s.version>=4){
            std::cout<<"All 20 poses collected. Returning to reviewed home pose.\n"<<std::flush;
            if(s.version>=5)move_and_wait(robot,s,expected_start,plan.home,warnings_before_confirmation,&plan.home_task,plan.home_solution);
            else move_and_wait(robot,s,expected_start,plan.home,warnings_before_confirmation);
            std::cout<<"Home reached; closing controller connection.\n"<<std::flush;
        }else std::cout<<"All 20 poses collected. Robot remains at the last pose; closing controller connection.\n"<<std::flush;
        if(!robot.close_connection())throw std::runtime_error("Failed to close the robot connection before fitting");
        connection.open=false;
        session.ready_for_fit();
        std::cout<<"Fitting...\n"<<std::flush;
        if(run_fit(fit,{"calibrate",csv,model})!=0)throw std::runtime_error("Calibration fit failed; inspect the recorded CSV");
        if(run_fit(fit,{"validate",csv,model,"valid"})!=0)throw std::runtime_error("Validation report failed");
        if(run_fit(fit,{"apply",csv,model,compensated})!=0)
            throw std::runtime_error("Gravity compensation failed; inspect the samples and model");
        session.complete();
    } catch(const std::exception& error) {
        try {session.fail(error.what());} catch(...) {std::cerr<<"Failed to write session failure status.\n";}
        throw;
    } catch(...) {
        try {session.fail("Unknown failure");} catch(...) {std::cerr<<"Failed to write session failure status.\n";}
        throw;
    }
    std::cout<<"Calibration model: "<<model<<"\nCompensated readings: "<<compensated<<'\n'
             <<"Review independent validation residuals; saving a model does not mean it passed acceptance.\n";
}
void run_plan_text(const std::string& text,const std::string& code,const std::string& csv,const std::string& model) {
    if(!code.empty()&&fingerprint(text)!=code)throw std::runtime_error("Plan check code does not match; review the actual plan again");
    execute_sequence(parse_plan(text),csv,model,false,code);
}
void run_plan(const std::string& file,const std::string& approved_code,const std::string& csv,const std::string& model) {
    if(approved_code.empty())throw std::runtime_error("A plan check code is required");
    run_plan_text(slurp(file),approved_code,csv,model);
}
void run_recorded(const std::string& poses,const std::string& csv,const std::string& model,
                  ftcal::Quat sensor_mount={0,0,0,1}) {
    constexpr const char* robot_ip="192.168.123.156";
    constexpr const char* sensor_device="/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00TFOX-if00-port0";
    const auto recorded=load_recorded_poses(poses,robot_ip);
    Plan sequence;auto& s=sequence.s;
    s.version=4;s.ip=robot_ip;s.device=sensor_device;s.velocity=100;s.acceleration=200;s.settle=3;
    sequence.home={{90,0,90,0,90,90}};
    for(std::size_t i=0;i<recorded.size();++i) {
        Waypoint point;point.joints=recorded[i].joints;
        point.split=((i+1)%4==0?"valid":"train");
        sequence.points.push_back(point);
    }
    execute_sequence(std::move(sequence),csv,model,true,"",sensor_mount);
}
void auto_usage(const char* prog) {
    std::cout<<"Usage:\n  "<<prog<<" inspect <robot-ip>\n  "<<prog<<" plan <site-settings.txt> <new-plan.txt>\n"
             <<"  "<<prog<<" plan-recorded <recorded-settings.txt> <recorded-poses.csv> <new-plan.txt>\n"
             <<"  "<<prog<<" plan-confirmed <recorded-poses.csv> <new-plan.txt>\n"
             <<"  "<<prog<<" plan-home <recorded-poses.csv> <new-plan.txt>\n"
             <<"  "<<prog<<" run-recorded <recorded-poses.csv> <new-samples.csv> <new-model.txt> [--mount-q qx qy qz qw]\n"
             <<"  "<<prog<<" run <reviewed-plan.txt> <plan-check-code> <new-samples.csv> <new-model.txt>\n"
             <<"Planning reads the robot only. Running may enable the servo once after Enter and a normal control-right grant, then commands motion.\n"
             <<"No collision checking is provided. Review all targets and complete paths with the actual setup.\n";
}
} // namespace

int main(int argc,char** argv){
    try {
        if(argc==2&&std::string(argv[1])=="--help"){auto_usage(argv[0]);return 0;}
        if(argc==3&&std::string(argv[1])=="inspect"){inspect_robot(argv[2]);return 0;}
        if(argc==4&&std::string(argv[1])=="plan"){make_plan(argv[2],argv[3]);return 0;}
        if(argc==5&&std::string(argv[1])=="plan-recorded"){make_recorded_plan(argv[2],argv[3],argv[4]);return 0;}
        if(argc==4&&std::string(argv[1])=="plan-confirmed"){make_confirmed_plan(argv[2],argv[3]);return 0;}
        if(argc==4&&std::string(argv[1])=="plan-home"){make_confirmed_plan(argv[2],argv[3],true);return 0;}
        if((argc==5||argc==10)&&std::string(argv[1])=="run-recorded"){
            ftcal::Quat sensor_mount{0,0,0,1};
            if(argc==10){
                if(std::string(argv[5])!="--mount-q")throw std::runtime_error("Expected --mount-q qx qy qz qw");
                sensor_mount=ftcal::normalize_quaternion({number(argv[6]),number(argv[7]),number(argv[8]),number(argv[9])});
            }
            run_recorded(argv[2],argv[3],argv[4],sensor_mount);return 0;
        }
        if(argc==6&&std::string(argv[1])=="run"){run_plan(argv[2],argv[3],argv[4],argv[5]);return 0;}
        auto_usage(argv[0]);return 2;
    }catch(const std::exception& e){std::cerr<<"ERROR: "<<e.what()<<'\n';return 1;}
}
