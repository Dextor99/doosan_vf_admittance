// Read-only Doosan pose recorder. It does not request control or command motion.
#include <DRFLEx.h>
#include <arpa/inet.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {
using Clock=std::chrono::steady_clock;
constexpr auto kFresh=std::chrono::milliseconds(250);
constexpr auto kStabilize=std::chrono::milliseconds(500);
constexpr double kMaxMotion=0.2; // degrees, over the recording window
volatile std::sig_atomic_t stopped=0;
std::atomic<bool> disconnected{false};
void signal_handler(int){stopped=1;}

struct Snapshot {
    std::array<double,6> joints{}, flange{}, min_joints{}, max_joints{};
    Clock::time_point received{};
    std::uint64_t sequence=0;
    bool valid=false, invalid=false, watching=false;
};
std::mutex mutex;
Snapshot latest;
void on_data(const LPMONITORING_DATA data) {
    if(!data)return;
    std::lock_guard<std::mutex> lock(mutex);
    std::array<double,6> q{}, f{};
    for(int i=0;i<6;++i) {
        q[i]=data->_tCtrl._tJoint._fActualPos[i];
        f[i]=data->_tCtrl._tTask._fActualPos[1][i];
        if(!std::isfinite(q[i]) || !std::isfinite(f[i])) {latest.invalid=true;return;}
    }
    if(latest.watching)for(int i=0;i<6;++i){
        latest.min_joints[i]=std::min(latest.min_joints[i],q[i]);
        latest.max_joints[i]=std::max(latest.max_joints[i],q[i]);
    }
    latest.joints=q;latest.flange=f;latest.received=Clock::now();latest.valid=true;++latest.sequence;
}
void on_disconnect(){disconnected.store(true);}
Snapshot get_snapshot(){std::lock_guard<std::mutex> lock(mutex);return latest;}
bool fresh(const Snapshot& s){return s.valid && !s.invalid && Clock::now()-s.received<=kFresh;}

class Terminal {
    termios original_{};
    bool changed_=false;
public:
    Terminal() {
        if(!isatty(STDIN_FILENO))throw std::runtime_error("Use a terminal (e.g. VS Code integrated terminal) for single-key input");
        if(tcgetattr(STDIN_FILENO,&original_)!=0)throw std::runtime_error("Cannot read terminal settings");
        auto value=original_;value.c_lflag &= static_cast<tcflag_t>(~(ICANON|ECHO));
        value.c_cc[VMIN]=0;value.c_cc[VTIME]=0;
        if(tcsetattr(STDIN_FILENO,TCSANOW,&value)!=0)throw std::runtime_error("Cannot set terminal single-key mode");
        changed_=true;
    }
    ~Terminal(){if(changed_)tcsetattr(STDIN_FILENO,TCSANOW,&original_);}
    Terminal(const Terminal&)=delete;Terminal& operator=(const Terminal&)=delete;
};
class NewCsv {
    int fd_=-1;
public:
    explicit NewCsv(const std::string& path){
        fd_=open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC,0644);
        if(fd_<0)throw std::runtime_error("Cannot create "+path+": "+std::strerror(errno)+" (choose a new file)");
    }
    ~NewCsv(){if(fd_>=0)close(fd_);}
    void append(const std::string& text){
        std::size_t pos=0;
        while(pos<text.size()){
            const ssize_t n=write(fd_,text.data()+pos,text.size()-pos);
            if(n<0&&errno==EINTR)continue;
            if(n<=0)throw std::runtime_error("Failed writing pose CSV: "+std::string(std::strerror(errno)));
            pos+=static_cast<std::size_t>(n);
        }
        if(fdatasync(fd_)!=0)throw std::runtime_error("Failed syncing pose CSV: "+std::string(std::strerror(errno)));
    }
    NewCsv(const NewCsv&)=delete;NewCsv& operator=(const NewCsv&)=delete;
};
struct Connection {
    DRAFramework::CDRFLEx& robot;bool open=false;
    explicit Connection(DRAFramework::CDRFLEx& r):robot(r){}
    ~Connection(){if(open)robot.close_connection();}
};
struct Watch {
    Watch(){
        std::lock_guard<std::mutex> lock(mutex);
        if(!fresh(latest))throw std::runtime_error("Robot data is stale; pose not saved");
        latest.min_joints=latest.joints;latest.max_joints=latest.joints;latest.watching=true;
    }
    ~Watch(){std::lock_guard<std::mutex> lock(mutex);latest.watching=false;}
};
bool collect_stationary(Snapshot& result) {
    Watch watch;const auto initial=get_snapshot();
    const auto deadline=Clock::now()+kStabilize;
    while(!stopped && !disconnected.load() && Clock::now()<deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    result=get_snapshot();
    if(stopped||disconnected.load())return false;
    if(!fresh(result)||result.sequence<initial.sequence+3)return false;
    for(int i=0;i<6;++i)if(result.max_joints[i]-result.min_joints[i]>kMaxMotion)return false;
    return true;
}
void print_joints(const Snapshot& s){
    std::cout<<std::fixed<<std::setprecision(2)<<"\rJ1..J6: ";
    for(double j:s.joints)std::cout<<std::setw(8)<<j;
    std::cout<<" deg   [space: save | enter: finish]  "<<std::flush;
}
std::string csv_row(std::size_t index,const Snapshot& s,const std::string& ip){
    const double stamp=std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    std::ostringstream row;row<<std::setprecision(17)<<index<<','<<stamp<<','<<ip;
    for(double v:s.joints)row<<','<<v;
    for(double v:s.flange)row<<','<<v;
    row<<'\n';return row.str();
}
void summary(const std::vector<Snapshot>& poses){
    if(poses.empty())return;
    std::array<double,6> min=poses[0].joints,max=poses[0].joints,max_offset{},max_step{};
    for(std::size_t n=0;n<poses.size();++n)for(int i=0;i<6;++i){
        min[i]=std::min(min[i],poses[n].joints[i]);max[i]=std::max(max[i],poses[n].joints[i]);
        max_offset[i]=std::max(max_offset[i],std::abs(poses[n].joints[i]-poses[0].joints[i]));
        if(n)max_step[i]=std::max(max_step[i],std::abs(poses[n].joints[i]-poses[n-1].joints[i]));
    }
    std::cout<<std::fixed<<std::setprecision(2)<<"Recorded joint spans [deg] (observations, NOT safe limits):\n";
    for(int i=0;i<6;++i)std::cout<<" J"<<i+1<<": "<<min[i]<<" .. "<<max[i]
        <<", max |offset from pose 1|="<<max_offset[i]<<", max |step|="<<max_step[i]<<'\n';
    std::cout<<"Automatic replay needs exactly 20 poses. plan-recorded uses a 5 deg cap; run-recorded uses a 360 deg cap. Review the actual paths.\n";
}
} // namespace

int main(int argc,char** argv){
    if(argc==2&&std::string(argv[1])=="--help"){
        std::cout<<"Usage: "<<argv[0]<<" <robot-ipv4> <new-poses.csv>\nSpace saves a stationary pose; Enter finishes. Read-only monitoring.\n";return 0;
    }
    if(argc!=3){std::cerr<<"Usage: "<<argv[0]<<" <robot-ipv4> <new-poses.csv>\n";return 2;}
    in_addr a{};if(inet_pton(AF_INET,argv[1],&a)!=1){std::cerr<<"Invalid IPv4 address\n";return 2;}
    std::signal(SIGINT,signal_handler);std::signal(SIGTERM,signal_handler);
    std::vector<Snapshot> poses;
    try{
        Terminal terminal;
        DRAFramework::CDRFLEx robot;Connection connection(robot);
        robot.set_on_monitoring_data(on_data);robot.set_on_disconnected(on_disconnect);
        std::cout<<"Connecting to "<<argv[1]<<"; no motion or control commands are sent...\n"<<std::flush;
        if(!robot.open_connection(argv[1],12345))throw std::runtime_error("Robot connection failed");
        connection.open=true;
        const auto deadline=Clock::now()+std::chrono::seconds(5);
        while(!stopped&&!disconnected.load()&&!get_snapshot().valid&&Clock::now()<deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if(stopped)throw std::runtime_error("Interrupted");
        if(disconnected.load()||!fresh(get_snapshot()))throw std::runtime_error("No fresh robot monitoring data");
        NewCsv csv(argv[2]);
        csv.append("index,stamp_s,robot_ip,j1_deg,j2_deg,j3_deg,j4_deg,j5_deg,j6_deg,flange_x_mm,flange_y_mm,flange_z_mm,flange_a_deg,flange_b_deg,flange_c_deg\n");
        std::cout<<"Drag using the robot's approved hand-guiding procedure. Hold still, then press Space.\n"
                 <<"Enter finishes. Data is saved after each pose: "<<argv[2]<<"\n";
        auto next=Clock::now();
        while(!stopped&&!disconnected.load()){
            const auto s=get_snapshot();
            if(s.invalid)throw std::runtime_error("Non-finite monitoring data; stopped");
            if(Clock::now()>=next){if(fresh(s))print_joints(s);else std::cout<<"\rWaiting for fresh robot data...                      "<<std::flush;next=Clock::now()+std::chrono::milliseconds(200);}
            pollfd p{STDIN_FILENO,POLLIN,0};const int ready=poll(&p,1,50);
            if(ready<0&&errno==EINTR)continue;
            if(ready<0 || (p.revents&(POLLERR|POLLNVAL)))throw std::runtime_error("Terminal input error");
            if(ready && (p.revents&(POLLIN|POLLHUP))){
                char c=0;const ssize_t n=read(STDIN_FILENO,&c,1);
                if(n<0&&errno==EINTR)continue;
                if(n<0)throw std::runtime_error("Terminal read failed");
                if(n==0){if(p.revents&POLLHUP)break;continue;}
                if(c=='\n'||c=='\r')break;
                if(c!=' ')continue;
                std::cout<<"\nChecking that the robot is stationary (0.5 s)...\n"<<std::flush;
                Snapshot sample;
                if(!fresh(get_snapshot()) || !collect_stationary(sample)){
                    std::cout<<"Pose not saved: robot moved, data was stale, or capture was interrupted.\n";
                    continue;
                }
                bool duplicate=false;
                for(const auto& old:poses){
                    bool close=true;for(int i=0;i<6;++i)if(std::abs(sample.joints[i]-old.joints[i])>=0.1)close=false;
                    if(close){duplicate=true;break;}
                }
                if(duplicate){std::cout<<"Pose not saved: too similar to an existing pose (<0.1 deg per joint).\n";continue;}
                if(!poses.empty()){
                    bool beyond=false;
                    for(int i=0;i<6;++i) {
                        if(std::abs(sample.joints[i]-poses.front().joints[i])>5.0 ||
                           std::abs(sample.joints[i]-poses.back().joints[i])>5.0)beyond=true;
                    }
                    if(beyond)std::cout<<"WARNING: This pose exceeds the 5 deg plan-recorded cap; direct run-recorded uses different limits. Review the path.\n";
                }
                csv.append(csv_row(poses.size()+1,sample,argv[1]));poses.push_back(sample);
                std::cout<<"Saved pose "<<poses.size()<<".\n";
            }
        }
        std::cout<<"\nSaved "<<poses.size()<<" poses to "<<argv[2]<<".\n";summary(poses);
        if(disconnected.load())throw std::runtime_error("Robot disconnected; previously saved rows remain in the CSV");
        if(!robot.close_connection())throw std::runtime_error("Failed to close robot connection");
        connection.open=false;return 0;
    }catch(const std::exception& e){std::cerr<<"\nERROR: "<<e.what()<<"; "<<poses.size()<<" pose(s) already saved if file was created.\n";return 1;}
}
