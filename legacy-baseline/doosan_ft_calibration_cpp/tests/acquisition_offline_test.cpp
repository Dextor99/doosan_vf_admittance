#include "ft_acquisition.hpp"
#include <atomic>
#include <cassert>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <thread>
#include <vector>

int main() {
    const std::string crc_example="123456789";
    assert(modbus_crc(reinterpret_cast<const uint8_t*>(crc_example.data()),crc_example.size())==0x4b37);
    char temp[]="/tmp/ft-acquisition-test-XXXXXX";
    const char* dir=mkdtemp(temp);assert(dir);
    const std::string csv=(std::filesystem::path(dir)/"samples.csv").string();

    const int master=posix_openpt(O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);
    assert(master>=0 && grantpt(master)==0 && unlockpt(master)==0);
    const char* slave=ptsname(master);assert(slave);
    const std::array<float,6> values{{1.25f,-2.5f,3.75f,0.1f,-0.2f,0.3f}};
    std::atomic<bool> running{true},corrupt_next{false},emulator_error{false};
    std::atomic<float> joint_shift{0};
    mounting={0,0,0,1};robot_disconnected.store(false);stop_requested=0;

    {
        SerialPort serial;std::string error;assert(serial.connect(slave,error));
        std::thread sensor([&] {
            std::array<uint8_t,8> request{};std::size_t used=0;
            while(running.load()) {
                pollfd entry{master,POLLIN,0};
                if(poll(&entry,1,50)<=0)continue;
                const auto n=read(master,request.data()+used,request.size()-used);
                if(n<0 && (errno==EAGAIN||errno==EINTR))continue;
                if(n<=0){emulator_error.store(true);break;}
                used+=static_cast<std::size_t>(n);
                if(used<request.size())continue;
                const uint16_t request_crc=static_cast<uint16_t>(request[6])|(static_cast<uint16_t>(request[7])<<8);
                if(request[0]!=1||request[1]!=4||request[5]!=12||request_crc!=modbus_crc(request.data(),6)) {
                    emulator_error.store(true);break;
                }
                std::array<uint8_t,29> frame{};frame[0]=1;frame[1]=4;frame[2]=24;
                for(std::size_t i=0;i<values.size();++i) {
                    uint32_t bits=0;std::memcpy(&bits,&values[i],sizeof(bits));
                    for(int j=0;j<4;++j)frame[3+4*i+j]=static_cast<uint8_t>(bits>>(24-8*j));
                }
                const uint16_t crc=modbus_crc(frame.data(),27);
                frame[27]=static_cast<uint8_t>(crc);frame[28]=static_cast<uint8_t>(crc>>8);
                if(corrupt_next.exchange(false))frame[28]^=0x80;
                if(write(master,frame.data(),frame.size())!=static_cast<ssize_t>(frame.size())) {
                    emulator_error.store(true);break;
                }
                used=0;
            }
        });
        std::thread monitoring([&] {
            MONITORING_DATA_EX data{};
            for(int i=0;i<6;++i)data._tCtrl._tJoint._fActualPos[i]=static_cast<float>(i*10);
            data._tCtrl._tTask._fActualPos[1][3]=30;
            data._tCtrl._tTask._fActualPos[1][4]=40;
            data._tCtrl._tTask._fActualPos[1][5]=50;
            while(running.load()) {
                data._tCtrl._tJoint._fActualPos[0]=joint_shift.load();
                on_robot_data_ex(&data);std::this_thread::sleep_for(std::chrono::milliseconds(30));
            }
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        assert(snapshot().received && std::abs(snapshot().joints[5]-50)<1e-6);
        prepare_csv(csv,"/simulated/serial",mounting);
        int checks=0;
        capture_pose(serial,csv,"train",[&] {++checks;require_fresh_pose(snapshot());});
        assert(checks>=62);

        std::ifstream in(csv);std::string header,line,extra;
        assert(std::getline(in,header) && std::getline(in,line) && !std::getline(in,extra));
        std::istringstream row(line);std::vector<std::string> columns;std::string value;
        while(std::getline(row,value,','))columns.push_back(value);
        assert(columns.size()==12 && columns[0]=="train");
        const auto expected_q=ftcal::quaternion_from_zyz_degrees(30,40,50);
        for(std::size_t i=0;i<expected_q.size();++i)
            assert(std::abs(std::stod(columns[2+i])-expected_q[i])<1e-6);
        for(std::size_t i=0;i<values.size();++i)
            assert(std::abs(std::stod(columns[6+i])-values[i])<1e-6);

        checks=0;bool blocked=false;
        try {capture_pose(serial,csv,"valid",[&] {if(++checks==5)throw std::runtime_error("simulated state change");});}
        catch(const std::runtime_error&) {blocked=true;}
        assert(blocked);

        checks=0;blocked=false;
        try {capture_pose(serial,csv,"valid",[&] {if(++checks==5)joint_shift.store(0.3f);});}
        catch(const std::runtime_error& e) {
            blocked=std::string(e.what()).find("Joint motion >0.2")!=std::string::npos;
        }
        assert(blocked);joint_shift.store(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        corrupt_next.store(true);blocked=false;
        try {capture_pose(serial,csv,"valid");} catch(const std::runtime_error& e) {
            blocked=std::string(e.what()).find("CRC mismatch")!=std::string::npos;
        }
        assert(blocked);
        std::ifstream after(csv);int lines=0;while(std::getline(after,line))++lines;
        assert(lines==2);
        running.store(false);sensor.join();monitoring.join();
        assert(!emulator_error.load());
    }
    close(master);std::filesystem::remove_all(dir);
}
