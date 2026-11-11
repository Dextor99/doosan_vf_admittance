#include "session_status.hpp"
#include <cassert>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

static std::string read(const std::filesystem::path& path) {
    std::ifstream in(path);
    assert(in);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}
int main() {
    char temp[]="/tmp/ft-session-status-XXXXXX";
    const char* dir=mkdtemp(temp);assert(dir);
    const auto csv=(std::filesystem::path(dir)/"samples.csv").string();
    const auto path=std::filesystem::path(csv+".status");
    assert(!require_calibration_session_ready(csv)); // Manual files need no automatic-session sidecar.
    {
        CalibrationSessionStatus status(path.string(),20);
        assert(read(path).find("state=IN_PROGRESS")!=std::string::npos);
        for(std::size_t i=1;i<=3;++i)status.pose_captured(i);
        assert(read(path).find("captured_poses=3")!=std::string::npos);
        bool rejected=false;
        try {status.complete();} catch(const std::runtime_error&) {rejected=true;}
        assert(rejected);
        rejected=false;
        try {require_calibration_session_ready(csv);} catch(const std::runtime_error&) {rejected=true;}
        assert(rejected);
    }
    assert(read(path).find("state=INCOMPLETE")!=std::string::npos);
    {
        CalibrationSessionStatus status(path.string(),20);
        status.pose_captured(1);
        status.fail("Sensor read failed\ncheck device");
    }
    assert(read(path).find("error=Sensor read failed check device")!=std::string::npos);
    {
        CalibrationSessionStatus status(path.string(),20);
        for(std::size_t i=1;i<=20;++i)status.pose_captured(i);
        status.ready_for_fit();
        assert(read(path).find("state=FITTING")!=std::string::npos);
        assert(require_calibration_session_ready(csv)==20);
        status.complete();
    }
    const auto finished=read(path);
    assert(finished.find("state=COMPLETE")!=std::string::npos);
    assert(finished.find("captured_poses=20")!=std::string::npos);
    assert(require_calibration_session_ready(csv)==20);
    std::filesystem::remove_all(dir);
}
