#pragma once

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

// A sidecar distinguishes an interrupted capture from a finished fit.
class CalibrationSessionStatus {
public:
    CalibrationSessionStatus(std::string path, std::size_t expected)
        : path_(std::move(path)), expected_(expected) { write("IN_PROGRESS"); }
    CalibrationSessionStatus(const CalibrationSessionStatus&) = delete;
    CalibrationSessionStatus& operator=(const CalibrationSessionStatus&) = delete;
    ~CalibrationSessionStatus() {
        if (!finished_) { try { write("INCOMPLETE"); } catch (...) {} }
    }
    void pose_captured(std::size_t count) {
        if(count!=captured_+1 || count>expected_)throw std::runtime_error("Invalid captured pose count");
        captured_ = count; write("IN_PROGRESS");
    }
    void ready_for_fit() {
        if(captured_!=expected_)throw std::runtime_error("Cannot fit a session with missing poses");
        write("FITTING"); fitting_ = true;
    }
    void complete() {
        if(captured_!=expected_ || !fitting_)throw std::runtime_error("Cannot complete a session before fitting");
        write("COMPLETE"); finished_ = true;
    }
    void fail(std::string reason) {
        for(char& c:reason)if(c=='\n'||c=='\r')c=' ';
        reason_=std::move(reason);write("INCOMPLETE");finished_=true;
    }

private:
    void write(const char* state) const {
        std::ofstream out(path_, std::ios::trunc);
        if (!out) throw std::runtime_error("Cannot write calibration status: " + path_);
        out << "ft_auto_session_v1\nstate=" << state << "\ncaptured_poses=" << captured_
            << "\nexpected_poses=" << expected_ << '\n';
        if(!reason_.empty())out<<"error="<<reason_<<'\n';
        out.flush();
        if (!out) throw std::runtime_error("Failed writing calibration status: " + path_);
    }
    std::string path_,reason_;
    std::size_t expected_ = 0, captured_ = 0;
    bool fitting_ = false, finished_ = false;
};

// Manually acquired CSVs have no sidecar. For automatic sessions, reject
// partial captures before fitting a model from them.
inline std::optional<std::size_t> require_calibration_session_ready(const std::string& csv) {
    const std::string path=csv+".status";
    std::error_code ec;
    const bool exists=std::filesystem::exists(path,ec);
    if(ec)throw std::runtime_error("Cannot inspect calibration status: "+path);
    if(!exists)return std::nullopt;
    std::ifstream in(path);
    if(!in)throw std::runtime_error("Cannot read calibration status: "+path);
    std::string line;
    if(!std::getline(in,line) || line!="ft_auto_session_v1")
        throw std::runtime_error("Invalid calibration status: "+path);
    std::map<std::string,std::string> fields;
    while(std::getline(in,line)) {
        const auto eq=line.find('=');
        if(eq==std::string::npos || !fields.emplace(line.substr(0,eq),line.substr(eq+1)).second)
            throw std::runtime_error("Invalid calibration status field: "+path);
    }
    if(in.bad())throw std::runtime_error("Failed reading calibration status: "+path);
    try {
        const auto state=fields.at("state");
        const auto captured_text=fields.at("captured_poses"), expected_text=fields.at("expected_poses");
        const auto digits=[](const std::string& value) {
            return !value.empty() && std::all_of(value.begin(),value.end(),[](unsigned char c){return c>='0'&&c<='9';});
        };
        if(!digits(captured_text)||!digits(expected_text))
            throw std::runtime_error("Invalid calibration status values: "+path);
        std::size_t a=0,b=0;
        const auto captured=std::stoull(captured_text,&a),expected=std::stoull(expected_text,&b);
        if(a!=captured_text.size()||b!=expected_text.size()||expected==0||captured!=expected||
           expected>std::numeric_limits<std::size_t>::max()||
           (state!="FITTING"&&state!="COMPLETE"))
            throw std::runtime_error("Automatic capture is incomplete; inspect "+path);
        return static_cast<std::size_t>(expected);
    } catch(const std::out_of_range&) {
        throw std::runtime_error("Missing or invalid calibration status values: "+path);
    } catch(const std::invalid_argument&) {
        throw std::runtime_error("Invalid calibration status values: "+path);
    }
}
