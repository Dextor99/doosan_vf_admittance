// Copyright (c) 2026. Educational reference implementation.
// C++17, standard library only; no ROS, Eigen or Doosan SDK required for offline use.
// IMPORTANT: Static, quasi-static gravity+offset model; NOT a robot motion controller.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "matrix_condition.hpp"
#include "session_status.hpp"

using V3 = std::array<double, 3>;
using V4 = std::array<double, 4>; // qx, qy, qz, qw
using V6 = std::array<double, 6>; // Fx Fy Fz Mx My Mz
constexpr double G0 = 9.80665;
constexpr int P = 10;
using Row = std::array<double, P>;

static V3 cross(const V3& a, const V3& b) {
    return {a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]};
}
static V3 scale(const V3& a, double b) {return {b*a[0], b*a[1], b*a[2]};}
static double norm4(const V4& q) {return std::sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);}
static V4 normalized_quat(const V4& q) {
    for (double v: q) if (!std::isfinite(v)) throw std::runtime_error("Non-finite quaternion");
    const double n = norm4(q);
    if (n < 0.98 || n > 1.02) throw std::runtime_error("Quaternion norm far from 1 (check orientation input)");
    return {q[0]/n,q[1]/n,q[2]/n,q[3]/n};
}
// Quaternion q_BS maps a vector expressed in SENSOR frame to BASE frame.
// Return g in sensor frame = R_BS^T * [0,0,-9.80665].
static V3 gravity_sensor(const V4& q_input) {
    const auto q=normalized_quat(q_input);
    V3 v={0.0,0.0,-G0};
    // Inverse quaternion rotation: v_s = v - 2*w*(q_xyz x v) + 2*q_xyz x (q_xyz x v)
    const V3 xyz={q[0],q[1],q[2]};
    const auto c1=cross(xyz,v);
    const auto c2=cross(xyz,c1);
    for (int i=0;i<3;++i) v[i] += -2.0*q[3]*c1[i]+2.0*c2[i];
    return v;
}
struct Sample {
    std::string split;
    double timestamp=0;
    V4 q{};
    V6 w{};
};
struct Model {
    // f = q_signed*g + bF; tau = h_signed x g + bT
    // signed_mass = sign_sensor * mass; h_signed = signed_mass * COM_sensor
    double signed_mass=0;
    V3 h{};
    V6 bias{};
};
static V6 prediction(const Model& model,const V4& q) {
    const V3 g=gravity_sensor(q);
    const V3 f=scale(g,model.signed_mass);
    const V3 t=cross(model.h,g);
    V6 p{};
    for (int j=0;j<3;++j) {p[j]=f[j]+model.bias[j];p[3+j]=t[j]+model.bias[3+j];}
    return p;
}
static std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> out;
    std::istringstream in(line);
    std::string field;
    while (std::getline(in,field,',')) out.push_back(field);
    if (!line.empty() && line.back()==',') out.emplace_back("");
    return out;
}
static double parse_double(const std::string& s, const std::string& column) {
    std::size_t consumed=0;
    double x=0;
    try {x=std::stod(s,&consumed);} catch (...) {throw std::runtime_error("Invalid number in "+column+": "+s);}
    if (consumed!=s.size() || !std::isfinite(x)) throw std::runtime_error("Invalid/non-finite number in "+column+": "+s);
    return x;
}
static std::vector<Sample> load_csv(const std::string& file) {
    std::ifstream in(file);
    if (!in) throw std::runtime_error("Cannot open input CSV: "+file);
    std::string line;
    if (!std::getline(in,line)) throw std::runtime_error("Empty CSV");
    if (!line.empty() && line.back()=='\r') line.pop_back(); // Accept Windows CRLF
    const auto header=split_csv_line(line);
    const std::vector<std::string> wanted={"split","stamp_s","qx","qy","qz","qw","fx","fy","fz","mx","my","mz"};
    if (header!=wanted) throw std::runtime_error("CSV header must be exactly: split,stamp_s,qx,qy,qz,qw,fx,fy,fz,mx,my,mz");
    std::vector<Sample> samples;
    int row=1;
    while (std::getline(in,line)) {
        ++row;
        if (!line.empty() && line.back()=='\r') line.pop_back();
        if (line.empty() || line[0]=='#') continue;
        auto c=split_csv_line(line);
        if (c.size()!=wanted.size()) throw std::runtime_error("Bad number of CSV fields on line "+std::to_string(row));
        Sample s;
        s.split=c[0];
        if (s.split!="train" && s.split!="valid" && s.split!="contact") {
            throw std::runtime_error("Bad split on line "+std::to_string(row)+": use train, valid, contact");
        }
        s.timestamp=parse_double(c[1],"stamp_s");
        for (int i=0;i<4;++i) s.q[i]=parse_double(c[2+i],wanted[2+i]);
        s.q=normalized_quat(s.q);
        for (int i=0;i<6;++i) s.w[i]=parse_double(c[6+i],wanted[6+i]);
        samples.push_back(s);
    }
    if (samples.empty()) throw std::runtime_error("No CSV data rows found");
    return samples;
}
static void write_sample(std::ostream& out,const Sample& s) {
    out<<s.split<<','<<s.timestamp;
    for (double x:s.q) out<<','<<x;
    for (double x:s.w) out<<','<<x;
    out<<'\n';
}
static void write_csv(const std::string& file,const std::vector<Sample>& samples) {
    std::ofstream out(file);
    if (!out) throw std::runtime_error("Cannot write CSV: "+file);
    out<<"split,stamp_s,qx,qy,qz,qw,fx,fy,fz,mx,my,mz\n";
    out<<std::setprecision(15);
    for (const auto& s:samples) write_sample(out,s);
    out.flush();
    if (!out) throw std::runtime_error("Failed while writing CSV: "+file);
}
static std::pair<std::vector<Row>,std::vector<double>> build_matrix(const std::vector<Sample>& samples) {
    std::vector<Row> A;
    std::vector<double> y;
    A.reserve(samples.size()*6);
    y.reserve(samples.size()*6);
    for (const auto& s:samples) {
        if (s.split!="train") continue;
        const auto g=gravity_sensor(s.q);
        Row r0{};r0[0]=g[0];r0[4]=1;A.push_back(r0);y.push_back(s.w[0]);
        Row r1{};r1[0]=g[1];r1[5]=1;A.push_back(r1);y.push_back(s.w[1]);
        Row r2{};r2[0]=g[2];r2[6]=1;A.push_back(r2);y.push_back(s.w[2]);
        // tau = h x g + b_tau
        Row r3{};r3[2]=g[2];r3[3]=-g[1];r3[7]=1;A.push_back(r3);y.push_back(s.w[3]);
        Row r4{};r4[3]=g[0];r4[1]=-g[2];r4[8]=1;A.push_back(r4);y.push_back(s.w[4]);
        Row r5{};r5[1]=g[1];r5[2]=-g[0];r5[9]=1;A.push_back(r5);y.push_back(s.w[5]);
    }
    return {A,y};
}
struct QRResult {Row theta{};int rank=0;double diag_ratio=0;double condition_2=0;};
// Small dense pivoted Householder QR. Stable enough for this 10-parameter LS system;
// reject rank-deficient or severely ill-conditioned data instead of guessing.
static QRResult least_squares(const std::vector<Row>& A,const std::vector<double>& y) {
    const int m=static_cast<int>(A.size());
    if (m<P || m!=static_cast<int>(y.size())) throw std::runtime_error("Insufficient LS observations");
    std::vector<Row> R=A;
    std::vector<double> b=y;
    std::array<int,P> perm{};for (int j=0;j<P;++j) perm[j]=j;
    std::array<double,P> diag{};
    for (int k=0;k<P;++k) {
        // Select the remaining column with the largest trailing 2-norm.
        int best=k;double best_norm=-1.0;
        for (int j=k;j<P;++j) {
            double n=0;for(int i=k;i<m;++i)n+=R[i][j]*R[i][j];
            if(n>best_norm){best_norm=n;best=j;}
        }
        if(best!=k) {
            for(int i=0;i<m;++i)std::swap(R[i][k],R[i][best]);
            std::swap(perm[k],perm[best]);
        }
        const double norm=std::sqrt(std::max(0.0,best_norm));
        if(norm<=1e-14){diag[k]=0;continue;}
        const double alpha=(R[k][k]>=0)?-norm:norm;
        std::vector<double> v(m-k);
        v[0]=R[k][k]-alpha;
        for(int i=k+1;i<m;++i)v[i-k]=R[i][k];
        double vv=0;for(double z:v)vv+=z*z;
        if(vv<=1e-28){diag[k]=0;continue;}
        const double factor=2.0/vv;
        for(int j=k;j<P;++j){
            double dot=0;for(int i=k;i<m;++i)dot+=v[i-k]*R[i][j];
            for(int i=k;i<m;++i)R[i][j]-=factor*v[i-k]*dot;
        }
        double dotb=0;for(int i=k;i<m;++i)dotb+=v[i-k]*b[i];
        for(int i=k;i<m;++i)b[i]-=factor*v[i-k]*dotb;
        diag[k]=std::abs(R[k][k]);
    }
    const double max_diag=*std::max_element(diag.begin(),diag.end());
    int rank=0;
    const double rank_tol=max_diag*1e-10;
    for(double d:diag) if(d>rank_tol) ++rank;
    QRResult ans;ans.rank=rank;
    if(rank<P)throw std::runtime_error("Calibration design matrix is rank deficient (rank="+std::to_string(rank)+" < 10). Use more varied orientations (tilt X and Y), not repeated poses or only yaw.");
    const double min_diag=*std::min_element(diag.begin(),diag.end());
    ans.diag_ratio=min_diag/max_diag;
    if(ans.diag_ratio < 1e-7)throw std::runtime_error("Calibration data are too ill-conditioned: increase orientation diversity");
    std::array<Row,P> triangular{};
    for(int i=0;i<P;++i)for(int j=i;j<P;++j)triangular[i][j]=R[i][j];
    ans.condition_2=matrix_condition_2<P>(triangular);
    Row z{};
    for(int i=P-1;i>=0;--i){
        double rhs=b[i];
        for(int j=i+1;j<P;++j)rhs-=R[i][j]*z[j];
        z[i]=rhs/R[i][i];
    }
    for(int j=0;j<P;++j)ans.theta[perm[j]]=z[j];
    return ans;
}
static Model to_model(const Row& t) {
    Model model;model.signed_mass=t[0];
    for(int j=0;j<3;++j)model.h[j]=t[1+j];
    for(int j=0;j<6;++j)model.bias[j]=t[4+j];
    if(!std::isfinite(model.signed_mass) || std::abs(model.signed_mass)<1e-6)
        throw std::runtime_error("Estimated mass is zero or invalid; COM cannot be determined");
    for(double x:model.h) if(!std::isfinite(x)) throw std::runtime_error("Invalid first moment");
    for(double x:model.bias) if(!std::isfinite(x)) throw std::runtime_error("Invalid bias");
    return model;
}
static void save_model(const std::string& file,const Model& model) {
    std::ofstream out(file);
    if(!out)throw std::runtime_error("Cannot save model: "+file);
    out<<"# FT static gravity calibration model V1\n"<<std::setprecision(17);
    out<<"signed_mass="<<model.signed_mass<<'\n';
    out<<"hx="<<model.h[0]<<'\n';out<<"hy="<<model.h[1]<<'\n';out<<"hz="<<model.h[2]<<'\n';
    const std::array<std::string,6> fields={"bfx","bfy","bfz","bmx","bmy","bmz"};
    for(int i=0;i<6;++i)out<<fields[i]<<'='<<model.bias[i]<<'\n';
    out.flush();
    if(!out) throw std::runtime_error("Failed while saving model: "+file);
}
static Model load_model(const std::string& file) {
    std::ifstream in(file);if(!in)throw std::runtime_error("Cannot open model: "+file);
    std::vector<std::string> names={"signed_mass","hx","hy","hz","bfx","bfy","bfz","bmx","bmy","bmz"};
    Row t{};std::array<bool,P> seen{};
    std::string line;
    while(std::getline(in,line)) {
        if(!line.empty() && line.back()=='\r') line.pop_back();
        if(line.empty()||line[0]=='#')continue;
        const auto pos=line.find('=');if(pos==std::string::npos)throw std::runtime_error("Bad model format");
        const auto key=line.substr(0,pos);const auto value=line.substr(pos+1);
        const auto it=std::find(names.begin(),names.end(),key);
        if(it==names.end())throw std::runtime_error("Unknown model key: "+key);
        const int idx=static_cast<int>(it-names.begin());
        if(seen[idx])throw std::runtime_error("Duplicate model key: "+key);
        seen[idx]=true;t[idx]=parse_double(value,key);
    }
    for(bool s:seen)if(!s)throw std::runtime_error("Incomplete model file");
    return to_model(t);
}
static void print_model(const Model& m) {
    if(std::abs(m.signed_mass)<0.05)
        std::cerr<<"WARNING: Estimated mass is below 0.05 kg; COM may be very sensitive to noise.\n";
    std::cout<<"Estimated payload mass = "<<std::abs(m.signed_mass)<<" kg\n";
    std::cout<<"Wrench gravity sign = "<<(m.signed_mass>=0?"+1":"-1")<<" (check sign convention!)\n";
    std::cout<<"COM in sensor frame [m] = "
             <<m.h[0]/m.signed_mass<<", "<<m.h[1]/m.signed_mass<<", "<<m.h[2]/m.signed_mass<<"\n";
    std::cout<<"Bias [Fx Fy Fz Mx My Mz] = ";
    for(double v:m.bias)std::cout<<v<<" ";
    std::cout<<"[N, Nm]\n";
}
static bool report(const std::vector<Sample>& samples,const Model& model,const std::string& split) {
    V6 ss{};V6 max_abs{};std::size_t count=0;
    for(const auto& s:samples) {
        if(split!="all" && s.split!=split)continue;
        auto p=prediction(model,s.q);
        for(int j=0;j<6;++j){const double d=s.w[j]-p[j];ss[j]+=d*d;max_abs[j]=std::max(max_abs[j],std::abs(d));}
        ++count;
    }
    if(count==0){std::cout<<"No rows for split="<<split<<"\n";return false;}
    std::cout<<"Residuals for split="<<split<<", n="<<count<<"\n";
    std::cout<<" channel    RMSE       max|error|\n";
    const std::array<std::string,6> names={"Fx(N)","Fy(N)","Fz(N)","Mx(Nm)","My(Nm)","Mz(Nm)"};
    for(int j=0;j<6;++j)std::cout<<std::setw(8)<<names[j]<<"   "<<std::setw(10)<<std::sqrt(ss[j]/count)<<"  "<<std::setw(10)<<max_abs[j]<<"\n";
    return true;
}

static void ensure_distinct_output(const std::string& output,const std::string& input) {
    namespace fs = std::filesystem;
    std::error_code ec;
    if(fs::exists(output,ec) && fs::exists(input,ec) && fs::equivalent(output,input,ec))
        throw std::runtime_error("Output must not overwrite an input: "+input);
    if(fs::absolute(output).lexically_normal()==fs::absolute(input).lexically_normal())
        throw std::runtime_error("Output must not overwrite an input: "+input);
}

static void ensure_new_output(const std::string& output) {
    if(std::filesystem::exists(output))
        throw std::runtime_error("Output already exists; choose a new file: "+output);
}

static void check_session_metadata(const std::string& samples,const std::string& model) {
    const std::string sample_meta=samples+".meta", model_meta=model+".meta";
    const bool have_sample=std::filesystem::exists(sample_meta);
    const bool have_model=std::filesystem::exists(model_meta);
    if(have_sample && have_model) {
        std::ifstream a(sample_meta,std::ios::binary), b(model_meta,std::ios::binary);
        if(!a || !b) throw std::runtime_error("Cannot read session metadata");
        const std::string ma((std::istreambuf_iterator<char>(a)),std::istreambuf_iterator<char>());
        const std::string mb((std::istreambuf_iterator<char>(b)),std::istreambuf_iterator<char>());
        if(ma!=mb) throw std::runtime_error("Sample and model acquisition metadata differ");
    } else if(have_sample != have_model) {
        std::cerr<<"WARNING: Missing sample or model metadata; acquisition settings cannot be compared.\n";
    }
}
static V4 quaternion_from_rpy(double roll,double pitch,double yaw) {
    // q_BS = Rz(yaw) Ry(pitch) Rx(roll), right-handed active rotation.
    const double cr=std::cos(roll/2),sr=std::sin(roll/2);
    const double cp=std::cos(pitch/2),sp=std::sin(pitch/2);
    const double cy=std::cos(yaw/2),sy=std::sin(yaw/2);
    return {sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy};
}
static void demo_data(const std::string& file) {
    std::mt19937 gen(20261106); // fixed seed for reproducible results
    std::uniform_real_distribution<double> tilt(-0.85,0.85), yaw(-3.0,3.0);
    std::normal_distribution<double> fn(0.0,0.02), tn(0.0,0.0018);
    Model truth;truth.signed_mass=1.8;
    const V3 com={0.03,-0.015,0.08};truth.h=scale(com,truth.signed_mass);
    truth.bias={0.45,-0.25,0.31,0.018,-0.012,0.025};
    std::vector<Sample> rows;
    for(int k=0;k<36;++k) {
        Sample s;s.split=(k<28?"train":"valid");s.timestamp=0.5*k;
        s.q=quaternion_from_rpy(tilt(gen),tilt(gen),yaw(gen));
        s.w=prediction(truth,s.q);
        for(int j=0;j<6;++j)s.w[j]+=(j<3?fn(gen):tn(gen));
        rows.push_back(s);
    }
    write_csv(file,rows);
    std::cout<<"Generated synthetic data: "<<file<<" (28 train + 8 valid)\n";
    std::cout<<"Ground truth mass=1.8 kg, COM=(0.03,-0.015,0.08) m\n";
}
static void usage() {
    std::cout<<R"(FT static gravity/offset calibration (C++17, no ROS)
Usage:
  ft_calibration demo <synthetic.csv>
  ft_calibration calibrate <samples.csv> <model.txt>
  ft_calibration validate <samples.csv> <model.txt> [train|valid|all]
  ft_calibration apply <samples.csv> <model.txt> <compensated.csv>

Input CSV header EXACTLY:
  split,stamp_s,qx,qy,qz,qw,fx,fy,fz,mx,my,mz

- split=train: no contact, stationary, one MEAN observation per diverse pose.
- split=valid: held-out no-contact poses (not used in calibration).
- split=contact: optional contact data (do NOT use for fitting/zero validation).
- Quaternion q=(qx,qy,qz,qw) rotates SENSOR coordinates to robot BASE coordinates.
- Wrench in SENSOR coordinates: force N, moment N*m at sensor measurement origin.
- Base frame assumed Z-up, gravity = (0,0,-9.80665) m/s^2.
- Calibrate with raw (or consistently pre-tared) readings, but NEVER tare between poses.
- Contact estimate uses quasi-static model only; no inertia/acceleration correction.
)";
}
int main(int argc,char** argv) {
    try {
        if(argc<2){usage();return 0;}
        std::cout<<std::fixed<<std::setprecision(6);
        const std::string cmd=argv[1];
        if(cmd=="demo" && argc==3) {ensure_new_output(argv[2]);demo_data(argv[2]);return 0;}
        if(cmd=="calibrate" && argc==4) {
            ensure_distinct_output(argv[3],argv[2]);
            const std::string input_meta=std::string(argv[2])+".meta";
            const std::string model_meta=std::string(argv[3])+".meta";
            ensure_distinct_output(argv[3],input_meta);
            ensure_distinct_output(model_meta,argv[2]);
            ensure_distinct_output(model_meta,input_meta);
            ensure_new_output(argv[3]);
            if(std::filesystem::exists(model_meta))
                throw std::runtime_error("Model metadata already exists; choose a new model path to avoid mixing sessions: "+model_meta);
            const auto expected_auto_poses=require_calibration_session_ready(argv[2]);
            const auto samples=load_csv(argv[2]);
            if(expected_auto_poses && samples.size()!=*expected_auto_poses)
                throw std::runtime_error("Automatic capture row count does not match its status file");
            int ntrain=0;for(const auto& s:samples)if(s.split=="train")++ntrain;
            if(ntrain<10)throw std::runtime_error("Use at least 10 diverse TRAIN poses (recommend 15-30)");
            const auto [A,y]=build_matrix(samples);
            const auto fit=least_squares(A,y);
            const auto model=to_model(fit.theta);
            print_model(model);
            std::cout<<std::scientific<<"LS rank="<<fit.rank<<"/10, QR diag ratio="<<fit.diag_ratio
                     <<", unscaled design matrix 2-norm condition="<<fit.condition_2
                     <<" (depends on parameter units)\n"<<std::fixed;
            report(samples,model,"train");
            if(!report(samples,model,"valid"))
                std::cerr<<"WARNING: No held-out validation poses; model has not been independently validated.\n";
            save_model(argv[3],model);
            if(std::filesystem::exists(input_meta)) {
                std::filesystem::copy_file(input_meta,model_meta);
                std::cout<<"Copied acquisition settings to: "<<model_meta<<"\n";
            } else {
                std::cerr<<"WARNING: Input has no acquisition metadata; model cannot be used by ft_acquire monitor.\n";
            }
            std::cout<<"Saved calibration model: "<<argv[3]<<"\n";
            return 0;
        }
        if(cmd=="validate" && (argc==4||argc==5)) {
            check_session_metadata(argv[2],argv[3]);
            const auto samples=load_csv(argv[2]);const auto model=load_model(argv[3]);
            const std::string split=(argc==5?argv[4]:"valid");
            if(split!="train" && split!="valid" && split!="all")
                throw std::runtime_error("Validation split must be train, valid or all");
            print_model(model);
            if(split=="all") std::cerr<<"WARNING: all includes any contact rows; those residuals are not a no-contact validation.\n";
            return report(samples,model,split)?0:1;
        }
        if(cmd=="apply" && argc==5) {
            ensure_distinct_output(argv[4],argv[2]);
            ensure_distinct_output(argv[4],argv[3]);
            ensure_distinct_output(argv[4],std::string(argv[2])+".meta");
            ensure_distinct_output(argv[4],std::string(argv[3])+".meta");
            ensure_new_output(argv[4]);
            check_session_metadata(argv[2],argv[3]);
            const auto samples=load_csv(argv[2]);const auto model=load_model(argv[3]);
            std::ofstream out(argv[4]);if(!out)throw std::runtime_error("Cannot write compensated CSV");
            out<<"split,stamp_s,fx_raw,fy_raw,fz_raw,mx_raw,my_raw,mz_raw,fx_comp,fy_comp,fz_comp,mx_comp,my_comp,mz_comp\n";
            out<<std::setprecision(15);
            for(const auto& s:samples) {
                const auto pred=prediction(model,s.q);
                out<<s.split<<','<<s.timestamp;
                for(double w:s.w)out<<','<<w;
                for(int j=0;j<6;++j)out<<','<<(s.w[j]-pred[j]);
                out<<'\n';
            }
            out.flush();
            if(!out)throw std::runtime_error("Failed while writing compensated CSV");
            std::cout<<"Saved compensated CSV: "<<argv[4]<<"\n";
            std::cout<<"WARNING: Nonzero 'comp' is CONTACT only under quasi-static conditions after checking signs/calibration.\n";
            return 0;
        }
        usage();return 2;
    }catch(const std::exception& e){std::cerr<<"ERROR: "<<e.what()<<'\n';return 1;}
}
