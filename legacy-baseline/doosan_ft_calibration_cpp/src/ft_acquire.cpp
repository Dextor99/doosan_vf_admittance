#include "ft_acquisition.hpp"

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") { usage(argv[0]); return 0; }
    if (argc != 10 || (std::string(argv[1]) != "record" && std::string(argv[1]) != "monitor") ||
        std::string(argv[9]) != "--assume-xjc-format") { usage(argv[0]); return 2; }
    try {
        const std::string mode(argv[1]), file(argv[2]), device(argv[4]);
        in_addr addr{};
        if (inet_pton(AF_INET, argv[3], &addr) != 1) throw std::runtime_error("Invalid robot IPv4 address");
        mounting = ftcal::normalize_quaternion({number(argv[5]), number(argv[6]), number(argv[7]), number(argv[8])});
        std::signal(SIGINT, on_signal); std::signal(SIGTERM, on_signal);
        ftcal::Parameters params;
        if (mode == "monitor") {
            params = ftcal::load_parameters(file);
            check_model_metadata(file, device, mounting);
        }
        if (mode == "record") prepare_csv(file, device, mounting);
        std::string err;
        SerialPort serial;
        if (!serial.connect(device, err)) throw std::runtime_error(err);
        DRAFramework::CDRFLEx robot;
        robot.set_on_monitoring_data(on_robot_data);
        robot.set_on_disconnected(on_robot_disconnected);
        std::cout << "Connecting to Doosan " << argv[3] << ":12345...\n";
        if (!robot.open_connection(argv[3], 12345)) throw std::runtime_error("Cannot connect to robot");
        int result = 0;
        try {
            const auto deadline = Clock::now() + std::chrono::seconds(5);
            while (!stop_requested && !snapshot().received && !robot_disconnected.load() && Clock::now() < deadline)
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            if (stop_requested) throw std::runtime_error("Interrupted");
            require_fresh_pose(snapshot());
            std::cout << "Using flange pose and q_flange_sensor = " << mounting[0] << ' ' << mounting[1]
                      << ' ' << mounting[2] << ' ' << mounting[3] << "\n"
                      << "XJC values are candidate Fx Fy Fz Mx My Mz; verify axis order, units and calibration.\n"
                      << "Host timestamps only; no hardware synchronization. Static/quasi-static use only.\n";
            print_axes_in_base(snapshot().q);
            if (mode == "record") {
                std::string command;
                while (!stop_requested) {
                    std::cout << "Hold still, no contact. Enter train, valid or quit: " << std::flush;
                    if (!read_command(command) || command == "quit") break;
                    if (command != "train" && command != "valid") { std::cout << "Use train, valid or quit.\n"; continue; }
                    try { capture_pose(serial, file, command); }
                    catch (const std::exception& e) {
                        std::cerr << "Capture failed: " << e.what() << '\n';
                        if (robot_disconnected.load() || snapshot().invalid || stop_requested) throw;
                    }
                }
            } else {
                const ftcal::Compensator comp(params);
                std::cout << "Showing raw and compensated values; Ctrl+C to stop. Verify model on held-out poses.\n";
                while (!stop_requested) {
                    const auto start = Clock::now();
                    Observation o;
                    try { o = read_observation(serial); }
                    catch (...) { if (stop_requested) break; throw; }
                    const auto corrected = comp.compensate(o.wrench, o.q);
                    std::cout << std::fixed << std::setprecision(3) << "raw:";
                    for (double v : o.wrench) std::cout << ' ' << v;
                    std::cout << "  compensated:";
                    for (double v : corrected) std::cout << ' ' << v;
                    std::cout << "  pose_dt_ms=" << o.pose_age_ms << std::endl;
                    wait_interval(start);
                }
            }
        } catch (...) {
            robot.close_connection();
            throw;
        }
        if (!robot.close_connection()) { std::cerr << "Robot connection close failed\n"; result = 1; }
        return result;
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << '\n';
        return stop_requested ? 0 : 1;
    }
}
