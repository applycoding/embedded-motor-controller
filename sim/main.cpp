#include "motor/controller.hpp"
#include "motor/sim/plant.hpp"
#include "motor/sim/sim_hal.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>

namespace {

struct Options {
    int rpm = 1200;
    float seconds = 1.5f;
    float dt = 0.001f;
    const char* csv_path = "telemetry.csv";
};

void usage() {
    std::fputs(
        "usage: motor_sim [--rpm N] [--seconds T] [--dt T] [--csv PATH]\n"
        "\n"
        "Host-side DC motor simulation. Writes CSV telemetry and talks to the\n"
        "controller over the simulated UART (SET_RPM / START, checksummed).\n",
        stdout);
}

bool parse_int(const char* text, int& out) {
    if (text == nullptr || *text == '\0') {
        return false;
    }
    char* end = nullptr;
    const long value = std::strtol(text, &end, 10);
    if (end == text || *end != '\0') {
        return false;
    }
    out = static_cast<int>(value);
    return true;
}

bool parse_float(const char* text, float& out) {
    if (text == nullptr || *text == '\0') {
        return false;
    }
    char* end = nullptr;
    const float value = std::strtof(text, &end);
    if (end == text || *end != '\0') {
        return false;
    }
    out = value;
    return true;
}

bool parse_args(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
        const char* arg = argv[i];
        const char* value = (i + 1 < argc) ? argv[i + 1] : nullptr;
        if (std::strcmp(arg, "--help") == 0) {
            usage();
            return false;
        }
        if (std::strcmp(arg, "--rpm") == 0) {
            if (!parse_int(value, options.rpm)) {
                std::fputs("motor_sim: --rpm needs an integer\n", stderr);
                return false;
            }
            ++i;
            continue;
        }
        if (std::strcmp(arg, "--seconds") == 0) {
            if (!parse_float(value, options.seconds)) {
                std::fputs("motor_sim: --seconds needs a number\n", stderr);
                return false;
            }
            ++i;
            continue;
        }
        if (std::strcmp(arg, "--dt") == 0) {
            if (!parse_float(value, options.dt)) {
                std::fputs("motor_sim: --dt needs a number\n", stderr);
                return false;
            }
            ++i;
            continue;
        }
        if (std::strcmp(arg, "--csv") == 0) {
            if (value == nullptr || *value == '\0') {
                std::fputs("motor_sim: --csv needs a path\n", stderr);
                return false;
            }
            options.csv_path = value;
            ++i;
            continue;
        }
        std::fprintf(stderr, "motor_sim: unknown argument %s\n", arg);
        return false;
    }

    if (options.rpm < -5000 || options.rpm > 5000) {
        std::fputs("motor_sim: rpm must be between -5000 and 5000\n", stderr);
        return false;
    }
    if (!(options.seconds >= 0.05f && options.seconds <= 30.f)) {
        std::fputs("motor_sim: seconds must be between 0.05 and 30\n", stderr);
        return false;
    }
    if (!(options.dt >= 1.0e-4f && options.dt <= 0.01f) || options.dt > options.seconds) {
        std::fputs("motor_sim: dt must be between 0.0001 and 0.01, and below seconds\n", stderr);
        return false;
    }
    return true;
}

void send_payload(motor::sim::SimUart& uart, const char* payload) {
    char frame[80];
    const int n = motor::format_frame(frame, static_cast<int>(sizeof(frame)), payload);
    if (n > 0) {
        static_cast<void>(uart.inject_rx(frame, n));
    }
}

motor::ControllerConfig demo_config(float counts_per_rev) {
    motor::ControllerConfig config;
    // Plant gain is about 9500 rpm per unit duty. These gains plus the ramp
    // keep the first samples off the voltage rail.
    config.gains = motor::PidGains{0.0002f, 0.0012f, 0.f};
    config.limits = motor::PidLimits{-1.f, 1.f, -0.4f, 0.4f};
    config.max_rpm_per_s = 4000.f;
    config.counts_per_rev = counts_per_rev;
    config.speed_lpf_alpha = 0.35f;
    config.max_abs_rpm = 5000.f;
    config.faults = motor::FaultConfig{4.f, 8, 40.f, 0.75f, 300};
    config.watchdog_timeout_ticks = 250;
    return config;
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parse_args(argc, argv, options)) {
        if (argc > 1 && std::strcmp(argv[1], "--help") == 0) {
            return 0;
        }
        return 2;
    }

    motor::sim::SimPwm pwm;
    motor::sim::SimEncoder encoder;
    motor::sim::SimUart uart;
    motor::sim::DcMotorPlant plant;
    motor::SpeedController controller(pwm, encoder, uart,
                                      demo_config(plant.params().counts_per_rev));

    const std::string setpoint = "SET_RPM " + std::to_string(options.rpm);
    send_payload(uart, "START");
    send_payload(uart, setpoint.c_str());

    std::ofstream csv(options.csv_path);
    if (!csv) {
        std::fprintf(stderr, "motor_sim: could not open %s\n", options.csv_path);
        return 1;
    }
    csv.setf(std::ios::fixed);
    csv.precision(4);
    csv << "t_s,rpm_cmd,rpm_meas,duty,current_a,mode,fault\n";

    const int steps = static_cast<int>(options.seconds / options.dt);
    char tx_dump[128];
    for (int i = 0; i < steps; ++i) {
        if (i % 100 == 0) {
            send_payload(uart, setpoint.c_str());
        }
        controller.tick(options.dt, plant.current_a());
        plant.step(pwm.duty(), options.dt, encoder);
        static_cast<void>(uart.read_tx(tx_dump, static_cast<int>(sizeof(tx_dump))));

        const float t = static_cast<float>(i) * options.dt;
        csv << t << ',' << controller.commanded_rpm() << ',' << controller.measured_rpm() << ','
            << controller.duty() << ',' << plant.current_a() << ','
            << motor::mode_name(controller.mode()) << ',' << motor::fault_name(controller.fault())
            << '\n';
    }

    std::printf("wrote %s (%d samples), final rpm %.1f, duty %.3f, mode %s, fault %s\n",
                options.csv_path, steps, static_cast<double>(controller.measured_rpm()),
                static_cast<double>(controller.duty()), motor::mode_name(controller.mode()),
                motor::fault_name(controller.fault()));
    return controller.mode() == motor::Mode::Fault ? 1 : 0;
}
