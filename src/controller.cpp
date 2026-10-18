#include "motor/controller.hpp"

#include "motor/detail/exhaustive.hpp"

#include <cstring>

namespace motor {
namespace {

constexpr int kRxChunk = 48;

}  // namespace

SpeedController::SpeedController(hal::Pwm& pwm, hal::Encoder& encoder, hal::Uart& uart,
                                 ControllerConfig config)
    : pwm_(pwm),
      encoder_(encoder),
      uart_(uart),
      config_(config),
      pid_(config.gains, config.limits),
      ramp_(config.max_rpm_per_s),
      estimator_(config.counts_per_rev, config.speed_lpf_alpha),
      faults_(config.faults),
      watchdog_(config.watchdog_timeout_ticks) {}

void SpeedController::tick(float dt_s, float current_a) {
    measured_rpm_ = estimator_.step(encoder_.count(), dt_s);
    service_uart();

    if (state_.mode() == Mode::Run && watchdog_.tick()) {
        state_.trip(FaultCode::Watchdog);
    }

    float duty = 0.f;
    if (state_.mode() == Mode::Run) {
        const float setpoint = ramp_.step(target_rpm_, dt_s);
        duty = pid_.step(setpoint, measured_rpm_, dt_s);
    } else {
        pid_.reset();
        ramp_.reset(0.f);
    }

    const FaultCode detected =
        faults_.update(measured_rpm_, duty, current_a, state_.mode() == Mode::Run);
    if (detected != FaultCode::None) {
        state_.trip(detected);
    }
    if (state_.mode() != Mode::Run) {
        duty = 0.f;
    }
    pwm_.set_duty(duty);
}

void SpeedController::service_uart() {
    char rx[kRxChunk];
    const int n = uart_.read(rx, kRxChunk);
    for (int i = 0; i < n; ++i) {
        Frame frame;
        if (parser_.feed(rx[i], frame)) {
            handle_frame(frame);
        }
    }
}

void SpeedController::handle_frame(const Frame& frame) {
    if (!frame.ok) {
        if (frame.error != nullptr && std::strcmp(frame.error, "bad_checksum") == 0) {
            reply("ERR bad_checksum");
        } else {
            reply("ERR bad_frame");
        }
        return;
    }

    Command command;
    const char* error = nullptr;
    if (!decode_command(frame.payload, frame.length, command, error)) {
        if (error != nullptr && std::strcmp(error, "bad_arg") == 0) {
            reply("ERR bad_arg");
        } else {
            reply("ERR unknown_command");
        }
        return;
    }
    handle_command(command);
}

void SpeedController::handle_command(const Command& command) {
    switch (command.id) {
        case CommandId::SetRpm: {
            if (state_.mode() == Mode::Fault) {
                reply("ERR fault");
                break;
            }
            const auto limit = static_cast<std::int32_t>(config_.max_abs_rpm);
            if (command.rpm > limit || command.rpm < -limit) {
                reply("ERR bad_arg");
                break;
            }
            target_rpm_ = static_cast<float>(command.rpm);
            watchdog_.kick();
            reply("OK");
            break;
        }
        case CommandId::GetStatus:
            reply_status();
            break;
        case CommandId::Start:
            if (state_.mode() == Mode::Fault) {
                reply("ERR fault");
                break;
            }
            if (!state_.start()) {
                reply("ERR state");
                break;
            }
            enter_run();
            reply("OK");
            break;
        case CommandId::Stop:
            if (!state_.stop()) {
                reply("ERR state");
                break;
            }
            pid_.reset();
            ramp_.reset(0.f);
            reply("OK");
            break;
        case CommandId::ClearFault:
            if (!state_.clear_fault()) {
                reply("ERR state");
                break;
            }
            faults_.reset();
            watchdog_.reset();
            pid_.reset();
            ramp_.reset(0.f);
            reply("OK");
            break;
        case CommandId::Ping:
            watchdog_.kick();
            reply("OK");
            break;
        default:
            detail::unhandled_enum(command.id);
            break;
    }
}

void SpeedController::reply(const char* payload) {
    char frame[96];
    const int n = format_frame(frame, static_cast<int>(sizeof(frame)), payload);
    if (n > 0) {
        static_cast<void>(uart_.write(frame, n));
    }
}

void SpeedController::reply_status() {
    char payload[80];
    const int n = format_status(payload, static_cast<int>(sizeof(payload)), state_.mode(),
                                state_.fault(), target_rpm_, measured_rpm_, pwm_.duty());
    if (n < 0) {
        reply("ERR internal");
        return;
    }
    reply(payload);
}

void SpeedController::enter_run() {
    pid_.reset();
    ramp_.reset(measured_rpm_);
    watchdog_.reset();
    watchdog_.kick();
}

}  // namespace motor
