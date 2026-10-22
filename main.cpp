#include <iostream>
#include <iomanip>
#include <cmath>
#include <array>
#include <unistd.h>
#include <algorithm> // std::min/max
#include <chrono>
#include <string>

#include "xsens_interface.hpp"
#include "realTimeHandler.h"

#include "i2c_bus.hpp"
#include "tca9548a.hpp"
#include "pololu_tic_i2c.hpp"

// ---------------- Parameters ----------------
constexpr float mass_kg = 0.65f;
constexpr float kp = 0.05f;
// constexpr float ki = 0.01f;
constexpr float ki = 0.05f;
constexpr float kd = 0.2f;

constexpr float R2D = 57.29577951f;
constexpr float D2R = 0.01745329251f;

constexpr float STEP_PER_METER = 160000.0f;

// ---------------- Axis config ----------------
constexpr uint8_t MUX_ADDR7 = 0x70;
constexpr uint8_t TIC_ADDR7 = 0x0E;

constexpr uint8_t CH_X = 0;
constexpr uint8_t CH_Y = 1;

// ---------------- Saturation ----------------
float sat(float v, float lo, float hi)
{
    return std::min(std::max(v, lo), hi);
}

// ---------------- 1st-order LPF (exp form) ----------------
struct FirstOrderLPFExp {
    float fc_Hz;   // cutoff
    float y;       // state
    bool inited;

    explicit FirstOrderLPFExp(float fc=1.0f)
        : fc_Hz(fc), y(0.0f), inited(false) {}

    void setCutoff(float fc) { fc_Hz = std::max(fc, 1e-3f); }

    float step(float x, float dt) {
        if (!inited) { y = x; inited = true; return y; }
        const float alpha = 1.0f - std::exp(-2.0f * static_cast<float>(M_PI) * fc_Hz * dt);
        y = y + alpha * (x - y);
        return y;
    }
};

void printXsensStats(const CallbackHandler& callback)
{
    const auto s = callback.receiveStats();
    std::cerr << "[XSENS] packets=" << s.packets
              << " orientation=" << s.orientation << " gyro=" << s.gyro
              << " complete=" << s.complete << " dropped=" << s.dropped
              << " errors=" << s.errors << " last_error=" << s.lastError << "\n";
}

int main(int argc, char** argv)
{
    const bool xsensOnly = argc == 2 && std::string(argv[1]) == "--xsens-only";
    if (argc > 1 && !xsensOnly) {
        std::cerr << "Usage: " << argv[0] << " [--xsens-only]\n";
        return 1;
    }
    // ---------------- Xsens ----------------
    XsControl* control = nullptr;
    XsDevice* device = nullptr;
    CallbackHandler callback;

    if (!setupXsens(control, device, callback)) {
        std::cerr << "[ERROR] Xsens init failed\n";
        shutdownXsens(control, device);
        return -1;
    }

    if (xsensOnly) {
        std::cerr << "[XSENS] sensor-only diagnostic for 10 seconds\n";
        using Clock = std::chrono::steady_clock;
        const auto start = Clock::now();
        auto report = start;
        unsigned int states = 0;
        float rpy[3] = {}, pqr[3] = {};
        while (Clock::now() - start < std::chrono::seconds(10)) {
            if (callback.getStates(rpy, pqr)) ++states;
            if (Clock::now() >= report) {
                printXsensStats(callback);
                std::cerr << "[XSENS] states_read=" << states
                          << " last_rpy_deg=" << rpy[0] << "," << rpy[1] << "," << rpy[2]
                          << " last_gyro_rad_s=" << pqr[0] << "," << pqr[1] << "," << pqr[2] << "\n";
                report += std::chrono::seconds(1);
            }
            usleep(5000);
        }
        printXsensStats(callback);
        shutdownXsens(control, device);
        return states ? 0 : 2;
    }

    // ---------------- I2C ----------------
    I2CBus bus("/dev/i2c-1");
    if (!bus.valid()) {
        std::cerr << "[ERROR] I2C open failed\n";
        shutdownXsens(control, device);
        return -1;
    }

    TCA9548A mux(MUX_ADDR7);
    TicI2C tic(TIC_ADDR7);

    // -------- axis init lambda --------
    auto init_axis = [&](const char* name, uint8_t ch) -> bool {
        if (!mux.select(bus, ch)) return false;

        if (!bus.probeAck(TIC_ADDR7)) {
            std::cerr << "[" << name << "] Tic not responding\n";
            return false;
        }
        tic.energize(bus);
        tic.exitSafeStart(bus);
        tic.resetCommandTimeout(bus);
        return true;
    };

    if (!init_axis("X", CH_X)) return -1;
    if (!init_axis("Y", CH_Y)) return -1;

    mux.select(bus, CH_X);
    tic.resetCommandTimeout(bus);
    tic.exitSafeStart(bus);
    tic.setTargetPosition(bus, 0);
    usleep(10000);

    mux.select(bus, CH_Y);
    tic.resetCommandTimeout(bus);
    tic.exitSafeStart(bus);
    tic.setTargetPosition(bus, 0);
    usleep(2000000);

    std::cout << "Entering control loop...\n";

    // ---------------- Control state ----------------
    std::array<float, 2> eulerIntegral = {0.0f, 0.0f};

    float euler_deg[3] = {0};
    float euler_rad[3] = {0};
    float bodyRate[3]  = {0};

    // ---- loop time ----
    const float sampleTime_s = 0.02f;

    // ---- LPF cutoff (Hz) ----
    float cutoff_Hz = 3.0f;

    FirstOrderLPFExp lpf_wx(cutoff_Hz);
    FirstOrderLPFExp lpf_wy(cutoff_Hz);

    // initSystemConfigs();
    PeriodicTimer timer;
    startTimer(&timer, sampleTime_s);

    bool firstStateRead = true;
    unsigned int noStateCount = 0;

    while (true) {

        if (firstStateRead) {
            std::cerr << "[DEBUG] before first getStates\n";
        }

        const bool hasStates = callback.getStates(euler_deg, bodyRate);

        if (firstStateRead) {
            std::cerr << "[DEBUG] after first getStates: "
                      << (hasStates ? "data received" : "no data") << "\n";
            firstStateRead = false;
        }

        if (hasStates) {
            noStateCount = 0;

            euler_rad[0] = euler_deg[0] * D2R;
            euler_rad[1] = euler_deg[1] * D2R;

            lpf_wx.setCutoff(cutoff_Hz);
            lpf_wy.setCutoff(cutoff_Hz);

            const float wx_f = lpf_wx.step(bodyRate[0], sampleTime_s);
            const float wy_f = lpf_wy.step(bodyRate[1], sampleTime_s);

            std::array<float,2> torque = {
                -kp * euler_rad[0] - ki * eulerIntegral[0] - kd * wx_f,
                -kp * euler_rad[1] - ki * eulerIntegral[1] - kd * wy_f
            };

            std::array<float,2> pos_m = {
                torque[0] / (mass_kg * 9.81f),
                torque[1] / (mass_kg * 9.81f)
            };

            pos_m[0] = sat(pos_m[0], -0.05f, 0.05f);
            pos_m[1] = sat(pos_m[1], -0.05f, 0.05f);

            eulerIntegral[0] += euler_rad[0] * sampleTime_s;
            eulerIntegral[1] += euler_rad[1] * sampleTime_s;

            int32_t pulseX = static_cast<int32_t>(pos_m[0] * STEP_PER_METER);
            int32_t pulseY = static_cast<int32_t>(pos_m[1] * STEP_PER_METER);

            // ---- motor command ----
            mux.select(bus, CH_X);
            tic.resetCommandTimeout(bus);
            tic.exitSafeStart(bus);
            tic.setTargetPosition(bus, pulseX);

            mux.select(bus, CH_Y);
            tic.resetCommandTimeout(bus);
            tic.exitSafeStart(bus);
            tic.setTargetPosition(bus, pulseY);

            std::cout << std::fixed << std::setprecision(2)
                      << "Roll "  << std::setw(7) << euler_deg[0]
                      << " | Pitch " << std::setw(7) << euler_deg[1]
                      << " | wx " << std::setw(7) << wx_f * R2D
                      << " | wy " << std::setw(7) << wy_f * R2D
                      << " | X " << std::setw(7) << pos_m[0] * 100.0f
                      << " | Y " << std::setw(7) << pos_m[1] * 100.0f
                      << "\n";
        } else if (++noStateCount >= 50) {
            std::cerr << "[WARN] no complete Xsens state received for 1 second\n";
            printXsensStats(callback);
            noStateCount = 0;
        }

        waitNextPeriod(&timer);
    }

    // Halt each axis and set current position = last target (or whatever semantics your driver has)
    // mux.select(bus, CH_X);
    // tic.haltAndSetPosition(bus, 0);

    // mux.select(bus, CH_Y);
    // tic.haltAndSetPosition(bus, 0);

    shutdownXsens(control, device);
    return 0;
}
