#include <iostream>
#include <iomanip>
#include <cmath>
#include <array>
#include <csignal>
#include <cstdio>
#include <unistd.h>
#include <algorithm>   // std::min/max
#include <termios.h>
#include <fcntl.h>
#include "xsens_interface.hpp"
#include "realTimeHandler.h"
#include "i2c_bus.hpp"
#include "tca9548a.hpp"
#include "pololu_tic_i2c.hpp"

// ---------------- Parameters ----------------
constexpr float mass_kg = 0.65f;
constexpr float kp = 0.05f;
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

static volatile sig_atomic_t g_stop = 0;
static void onSigint(int) { g_stop = 1; }

float sat(float v, float lo, float hi) { return std::min(std::max(v, lo), hi); }

// ---------------- 1st-order LPF (exp form) ----------------
struct FirstOrderLPFExp {
    float fc_Hz;
    float y;
    bool inited;
    explicit FirstOrderLPFExp(float fc = 1.0f) : fc_Hz(fc), y(0.0f), inited(false) {}
    void setCutoff(float fc) { fc_Hz = std::max(fc, 1e-3f); }
    float step(float x, float dt) {
        if (!inited) { y = x; inited = true; return y; }
        const float alpha = 1.0f - std::exp(-2.0f * static_cast<float>(M_PI) * fc_Hz * dt);
        y = y + alpha * (x - y);
        return y;
    }
};

// ---------------- Non-blocking keyboard ----------------
struct KeyboardNonBlocking {
    termios orig{};
    bool ok{false};
    bool init() {
        if (tcgetattr(STDIN_FILENO, &orig) != 0) return false;
        termios raw = orig;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN]  = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) return false;
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (flags < 0) return false;
        if (fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) != 0) return false;
        ok = true;
        return true;
    }
    int readChar() {
        if (!ok) return -1;
        unsigned char c;
        return (::read(STDIN_FILENO, &c, 1) == 1) ? static_cast<int>(c) : -1;
    }
    void restore() {
        if (ok) tcsetattr(STDIN_FILENO, TCSANOW, &orig);
        ok = false;
    }
    ~KeyboardNonBlocking() { restore(); }
};

int main()
{
    std::signal(SIGINT, onSigint);
    std::printf("[1] enter main\n"); std::fflush(stdout);

    // ---------------- Xsens ----------------
    XsControl* control = nullptr;
    XsDevice*  device  = nullptr;
    CallbackHandler callback;

    std::printf("[2] setupXsens() ...\n"); std::fflush(stdout);
    if (!setupXsens(control, device, callback)) {
        std::fprintf(stderr, "[ERROR] Xsens init failed\n");
        return -1;
    }
    std::printf("[3] setupXsens OK\n"); std::fflush(stdout);

    // ---------------- I2C ----------------
    std::printf("[4] I2C open ...\n"); std::fflush(stdout);
    I2CBus bus("/dev/i2c-1");
    if (!bus.valid()) {
        std::fprintf(stderr, "[ERROR] I2C open failed\n");
        shutdownXsens(control, device);
        return -1;
    }
    std::printf("[5] I2C OK\n"); std::fflush(stdout);

    TCA9548A mux(MUX_ADDR7);
    TicI2C   tic(TIC_ADDR7);

    auto init_axis = [&](const char* name, uint8_t ch) -> bool {
        std::printf("    init axis %s (ch=%u) ...\n", name, (unsigned)ch); std::fflush(stdout);
        if (!mux.select(bus, ch)) { std::fprintf(stderr, "[%s] mux select failed\n", name); return false; }
        if (!bus.probeAck(TIC_ADDR7)) { std::fprintf(stderr, "[%s] Tic not responding\n", name); return false; }
        tic.energize(bus);
        tic.exitSafeStart(bus);
        tic.resetCommandTimeout(bus);
        std::printf("    axis %s OK\n", name); std::fflush(stdout);
        return true;
    };

    std::printf("[6] axis init ...\n"); std::fflush(stdout);
    if (!init_axis("X", CH_X)) { shutdownXsens(control, device); return -1; }
    if (!init_axis("Y", CH_Y)) { shutdownXsens(control, device); return -1; }

    mux.select(bus, CH_X);
    tic.resetCommandTimeout(bus);
    tic.exitSafeStart(bus);
    tic.setTargetPosition(bus, 0);
    usleep(10000);

    mux.select(bus, CH_Y);
    tic.resetCommandTimeout(bus);
    tic.exitSafeStart(bus);
    tic.setTargetPosition(bus, 0);

    std::printf("[7] homing, wait 2s ...\n"); std::fflush(stdout);
    usleep(2000000);

    // ---- Keyboard ----
    KeyboardNonBlocking kb;
    if (!kb.init()) {
        std::fprintf(stderr, "[WARN] keyboard init failed (Ctrl+C still works)\n");
    } else {
        std::printf("Press 'q' to stop\n");
    }

    // ---------------- Control state ----------------
    std::array<float, 2> eulerIntegral = {0.0f, 0.0f};
    float euler_deg[3] = {0};
    float euler_rad[3] = {0};
    float bodyRate[3]  = {0};

    const float sampleTime_s = 0.02f;   // 50 Hz
    float cutoff_Hz = 3.0f;
    FirstOrderLPFExp lpf_wx(cutoff_Hz);
    FirstOrderLPFExp lpf_wy(cutoff_Hz);

    std::printf("[8] initSystemConfigs() ...\n"); std::fflush(stdout);
    initSystemConfigs();
    std::printf("[9] initSystemConfigs OK\n"); std::fflush(stdout);

    PeriodicTimer timer;
    std::printf("[10] startTimer() ...\n"); std::fflush(stdout);
    startTimer(&timer, sampleTime_s);
    std::printf("[11] entering control loop\n"); std::fflush(stdout);

    long loop = 0, got = 0;
    bool stopRequested = false;

    while (!stopRequested && !g_stop) {
        ++loop;

        const int ch = kb.readChar();
        if (ch == 'q' || ch == 'Q') stopRequested = true;

        if (callback.getStates(euler_deg, bodyRate)) {
            ++got;

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
            eulerIntegral[0] = sat(eulerIntegral[0], -1.0f, 1.0f);   // anti-windup
            eulerIntegral[1] = sat(eulerIntegral[1], -1.0f, 1.0f);

            const int32_t pulseX = static_cast<int32_t>(pos_m[0] * STEP_PER_METER);
            const int32_t pulseY = static_cast<int32_t>(pos_m[1] * STEP_PER_METER);

            // ---- motor command ----
            mux.select(bus, CH_X);
            tic.resetCommandTimeout(bus);
            tic.exitSafeStart(bus);
            tic.setTargetPosition(bus, pulseX);

            mux.select(bus, CH_Y);
            tic.resetCommandTimeout(bus);
            tic.exitSafeStart(bus);
            tic.setTargetPosition(bus, pulseY);

            if (loop % 5 == 0) {   // 10 Hz 로만 출력
                std::printf("Roll %7.2f | Pitch %7.2f | wx %7.2f | wy %7.2f | X %7.2f | Y %7.2f\n",
                            (double)euler_deg[0], (double)euler_deg[1],
                            (double)(wx_f * R2D), (double)(wy_f * R2D),
                            (double)(pos_m[0] * 100.0f), (double)(pos_m[1] * 100.0f));
                std::fflush(stdout);
            }
        }

        // 1초에 한 번, IMU 데이터가 전혀 없으면 패킷 수신 통계 출력
        if (loop % 50 == 0 && got == 0) {
            long rx = 0, ori = 0, gyr = 0;
            callback.getStats(rx, ori, gyr);
            std::printf("[hb] loop=%ld got=0  rx=%ld  withOri=%ld  withGyr=%ld\n",
                        loop, rx, ori, gyr);
            std::fflush(stdout);
        }

        waitNextPeriod(&timer);
    }

    long rx = 0, ori = 0, gyr = 0;
    callback.getStats(rx, ori, gyr);
    std::printf("\n[12] loop exit: loop=%ld got=%ld | rx=%ld withOri=%ld withGyr=%ld\n",
                loop, got, rx, ori, gyr);

    kb.restore();
    shutdownXsens(control, device);
    std::printf("[13] shutdown done\n");
    return 0;
}