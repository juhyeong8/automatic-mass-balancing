#include <iostream>
#include <iomanip>
#include <cmath>
#include <unistd.h>

#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>

#include "xsens_interface.hpp"
#include "realTimeHandler.h"

// --------- wall-clock timestamp: "YYYY-MM-DD HH:MM:SS.mmm" ----------
static std::string now_timestamp_ms()
{
    using namespace std::chrono;

    const auto now = system_clock::now();
    const auto ms  = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

    std::time_t tt = system_clock::to_time_t(now);

    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif

    std::ostringstream oss;
    oss << std::put_time(&tm, "%F %T")  // YYYY-MM-DD HH:MM:SS
        << '.' << std::setw(3) << std::setfill('0') << ms.count();
    return oss.str();
}

// --------- monotonic time in seconds (for loop timing) ----------
static double monotonic_sec()
{
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) + 1e-9 * static_cast<double>(ts.tv_nsec);
}

int main()
{
    // ---------------- Xsens ----------------
    XsControl* control = nullptr;
    XsDevice* device   = nullptr;
    CallbackHandler callback;

    if (!setupXsens(control, device, callback)) {
        std::cerr << "[ERROR] Xsens init failed\n";
        return -1;
    }

    // NOTE: 0.02s is 50 Hz, not 10 Hz
    constexpr float sampleTime_s = 0.02f;

    // ---------------- CSV file ----------------
    std::ofstream log("../xsens_log.csv", std::ios::out | std::ios::trunc);
    if (!log.is_open()) {
        std::cerr << "[ERROR] Failed to open xsens_log.csv\n";
        shutdownXsens(control, device);
        return -1;
    }

    // header
    log << "timestamp,roll_deg,pitch_deg,yaw_deg,wx_rps,wy_rps,wz_rps,loop_ms\n";
    log << std::fixed << std::setprecision(6);

    initSystemConfigs();
    PeriodicTimer timer;
    startTimer(&timer, sampleTime_s);

    float euler_deg[3]     = {0};
    float bodyRate_rps[3]  = {0};

    double t_prev = monotonic_sec();

    while (true) {

        // measure loop period in ms (monotonic, not affected by NTP/time changes)
        const double t_now = monotonic_sec();
        const double loop_ms = (t_now - t_prev) * 1000.0;
        t_prev = t_now;

        if (callback.getStates(euler_deg, bodyRate_rps)) {

            // timestamp is wall-clock for human readability / matching other logs
            const std::string ts = now_timestamp_ms();

            std::cout << loop_ms << "ms\n";

            log << ts << ','
                << euler_deg[0] << ','
                << euler_deg[1] << ','
                << euler_deg[2] << ','
                << bodyRate_rps[0] << ','
                << bodyRate_rps[1] << ','
                << bodyRate_rps[2] << ','
                << loop_ms
                << '\n';

            // optional: flush occasionally so you don't lose data on crash
            // static int cnt=0; if ((++cnt % 50)==0) log.flush();
        }

        waitNextPeriod(&timer);
    }

    // unreachable unless you add break condition
    shutdownXsens(control, device);
    return 0;
}
