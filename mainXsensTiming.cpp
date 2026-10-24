// Diagnostic copy of main.cpp: keep Xsens setup and the 50 Hz timer only.
// Reuse getStates() to select the latest packet and clear older packets; never send motor commands.
#include "xsens_interface.hpp"
#include "realTimeHandler.h"

#include <algorithm>
#include <chrono>
#include <array>
#include <cmath>
#include <csignal>
#include <deque>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>

using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;

static double milliseconds(Time end, Time start) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

struct Stats {
    unsigned count = 0;
    double sum = 0;
    double minimum = 0;
    double maximum = 0;

    void add(double value) {
        if (count == 0) minimum = maximum = value;
        minimum = std::min(minimum, value);
        maximum = std::max(maximum, value);
        sum += value;
        ++count;
    }
    double mean() const { return count ? sum / count : 0; }
};

// Same exponential first-order LPF as main.cpp, applied to every gyro sample.
struct GyroLPF {
    explicit GyroLPF(float sampleTime)
        : alpha(1.0f - std::exp(-2.0f * 3.14159265359f * 3.0f * sampleTime)) {}

    void step(const XsVector& input) {
        for (size_t axis = 0; axis < value.size(); ++axis) {
            const float x = static_cast<float>(input[axis]);
            value[axis] = initialized ? value[axis] + alpha * (x - value[axis]) : x;
        }
        initialized = true;
    }

    const float alpha;
    bool initialized = false;
    std::array<float, 3> value = {{0, 0, 0}};
};

// Euler angles are degrees. Use the shortest angular difference at +/-180 deg.
// This component-wise filter is intended for the near-horizontal operating range.
struct EulerLPF {
    explicit EulerLPF(float sampleTime)
        : alpha(1.0f - std::exp(-2.0f * 3.14159265359f * 3.0f * sampleTime)) {}

    void step(const XsEuler& input) {
        const std::array<float, 3> angle = {{static_cast<float>(input.roll()),
            static_cast<float>(input.pitch()), static_cast<float>(input.yaw())}};
        for (size_t axis = 0; axis < value.size(); ++axis) {
            if (!initialized) value[axis] = angle[axis];
            else {
                const float delta = std::remainder(angle[axis] - value[axis], 360.0f);
                value[axis] = std::remainder(value[axis] + alpha * delta, 360.0f);
            }
        }
        initialized = true;
    }

    const float alpha;
    bool initialized = false;
    std::array<float, 3> value = {{0, 0, 0}};
};

// Welford's algorithm avoids cancellation when the signal has a nonzero bias.
struct Variance {
    unsigned count = 0;
    double mean = 0;
    double m2 = 0;

    void add(double x) {
        ++count;
        const double delta = x - mean;
        mean += delta / count;
        m2 += delta * (x - mean);
    }
    double stddev() const { return count > 1 ? std::sqrt(m2 / (count - 1)) : 0; }
};

struct AxisNoise {
    std::array<Variance, 3> raw;
    std::array<Variance, 3> filtered;
};

struct ReceiveReport {
    unsigned received = 0;
    unsigned dropped = 0;  // Buffer overflow at reception.
    unsigned skipped = 0;  // Older packets discarded when reading the latest.
    size_t queued = 0;
    Stats interval;
};

class TimingCallback : public CallbackHandler {
public:
    // Keep raw packets for before/after comparisons; local diagnostic LPFs run below.
    TimingCallback() : CallbackHandler(bufferSize, false) {}

    // Also used with explicit timestamps by the deterministic self-test.
    void receive(const XsDataPacket& packet, Time arrival) {
        std::lock_guard<std::mutex> lock(mutex);
        if (havePreviousArrival) report.interval.add(milliseconds(arrival, previousArrival));
        previousArrival = arrival;
        havePreviousArrival = true;
        ++report.received;
        if (packet.containsCalibratedGyroscopeData()) {
            const XsVector gyro = packet.calibratedGyroscopeData();
            receiveLPF.step(gyro);  // 100 Hz sensor time, not USB arrival jitter.
            if (arrival >= noiseStart) {
                for (size_t axis = 0; axis < 3; ++axis) {
                    noise.raw[axis].add(gyro[axis]);
                    noise.filtered[axis].add(receiveLPF.value[axis]);
                }
            }
        }
        if (packet.containsOrientation()) {
            const XsEuler euler = packet.orientationEuler();
            receiveEulerLPF.step(euler);
            if (arrival >= noiseStart) {
                const std::array<float, 3> angle = {{static_cast<float>(euler.roll()),
                    static_cast<float>(euler.pitch()), static_cast<float>(euler.yaw())}};
                if (!haveEulerReference) {
                    eulerReference = angle;
                    haveEulerReference = true;
                }
                for (size_t axis = 0; axis < 3; ++axis) {
                    // Measure angular deviations so crossing +/-180 does not inflate noise.
                    eulerNoise.raw[axis].add(std::remainder(angle[axis] - eulerReference[axis], 360.0f));
                    eulerNoise.filtered[axis].add(std::remainder(
                        receiveEulerLPF.value[axis] - eulerReference[axis], 360.0f));
                }
            }
        }
        if (arrivals.size() == bufferSize) {
            arrivals.pop_front();
            ++report.dropped;
        }
        CallbackHandler::onLiveDataAvailable(nullptr, &packet);
        arrivals.push_back(arrival);
    }

    // False means an empty queue. 'valid' separately reports missing fields.
    bool read(float* euler, float* rate, double& ageMs, bool& valid,
              float* filteredRate = nullptr, float* filteredEuler = nullptr) {
        std::lock_guard<std::mutex> lock(mutex);
        if (arrivals.empty()) return false;
        ageMs = milliseconds(Clock::now(), arrivals.back());
        report.skipped += arrivals.size() - 1;
        arrivals.clear();
        valid = CallbackHandler::getStates(euler, rate);
        if (valid && filteredRate)
            std::copy(receiveLPF.value.begin(), receiveLPF.value.end(), filteredRate);
        if (valid && filteredEuler)
            std::copy(receiveEulerLPF.value.begin(), receiveEulerLPF.value.end(), filteredEuler);
        return true;
    }

    void beginNoiseMeasurement(Time start) {
        std::lock_guard<std::mutex> lock(mutex);
        noiseStart = start;
        noise = AxisNoise();
        eulerNoise = AxisNoise();
        haveEulerReference = false;
    }

    AxisNoise noiseSummary() {
        std::lock_guard<std::mutex> lock(mutex);
        return noise;
    }

    AxisNoise eulerNoiseSummary() {
        std::lock_guard<std::mutex> lock(mutex);
        return eulerNoise;
    }

    ReceiveReport takeReport() {
        std::lock_guard<std::mutex> lock(mutex);
        ReceiveReport result = report;
        result.queued = arrivals.size();
        report = ReceiveReport();
        return result;
    }

protected:
    void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) override {
        receive(*packet, Clock::now());
    }

private:
    static constexpr size_t bufferSize = 5;
    std::mutex mutex;
    std::deque<Time> arrivals;
    bool havePreviousArrival = false;
    Time previousArrival;
    ReceiveReport report;
    GyroLPF receiveLPF{0.01f};
    EulerLPF receiveEulerLPF{0.01f};
    Time noiseStart = Time::max();
    AxisNoise noise;
    AxisNoise eulerNoise;
    bool haveEulerReference = false;
    std::array<float, 3> eulerReference = {{0, 0, 0}};
};

static volatile std::sig_atomic_t stopRequested = 0;
static void requestStop(int) { stopRequested = 1; }

static bool selfTest() {
    TimingCallback callback;
    const Time origin = Clock::now();
    auto packet = [](int index) {
        XsDataPacket result;
        result.setOrientationEuler(XsEuler(index, 0, 0), XDI_CoordSysEnu);
        XsVector rate(3);
        rate[0] = index;
        rate[1] = rate[2] = 0;
        result.setCalibratedGyroscopeData(rate);
        return result;
    };
    auto check = [](bool condition, const char* message) {
        if (!condition) std::cerr << "FAIL: " << message << '\n';
        return condition;
    };
    float euler[3] = {}, rate[3] = {};
    double age = 0;
    bool valid = false;
    if (!check(!callback.read(euler, rate, age, valid), "initial queue is empty")) return false;

    // Model 100 Hz arrival / 50 Hz consumption for one second.
    unsigned consumed = 0;
    for (int index = 0; index < 100; ++index) {
        callback.receive(packet(index), origin + std::chrono::milliseconds(index * 10));
        if (index % 2 == 1) {
            if (!check(callback.read(euler, rate, age, valid) && valid,
                       "complete packet available at each 50 Hz read")) return false;
            ++consumed;
            if (!check(euler[0] == index && rate[0] == index,
                       "latest packet and Euler/gyro pairing")) return false;
            if (!check(!callback.packetAvailable(), "original buffer cleared after latest read")) return false;
        }
    }
    ReceiveReport report = callback.takeReport();
    if (!check(report.received == 100 && consumed == 50 && report.dropped == 0 &&
               report.skipped == 50 && report.queued == 0,
               "100 arrivals = 50 reads + 50 skipped; no backlog")) return false;
    if (!check(report.interval.count == 99 && report.interval.mean() == 10,
               "100 Hz callback interval accounting")) return false;

    if (!check(!callback.read(euler, rate, age, valid), "no repeated state without new data")) return false;
    if (!check(!callback.getStates(euler, rate), "original getStates returns false when empty")) return false;

    // A burst larger than the buffer must still return the newest packet.
    for (int index = 0; index < 10; ++index)
        callback.receive(packet(index), Clock::now());
    if (!check(callback.read(euler, rate, age, valid) && valid && euler[0] == 9 && rate[0] == 9,
               "latest packet after buffer overflow")) return false;
    report = callback.takeReport();
    if (!check(report.received == 10 && report.dropped == 5 && report.skipped == 4 && report.queued == 0,
               "overflow and read-time discard counted separately")) return false;
    if (!check(!callback.packetAvailable(), "no stale backlog after burst")) return false;

    XsDataPacket incomplete;
    incomplete.setOrientationEuler(XsEuler(42, 0, 0), XDI_CoordSysEnu);
    callback.receive(packet(88), Clock::now());
    callback.receive(incomplete, Clock::now());
    if (!check(callback.read(euler, rate, age, valid) && !valid && euler[0] == 9 && rate[0] == 9,
               "incomplete packet consumed without partially updating outputs")) return false;
    if (!check(!callback.packetAvailable(), "incomplete packet is not retained")) return false;

    // Check callback-to-read age using a deliberately old arrival timestamp.
    callback.receive(packet(123), Clock::now() - std::chrono::milliseconds(40));
    if (!check(callback.read(euler, rate, age, valid) && valid && age >= 40,
               "callback-to-read age measurement")) return false;
    // Verify that a gyro sample discarded by the reader still affects the LPF.
    TimingCallback filteredCallback;
    filteredCallback.receive(packet(0), Clock::now());
    filteredCallback.receive(packet(1), Clock::now());
    filteredCallback.receive(packet(0), Clock::now());
    float filtered[3] = {}, filteredEuler[3] = {};
    const float alpha = 1.0f - std::exp(-2.0f * 3.14159265359f * 3.0f * 0.01f);
    if (!check(filteredCallback.read(euler, rate, age, valid, filtered, filteredEuler) && valid &&
               euler[0] == 0 && rate[0] == 0 &&
               std::abs(filtered[0] - alpha * (1.0f - alpha)) < 1e-6f &&
               std::abs(filteredEuler[0] - alpha * (1.0f - alpha)) < 1e-6f,
               "100 Hz gyro/Euler LPFs include intermediate samples; raw outputs stay unchanged")) return false;
    const auto filterBeforeEmptyRead = filtered[0];
    if (!check(!filteredCallback.read(euler, rate, age, valid, filtered) &&
               filtered[0] == filterBeforeEmptyRead,
               "empty reader does not advance the receive LPF")) return false;
    // Euler filtering must follow the short path across the angle boundary.
    TimingCallback wrappedCallback;
    XsDataPacket wrapped = packet(0);
    wrapped.setOrientationEuler(XsEuler(179, 0, 179), XDI_CoordSysEnu);
    wrappedCallback.receive(wrapped, Clock::now());
    wrapped.setOrientationEuler(XsEuler(-179, 0, -179), XDI_CoordSysEnu);
    wrappedCallback.receive(wrapped, Clock::now());
    if (!check(wrappedCallback.read(euler, rate, age, valid, filtered, filteredEuler) && valid &&
               std::abs(euler[0] + 179) < 1e-4f &&
               std::abs(filteredEuler[0] - (179 + alpha * 2)) < 1e-4f &&
               std::abs(filteredEuler[2] - (179 + alpha * 2)) < 1e-4f,
               "Euler +/-180 crossing uses shortest angular difference")) return false;
    EulerLPF constantEuler{0.01f};
    constantEuler.step(XsEuler(5, -3, 10));
    constantEuler.step(XsEuler(5, -3, 10));
    if (!check(constantEuler.value[0] == 5 && constantEuler.value[1] == -3 &&
               constantEuler.value[2] == 10, "Euler constant input stays unchanged")) return false;
    // Exercise the production receive LPF, independently of the diagnostic LPFs.
    class ProductionCallback : public CallbackHandler {
    public:
        void receive(const XsDataPacket& sample) { onLiveDataAvailable(nullptr, &sample); }
    } production;
    production.receive(packet(0));
    production.receive(packet(1));
    production.receive(packet(0));
    if (!check(production.getStates(euler, rate) && euler[0] == 0 &&
               std::abs(rate[0] - alpha * (1.0f - alpha)) < 1e-6f,
               "production gyro LPF uses all samples and leaves Euler raw")) return false;
    const float previousRate = rate[0];
    if (!check(!production.getStates(euler, rate) && rate[0] == previousRate,
               "production empty read does not repeat or refilter a packet")) return false;
    production.receive(packet(1));
    if (!check(production.getStates(euler, rate) && euler[0] == 1 &&
               std::abs(rate[0] - (previousRate + alpha * (1 - previousRate))) < 1e-6f,
               "production LPF state survives buffer clearing")) return false;
    // At 20 Hz each read follows five 100 Hz samples. Check samples discarded
    // by both read-time clearing and overflow still contribute to the filter.
    ProductionCallback slowReader;
    float expectedRate = 0;
    for (int index = 0; index < 20; ++index) {
        slowReader.receive(packet(index));
        expectedRate += alpha * (index - expectedRate);
        if (index % 5 == 4 &&
            !check(slowReader.getStates(euler, rate) && euler[0] == index &&
                   std::abs(rate[0] - expectedRate) < 1e-5f,
                   "20 Hz reader gets gyro filtered from all five incoming samples")) return false;
    }
    for (int index = 20; index < 30; ++index) {
        slowReader.receive(packet(index));
        expectedRate += alpha * (index - expectedRate);
    }
    if (!check(slowReader.getStates(euler, rate) && euler[0] == 29 &&
               std::abs(rate[0] - expectedRate) < 1e-5f,
               "production overflow does not skip LPF updates")) return false;
    std::cout << "PASS: production 100 Hz gyro LPF with 20 Hz reads; raw Euler; no double filtering\n";
    Variance variance;
    variance.add(1); variance.add(2); variance.add(3);
    if (!check(std::abs(variance.stddev() - 1.0) < 1e-9,
               "sample standard deviation")) return false;
    std::cout << "PASS: receive-time 100 Hz gyro/Euler LPFs, angular wrap, noise statistics\n";
    std::cout << "PASS: latest packet, buffer clearing, paired fields, overflow/skip counts, empty/incomplete packets, age\n"
              << "100 Hz / 50 Hz model: received=100 consumed=50 skipped=50 dropped=0 queued=0\n";
    return true;
}

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--self-test") return selfTest() ? 0 : 1;
    if (argc != 1) {
        std::cerr << "Usage: mainXsensTiming [--self-test]\n";
        return 1;
    }
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);
    XsControl* control = nullptr;
    XsDevice* device = nullptr;
    TimingCallback callback;
    if (!setupXsens(control, device, callback)) {
        std::cerr << "Xsens setup failed: check sensor connection and device access.\n";
        // shutdownXsens assumes a non-null device; destruct handles partial setup.
        if (control) control->destruct();
        return 1;
    }
    std::cout << "Port: " << device->portInfo().portName().toStdString()
              << " | Product: " << device->productCode().toStdString() << '\n'
              << "100 Hz sensor / 50 Hz latest-packet reader / 30 seconds / no motor commands\n"
              << "Gyro and Euler LPFs: 3 Hz at 100 Hz reception; raw values retained for comparison.\n"
              << "Keep sensor still; noise comparison excludes the first 2 seconds.\n"
              << "Times are ms. age is callback-to-read, not sensor-to-host latency.\n"
              << "work measures reader processing only; reporting and waiting are excluded.\n"
              << "Counters and min/mean/max are per reporting window; 0 samples means no measurement.\n"
              << "elapsed_s,rx,dropped,skipped,queued,read,empty,invalid,"
              << "rx_dt_n,rx_dt_min,rx_dt_mean,rx_dt_max,"
              << "age_n,age_min,age_mean,age_max,"
              << "loop_dt_n,loop_dt_min,loop_dt_mean,loop_dt_max,"
              << "work_n,work_min,work_mean,work_max\n";

    const float sampleTime = 0.02f;
    // Exclude setup-time callbacks from the first measurement window.
    // Keep queued packets so the existing reception startup behavior is preserved.
    callback.takeReport();
    const Time start = Clock::now();
    const Time noiseStart = start + std::chrono::seconds(2);
    callback.beginNoiseMeasurement(noiseStart);
    GyroLPF selectedLPF{sampleTime};  // Previous main.cpp behavior for comparison.
    AxisNoise selectedNoise;
    std::array<Variance, 3> receiveFilteredAtRead;
    Time previousLoop = start, previousReport = start;
    bool firstLoop = true;
    PeriodicTimer timer;
    startTimer(&timer, sampleTime);
    unsigned reads = 0, empty = 0, invalid = 0;
    Stats ages, periods, work;
    auto printStats = [](const Stats& stats) {
        std::cout << ',' << stats.count << ',' << stats.minimum
                  << ',' << stats.mean() << ',' << stats.maximum;
    };
    // The last row also reports a partial window when Ctrl+C is pressed.
    auto printReport = [&](Time now) {
        const ReceiveReport received = callback.takeReport();
        std::cout << std::fixed << std::setprecision(3)
                  << milliseconds(now, start) / 1000 << ',' << received.received
                  << ',' << received.dropped << ',' << received.skipped << ',' << received.queued
                  << ',' << reads << ',' << empty << ',' << invalid;
        printStats(received.interval);
        printStats(ages);
        printStats(periods);
        printStats(work);
        std::cout << std::endl;
        reads = empty = invalid = 0;
        ages = periods = work = Stats();
        previousReport = now;
    };

    while (!stopRequested && milliseconds(Clock::now(), start) < 30000) {
        const Time loopStart = Clock::now();
        if (!firstLoop) periods.add(milliseconds(loopStart, previousLoop));
        firstLoop = false;
        previousLoop = loopStart;
        float euler[3] = {}, rate[3] = {}, filteredRate[3] = {}, filteredEuler[3] = {};
        double age = 0;
        bool valid = false;
        if (callback.read(euler, rate, age, valid, filteredRate, filteredEuler)) {
            ++reads;
            ages.add(age);
            if (!valid) {
                ++invalid;
            } else {
                XsVector gyro(3);
                for (size_t axis = 0; axis < 3; ++axis) gyro[axis] = rate[axis];
                selectedLPF.step(gyro);
                if (loopStart >= noiseStart) {
                    for (size_t axis = 0; axis < 3; ++axis) {
                        selectedNoise.raw[axis].add(rate[axis]);
                        selectedNoise.filtered[axis].add(selectedLPF.value[axis]);
                        receiveFilteredAtRead[axis].add(filteredRate[axis]);
                    }
                }
            }
        } else {
            ++empty;
        }
        // Reader work only; report output and timer waiting are excluded.
        // Their effects on scheduling are visible in loop_dt.
        work.add(milliseconds(Clock::now(), loopStart));
        if (milliseconds(Clock::now(), previousReport) >= 1000) printReport(Clock::now());
        waitNextPeriod(&timer);
    }
    printReport(Clock::now());
    shutdownXsens(control, device);
    const AxisNoise receivedNoise = callback.noiseSummary();
    const AxisNoise receivedEulerNoise = callback.eulerNoiseSummary();
    std::cout << "noise_stream,axis,n,mean_rad_s,std_rad_s,std_deg_s\n";
    auto printNoise = [](const char* stream, size_t axis, const Variance& stats) {
        std::cout << std::setprecision(9) << stream << ',' << "xyz"[axis]
                  << ',' << stats.count << ',' << stats.mean << ',' << stats.stddev()
                  << ',' << stats.stddev() * 57.295779513 << '\n';
    };
    for (size_t axis = 0; axis < 3; ++axis) {
        printNoise("raw_receive_100hz", axis, receivedNoise.raw[axis]);
        printNoise("lpf_receive_100hz", axis, receivedNoise.filtered[axis]);
        printNoise("raw_selected_50hz", axis, selectedNoise.raw[axis]);
        printNoise("lpf_after_selection_50hz", axis, selectedNoise.filtered[axis]);
        printNoise("receive_lpf_selected_50hz", axis, receiveFilteredAtRead[axis]);
    }
    std::cout << "Euler noise uses deviations from the first measured angle (degrees).\n"
              << "euler_stream,axis,n,mean_delta_deg,std_deg\n";
    for (size_t axis = 0; axis < 3; ++axis) {
        const Variance& raw = receivedEulerNoise.raw[axis];
        const Variance& filtered = receivedEulerNoise.filtered[axis];
        std::cout << "euler_raw_receive_100hz," << "rpy"[axis] << ',' << raw.count
                  << ',' << raw.mean << ',' << raw.stddev() << '\n';
        std::cout << "euler_lpf_receive_100hz," << "rpy"[axis] << ',' << filtered.count
                  << ',' << filtered.mean << ',' << filtered.stddev() << '\n';
    }
    return 0;
}
