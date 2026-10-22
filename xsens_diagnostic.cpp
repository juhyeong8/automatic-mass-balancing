// Standalone sensor diagnostic: no motor/I2C initialization and no filter resets.
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xsstatusflag.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

Journaller* gJournal = nullptr;
static volatile std::sig_atomic_t stopRequested = 0;
static void requestStop(int) { stopRequested = 1; }

struct FieldCount {
    uint64_t count = 0;
    unsigned int length = 0;
};
struct DiagnosticStats {
    uint64_t raw = 0, malformed = 0, live = 0, errors = 0;
    uint64_t rawAcc = 0, rawGyro = 0, rawOrientation = 0;
    uint64_t acc = 0, gyro = 0, orientation = 0, mag = 0, status = 0;
    uint64_t invalidAcc = 0, invalidGyro = 0, invalidOrientation = 0;
    uint64_t selfTestNotOk = 0, orientationNotValid = 0, accClipped = 0;
    uint32_t lastStatus = 0;
    int lastError = 0;
    double lastAcc[3] = {}, lastGyro[3] = {}, lastEuler[3] = {};
    std::map<unsigned int, FieldCount> fields;
};

// Validate all TLV boundaries before handing a raw MTData2 message to XDA.
static bool collectFields(const XsMessage& message, DiagnosticStats& stats) {
    const XsSize size = message.getDataSize();
    for (XsSize offset = 0; offset < size;) {
        if (size - offset < 3) return false;
        const unsigned int id = message.getDataShort(offset);
        const unsigned int length = message.getDataByte(offset + 2);
        if (length > size - offset - 3) return false;
        auto& field = stats.fields[id];
        ++field.count;
        field.length = length;
        offset += 3 + length;
    }
    return true;
}
static bool copyVector(const XsVector& input, double* output) {
    if (input.size() != 3) return false;
    bool finite = true;
    for (int i = 0; i < 3; ++i) {
        output[i] = input[i];
        finite = finite && std::isfinite(output[i]);
    }
    return finite;
}

class DiagnosticCallback : public XsCallback {
    std::mutex mutex;
    DiagnosticStats stats;
public:
    void reset() { std::lock_guard<std::mutex> lock(mutex); stats = DiagnosticStats(); }
    DiagnosticStats snapshot() { std::lock_guard<std::mutex> lock(mutex); return stats; }
protected:
    void onMessageReceivedFromDevice(XsDevice*, const XsMessage* message) override {
        if (!message || message->getMessageId() != XMID_MtData2) return;
        std::lock_guard<std::mutex> lock(mutex);
        ++stats.raw;
        if (!collectFields(*message, stats)) { ++stats.malformed; return; }
        XsDataPacket packet(message);
        stats.rawAcc += packet.containsCalibratedAcceleration();
        stats.rawGyro += packet.containsCalibratedGyroscopeData();
        stats.rawOrientation += packet.containsOrientation();
    }
    void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) override {
        if (!packet) return;
        std::lock_guard<std::mutex> lock(mutex);
        ++stats.live;
        if (packet->containsCalibratedAcceleration()) {
            ++stats.acc;
            if (!copyVector(packet->calibratedAcceleration(), stats.lastAcc)) ++stats.invalidAcc;
        }
        if (packet->containsCalibratedGyroscopeData()) {
            ++stats.gyro;
            if (!copyVector(packet->calibratedGyroscopeData(), stats.lastGyro)) ++stats.invalidGyro;
        }
        if (packet->containsOrientation()) {
            ++stats.orientation;
            const auto euler = packet->orientationEuler();
            bool finite = true;
            for (int i = 0; i < 3; ++i) {
                stats.lastEuler[i] = euler[i];
                finite = finite && std::isfinite(euler[i]);
            }
            if (!finite) ++stats.invalidOrientation;
        }
        stats.mag += packet->containsCalibratedMagneticField();
        if (packet->containsStatus()) {
            ++stats.status;
            stats.lastStatus = packet->status();
            stats.selfTestNotOk += !(stats.lastStatus & XSF_SelfTestOk);
            stats.orientationNotValid += !(stats.lastStatus & XSF_OrientationValid);
            stats.accClipped += anyAccClipped(static_cast<int>(stats.lastStatus));
        }
    }
    void onError(XsDevice*, XsResultValue error) override {
        std::lock_guard<std::mutex> lock(mutex);
        ++stats.errors;
        stats.lastError = static_cast<int>(error);
    }
};

static void printConfig(const char* label, const XsOutputConfigurationArray& config) {
    std::cout << label << " items=" << config.size() << '\n';
    for (const auto& item : config)
        std::cout << "  id=0x" << std::hex << static_cast<unsigned int>(item.m_dataIdentifier)
                  << std::dec << " frequency=" << item.m_frequency << '\n';
}
static void printStats(const DiagnosticStats& s, bool fields) {
    std::cout << "raw=" << s.raw << " raw_acc=" << s.rawAcc << " raw_gyro=" << s.rawGyro
              << " raw_orientation=" << s.rawOrientation << " malformed=" << s.malformed
              << " live=" << s.live << " acc=" << s.acc << " gyro=" << s.gyro
              << " orientation=" << s.orientation << " mag=" << s.mag
              << " status=" << s.status << " errors=" << s.errors << " last_error=" << s.lastError << '\n';
    std::cout << "nonfinite: acc=" << s.invalidAcc << " gyro=" << s.invalidGyro
              << " orientation=" << s.invalidOrientation << '\n';
    if (s.acc) {
        const auto& a = s.lastAcc;
        std::cout << "last_acc_m_s2=" << a[0] << ',' << a[1] << ',' << a[2]
                  << " norm=" << std::sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]) << '\n';
    }
    if (s.gyro) std::cout << "last_gyro_rad_s=" << s.lastGyro[0] << ',' << s.lastGyro[1] << ',' << s.lastGyro[2] << '\n';
    if (s.orientation) std::cout << "last_euler_deg=" << s.lastEuler[0] << ',' << s.lastEuler[1] << ',' << s.lastEuler[2] << '\n';
    if (s.status)
        std::cout << "last_status=0x" << std::hex << s.lastStatus << std::dec
                  << " self_test_not_ok_samples=" << s.selfTestNotOk
                  << " orientation_not_valid_samples=" << s.orientationNotValid
                  << " acc_clipped_samples=" << s.accClipped << '\n';
    if (fields) for (const auto& entry : s.fields)
        std::cout << "  RAW_FIELD id=0x" << std::hex << entry.first << std::dec
                  << " last_length=" << entry.second.length << " count=" << entry.second.count << '\n';
    std::cout << std::flush;
}

static bool sameConfiguration(const XsOutputConfigurationArray& a,
                              const XsOutputConfigurationArray& b) {
    using Item = std::pair<unsigned int, unsigned int>;
    std::vector<Item> left, right;
    for (const auto& item : a) left.emplace_back(item.m_dataIdentifier, item.m_frequency);
    for (const auto& item : b) right.emplace_back(item.m_dataIdentifier, item.m_frequency);
    std::sort(left.begin(), left.end());
    std::sort(right.begin(), right.end());
    return left == right;
}

struct Session {
    XsControl* control = nullptr;
    XsDevice* device = nullptr;
    XsOutputConfigurationArray original;
    bool restore = false, wasMeasuring = false;
    bool close() {
        bool ok = true;
        if (restore && device) {
            auto config = original;
            ok = device->gotoConfig() && device->setOutputConfiguration(config);
            if (ok) ok = sameConfiguration(device->outputConfiguration(), original);
            if (ok && wasMeasuring) ok = device->gotoMeasurement();
            std::cout << "RESTORE original_output=" << (ok ? "OK" : "FAILED") << '\n';
            if (!ok) std::cerr << "Restore failed; check sensor output settings before further use.\n";
        }
        if (control) {
            if (device) control->closePort(device->portName());
            control->destruct();
        }
        control = nullptr; device = nullptr; restore = false;
        return ok;
    }
    ~Session() { close(); }
};

static int diagnose(const std::string& port, unsigned int seconds) {
    // Callback outlives the session, including shutdown and restoration.
    DiagnosticCallback callback;
    Session session;
    session.control = XsControl::construct();
    if (!session.control) { std::cerr << "Cannot construct XsControl\n"; return 1; }
    const auto ports = XsScanner::scanPorts();
    std::vector<XsPortInfo> candidates;
    for (const auto& entry : ports) {
        if (!(entry.deviceId().isMti() || entry.deviceId().isMtig())) continue;
        std::cout << "FOUND port=" << entry.portName().toStdString()
                  << " id=" << entry.deviceId().toString().toStdString() << '\n';
        if (port.empty() || entry.portName().toStdString() == port) candidates.push_back(entry);
    }
    if (candidates.size() != 1) {
        std::cerr << "Expected one matching MTi; use --port /dev/ttyUSB0 to select a device.\n";
        return 1;
    }
    const auto selected = candidates.front();
    if (!session.control->openPort(selected.portName(), selected.baudrate())) {
        std::cerr << "openPort failed: " << session.control->lastResultText().toStdString() << '\n';
        return 1;
    }
    session.device = session.control->device(selected.deviceId());
    auto* device = session.device;
    if (!device) return 1;
    std::cout << "DEVICE product=" << device->productCode().toStdString()
              << " firmware=" << device->firmwareVersion().toString().toStdString()
              << " hardware=" << device->hardwareVersion().toString().toStdString() << '\n';
    session.wasMeasuring = device->deviceState() == XDS_Measurement || device->deviceState() == XDS_Recording;
    if (!device->gotoConfig()) { std::cerr << "gotoConfig failed\n"; return 1; }
    auto original = device->outputConfiguration();
    session.original.swap(original);
    if (session.original.empty()) { std::cerr << "Cannot read original configuration; aborting before changing outputs.\n"; return 1; }
    session.restore = true;
    printConfig("ORIGINAL", session.original);
    device->addCallbackHandler(&callback);
    std::cout << "Keep the sensor still initially; then gently tilt it to check acceleration changes.\n"
              << "Three phases, " << seconds << " seconds each. Status flags are clues, not a hardware verdict.\n";
    const XsDataIdentifier attitude[] = {XDI_None, XDI_EulerAngles, XDI_Quaternion};
    const char* names[] = {"INERTIAL", "EULER", "QUATERNION"};
    int failed = 0;
    for (int phase = 0; phase < 3 && !stopRequested; ++phase) {
        std::cout << "\nPHASE " << names[phase] << '\n';
        if (!device->gotoConfig()) { ++failed; break; }
        XsOutputConfigurationArray config;
        config.push_back(XsOutputConfiguration(XDI_PacketCounter, 50));
        config.push_back(XsOutputConfiguration(XDI_Acceleration, 50));
        config.push_back(XsOutputConfiguration(XDI_RateOfTurn, 50));
        config.push_back(XsOutputConfiguration(XDI_MagneticField, 50));
        config.push_back(XsOutputConfiguration(XDI_StatusWord, 50));
        if (attitude[phase] != XDI_None) config.push_back(XsOutputConfiguration(attitude[phase], 50));
        printConfig("REQUESTED", config);
        if (!device->setOutputConfiguration(config)) {
            std::cerr << "setOutputConfiguration failed: " << device->lastResultText().toStdString() << '\n';
            ++failed; continue;
        }
        printConfig("ACKNOWLEDGED", config);
        printConfig("READ_BACK", device->outputConfiguration());
        callback.reset();
        if (!device->gotoMeasurement()) {
            std::cerr << "gotoMeasurement failed: " << device->lastResultText().toStdString() << '\n';
            ++failed; continue;
        }
        using Clock = std::chrono::steady_clock;
        const auto start = Clock::now();
        auto nextReport = start + std::chrono::seconds(1);
        while (!stopRequested && Clock::now() - start < std::chrono::seconds(seconds)) {
            if (Clock::now() >= nextReport) {
                printStats(callback.snapshot(), false);
                nextReport += std::chrono::seconds(1);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (!device->gotoConfig()) { ++failed; break; }
        const auto result = callback.snapshot();
        std::cout << "SUMMARY " << names[phase] << '\n';
        printStats(result, true);
        if (!result.acc || !result.gyro || result.invalidAcc || result.invalidGyro ||
            result.malformed || result.errors || (phase && (!result.orientation || result.invalidOrientation))) ++failed;
    }
    const bool restored = session.close();
    std::cout << "Diagnostic complete. Missing data does not by itself prove hardware failure.\n";
    if (!restored) return 1;
    if (stopRequested) return 130;
    return failed ? 2 : 0;
}

#ifndef XSENS_DIAGNOSTIC_NO_MAIN
int main(int argc, char** argv) {
    std::string port;
    unsigned int seconds = 10;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--port" && i + 1 < argc) port = argv[++i];
        else if (arg == "--seconds" && i + 1 < argc) {
            std::istringstream input(argv[++i]);
            int value = 0;
            if (!(input >> value) || !input.eof() || value < 1 || value > 120) {
                std::cerr << "--seconds must be an integer from 1 to 120\n"; return 1;
            }
            seconds = static_cast<unsigned int>(value);
        } else {
            std::cerr << "Usage: " << argv[0] << " [--port /dev/ttyUSB0] [--seconds 10]\n";
            return arg == "--help" ? 0 : 1;
        }
    }
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);
    std::cout << std::fixed << std::setprecision(6);
    return diagnose(port, seconds);
}
#endif
