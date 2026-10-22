// ===== xsens_interface.cpp =====
#include "xsens_interface.hpp"
#include <xstypes/xsmessage.h>
#include <xstypes/xsfilterprofile.h>
#include <xstypes/xsfilterprofilearray.h>
#include <iostream>
#include <cstdio>

Journaller* gJournal = 0;

CallbackHandler::CallbackHandler(size_t maxBufferSize)
    : m_maxNumberOfPacketsInBuffer(maxBufferSize),
      m_numberOfPacketsInBuffer(0),
      m_rxPackets(0), m_rxWithOri(0), m_rxWithGyr(0),
      m_dumpLive(0), m_dumpProc(0), m_announcedOri(false)
{
}

CallbackHandler::~CallbackHandler() {}

bool CallbackHandler::packetAvailable() const {
    xsens::Lock locky(&m_mutex);
    return m_numberOfPacketsInBuffer > 0;
}

XsDataPacket CallbackHandler::getNextPacket() {
    xsens::Lock locky(&m_mutex);
    if (m_packetBuffer.empty()) return XsDataPacket();
    XsDataPacket oldestPacket(m_packetBuffer.front());
    m_packetBuffer.pop_front();
    --m_numberOfPacketsInBuffer;
    return oldestPacket;
}

void CallbackHandler::pushPacket(const XsDataPacket* packet, const char* tag) {
    if (packet == 0) return;

    xsens::Lock locky(&m_mutex);

    while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer && !m_packetBuffer.empty()) {
        m_packetBuffer.pop_front();
        --m_numberOfPacketsInBuffer;
    }

    m_packetBuffer.push_back(*packet);
    ++m_numberOfPacketsInBuffer;

    ++m_rxPackets;
    const bool hasOri = packet->containsOrientation();
    if (hasOri)                                    ++m_rxWithOri;
    if (packet->containsCalibratedGyroscopeData()) ++m_rxWithGyr;

    if (hasOri && !m_announcedOri) {
        m_announcedOri = true;
        std::printf("\n  [xs] *** orientation arrived via %s callback ***\n", tag);
        std::fflush(stdout);
    }

    // 콜백 종류별로 처음 2개씩만 덤프
    long* dumpCount = (tag[0] == 'L') ? &m_dumpLive : &m_dumpProc;
    if (*dumpCount < 2) {
        ++(*dumpCount);
        std::printf("  [%s#%ld] ori=%d gyr=%d acc=%d mag=%d\n",
                    tag, *dumpCount,
                    (int)packet->containsOrientation(),
                    (int)packet->containsCalibratedGyroscopeData(),
                    (int)packet->containsCalibratedAcceleration(),
                    (int)packet->containsCalibratedMagneticField());

        XsMessage msg = packet->toMessage();
        const XsSize n = msg.getDataSize();
        std::printf("  [%s#%ld] payload %d bytes:\n    ", tag, *dumpCount, (int)n);
        for (XsSize i = 0; i < n && i < 96; ++i) {
            std::printf("%02X ", (unsigned)msg.getDataByte(i));
            if ((i % 24) == 23) std::printf("\n    ");
        }
        std::printf("\n");
        std::fflush(stdout);
    }
}

void CallbackHandler::onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) {
    pushPacket(packet, "LIVE");
}

void CallbackHandler::onDataAvailable(XsDevice*, const XsDataPacket* packet) {
    pushPacket(packet, "PROC");
}

void CallbackHandler::getStats(long& rx, long& withOri, long& withGyr) const {
    xsens::Lock locky(&m_mutex);
    rx = m_rxPackets;
    withOri = m_rxWithOri;
    withGyr = m_rxWithGyr;
}

bool CallbackHandler::getEulerAngles(float* rpy) {
    xsens::Lock locky(&m_mutex);
    bool have = false;
    while (!m_packetBuffer.empty()) {
        XsDataPacket p = m_packetBuffer.front();
        m_packetBuffer.pop_front();
        --m_numberOfPacketsInBuffer;
        if (p.containsOrientation()) {
            XsEuler e = p.orientationEuler();
            rpy[0] = static_cast<float>(e.roll());
            rpy[1] = static_cast<float>(e.pitch());
            rpy[2] = static_cast<float>(e.yaw());
            have = true;
        }
    }
    return have;
}

bool CallbackHandler::getBodyRate(float* pqr) {
    xsens::Lock locky(&m_mutex);
    bool have = false;
    while (!m_packetBuffer.empty()) {
        XsDataPacket p = m_packetBuffer.front();
        m_packetBuffer.pop_front();
        --m_numberOfPacketsInBuffer;
        if (p.containsCalibratedGyroscopeData()) {
            XsVector r = p.calibratedGyroscopeData();
            pqr[0] = static_cast<float>(r[0]);
            pqr[1] = static_cast<float>(r[1]);
            pqr[2] = static_cast<float>(r[2]);
            have = true;
        }
    }
    return have;
}

bool CallbackHandler::getStates(float* rpy, float* pqr) {
    xsens::Lock locky(&m_mutex);

    bool haveOri = false;
    bool haveGyr = false;

    while (!m_packetBuffer.empty()) {
        XsDataPacket p = m_packetBuffer.front();
        m_packetBuffer.pop_front();
        --m_numberOfPacketsInBuffer;

        if (p.containsOrientation()) {
            XsEuler e = p.orientationEuler();
            rpy[0] = static_cast<float>(e.roll());
            rpy[1] = static_cast<float>(e.pitch());
            rpy[2] = static_cast<float>(e.yaw());
            haveOri = true;
        }
        if (p.containsCalibratedGyroscopeData()) {
            XsVector r = p.calibratedGyroscopeData();
            pqr[0] = static_cast<float>(r[0]);
            pqr[1] = static_cast<float>(r[1]);
            pqr[2] = static_cast<float>(r[2]);
            haveGyr = true;
        }
    }

    return haveOri && haveGyr;
}

bool setupXsens(XsControl*& control, XsDevice*& device, CallbackHandler& callback) {
    control = XsControl::construct();
    if (!control) { std::printf("  [xs] XsControl::construct failed\n"); return false; }

    std::printf("  [xs] scanning ports ...\n"); std::fflush(stdout);
    XsPortInfoArray portInfoArray = XsScanner::scanPorts();
    std::printf("  [xs] %d port(s) found\n", (int)portInfoArray.size());

    XsPortInfo mtPort;
    for (auto const& portInfo : portInfoArray) {
        std::printf("  [xs]   %s  id=%s\n",
                    portInfo.portName().toStdString().c_str(),
                    portInfo.deviceId().toString().toStdString().c_str());
        if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig()) {
            mtPort = portInfo;
            break;
        }
    }
    if (mtPort.empty()) { std::printf("  [xs] no MTi found\n"); return false; }

    std::printf("  [xs] opening %s @ %d baud\n",
                mtPort.portName().toStdString().c_str(), (int)mtPort.baudrate());
    std::fflush(stdout);

    if (!control->openPort(mtPort.portName().toStdString(), mtPort.baudrate())) {
        std::printf("  [xs] openPort failed\n");
        return false;
    }

    device = control->device(mtPort.deviceId());
    if (!device) { std::printf("  [xs] control->device() null\n"); return false; }

    device->addCallbackHandler(&callback);

    if (!device->gotoConfig()) { std::printf("  [xs] gotoConfig failed\n"); return false; }
    device->readEmtsAndDeviceConfiguration();

    std::printf("  [xs] product code : %s\n", device->productCode().toStdString().c_str());
    std::printf("  [xs] device id    : %s\n", device->deviceId().toString().toStdString().c_str());

    // ---------- 온보드 필터 프로파일 ----------
    {
        XsFilterProfileArray profiles = device->availableOnboardFilterProfiles();
        std::printf("  [xs] available filter profiles (%d):\n", (int)profiles.size());
        for (XsSize i = 0; i < profiles.size(); ++i) {
            std::printf("  [xs]   type=%d label=%s\n",
                        (int)profiles[i].type(),
                        profiles[i].label().toStdString().c_str());
        }
        std::printf("  [xs] current profile: %s\n",
                    device->onboardFilterProfile().label().toStdString().c_str());

        if (device->setOnboardFilterProfile(XsString("General")))
            std::printf("  [xs] filter profile -> General : OK\n");
        else
            std::printf("  [xs] filter profile -> General : FAILED\n");
    }
    std::fflush(stdout);

    XsOutputConfigurationArray configArray;
    configArray.push_back(XsOutputConfiguration(XDI_PacketCounter,  0));
    configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));
    configArray.push_back(XsOutputConfiguration(XDI_Quaternion,     100));
    configArray.push_back(XsOutputConfiguration(XDI_EulerAngles,    100));
    configArray.push_back(XsOutputConfiguration(XDI_Acceleration,   100));
    configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn,     100));
    configArray.push_back(XsOutputConfiguration(XDI_MagneticField,  100));

    if (!device->setOutputConfiguration(configArray)) {
        std::printf("  [xs] setOutputConfiguration failed\n");
        return false;
    }

    XsOutputConfigurationArray applied = device->outputConfiguration();
    std::printf("  [xs] applied output configuration (%d items):\n", (int)applied.size());
    for (XsSize i = 0; i < applied.size(); ++i) {
        std::printf("  [xs]   0x%04X @ %d Hz\n",
                    (unsigned)applied[i].m_dataIdentifier,
                    (int)applied[i].m_frequency);
    }

    // 장비가 자세를 안 내면 XDA(호스트)가 계산하도록. 결과는 onDataAvailable 로 온다.
    device->setOptions(XsOption(XSO_Calibrate | XSO_Orientation), XsOption(0));
    std::fflush(stdout);

    if (!device->gotoMeasurement()) { std::printf("  [xs] gotoMeasurement failed\n"); return false; }

    std::printf("  [xs] measurement started\n"); std::fflush(stdout);
    return true;
}

void shutdownXsens(XsControl* control, XsDevice* device) {
    if (device) {
        device->stopRecording();
        device->gotoConfig();
        if (control) control->closePort(device->portInfo().portName().toStdString());
    }
    if (control) control->destruct();
}