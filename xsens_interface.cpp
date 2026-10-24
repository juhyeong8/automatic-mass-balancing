// ===== xsens_interface.cpp =====
#include "xsens_interface.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

Journaller* gJournal = 0;

namespace {
constexpr int sensorOutputRateHz = 100;
constexpr float gyroCutoffHz = 3.0f;
// Use the configured sensor period, not the USB callback arrival interval.
const float gyroLpfAlpha = 1.0f - std::exp(
    -2.0f * 3.14159265359f * gyroCutoffHz / sensorOutputRateHz);
}

CallbackHandler::CallbackHandler(size_t maxBufferSize, bool filterGyroscope)
    : m_maxNumberOfPacketsInBuffer(maxBufferSize), m_numberOfPacketsInBuffer(0),
      m_filterGyroscope(filterGyroscope), m_gyroLpfInitialized(false),
      m_filteredGyro{{0.0f, 0.0f, 0.0f}}
{
}

CallbackHandler::~CallbackHandler() {}

bool CallbackHandler::packetAvailable() const {
    xsens::Lock locky(&m_mutex);
    return m_numberOfPacketsInBuffer > 0;
}

XsDataPacket CallbackHandler::getNextPacket() {
    assert(packetAvailable());
    xsens::Lock locky(&m_mutex);
    XsDataPacket oldestPacket(m_packetBuffer.front());
    m_packetBuffer.pop_front();
    --m_numberOfPacketsInBuffer;
    return oldestPacket;
}

void CallbackHandler::onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) {
    xsens::Lock locky(&m_mutex);
    assert(packet != 0);
    while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
        (void)getNextPacket();
    XsDataPacket filteredPacket(*packet);
    if (m_filterGyroscope && packet->containsCalibratedGyroscopeData()) {
        XsVector rate = packet->calibratedGyroscopeData();
        for (size_t axis = 0; axis < m_filteredGyro.size(); ++axis) {
            const float input = static_cast<float>(rate[axis]);
            if (!m_gyroLpfInitialized) m_filteredGyro[axis] = input;
            else m_filteredGyro[axis] += gyroLpfAlpha * (input - m_filteredGyro[axis]);
            rate[axis] = m_filteredGyro[axis];
        }
        m_gyroLpfInitialized = true;
        filteredPacket.setCalibratedGyroscopeData(rate);
    }
    // Euler stays unchanged; the stored gyro includes every received sample.
    m_packetBuffer.push_back(filteredPacket);
    ++m_numberOfPacketsInBuffer;
    assert(m_numberOfPacketsInBuffer <= m_maxNumberOfPacketsInBuffer);
}

bool CallbackHandler::getEulerAngles(float* rpy) {
    if (!packetAvailable()) return false;
    XsDataPacket packet = getNextPacket();
    if (!packet.containsOrientation()) return false;
    XsEuler euler = packet.orientationEuler();
    rpy[0] = static_cast<float>(euler.roll());
    rpy[1] = static_cast<float>(euler.pitch());
    rpy[2] = static_cast<float>(euler.yaw());
    return true;
}

bool CallbackHandler::getBodyRate(float* pqr) {
    if (!packetAvailable()) return false;
    XsDataPacket packet = getNextPacket();
    if (!packet.containsCalibratedGyroscopeData()) return false;
    XsVector rate = packet.calibratedGyroscopeData();
    pqr[0] = static_cast<float>(rate[0]);
    pqr[1] = static_cast<float>(rate[1]);
    pqr[2] = static_cast<float>(rate[2]);
    return true;
}

bool CallbackHandler::getStates(float* rpy, float* pqr) {
    XsDataPacket packet;
    {
        xsens::Lock locky(&m_mutex);
        if (m_packetBuffer.empty()) return false;
        // Select and clear under one lock so a newly arriving packet is not lost.
        packet = m_packetBuffer.back();
        m_packetBuffer.clear();
        m_numberOfPacketsInBuffer = 0;
    }
    // Keep both output arrays unchanged if this packet is incomplete.
    if (!packet.containsOrientation() || !packet.containsCalibratedGyroscopeData())
        return false;
    XsEuler euler = packet.orientationEuler();
    XsVector rate = packet.calibratedGyroscopeData();
    rpy[0] = static_cast<float>(euler.roll());
    rpy[1] = static_cast<float>(euler.pitch());
    rpy[2] = static_cast<float>(euler.yaw());
    pqr[0] = static_cast<float>(rate[0]);
    pqr[1] = static_cast<float>(rate[1]);
    pqr[2] = static_cast<float>(rate[2]);
    return true;
}

bool setupXsens(XsControl*& control, XsDevice*& device, CallbackHandler& callback) {
    control = XsControl::construct();
    if (!control) return false;

    XsPortInfoArray portInfoArray = XsScanner::scanPorts();
    XsPortInfo mtPort;
    for (auto const& portInfo : portInfoArray) {
        if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig()) {
            mtPort = portInfo;
            break;
        }
    }
    if (mtPort.empty()) return false;

    if (!control->openPort(mtPort.portName().toStdString(), mtPort.baudrate())) return false;
    device = control->device(mtPort.deviceId());
    if (!device) return false;

    device->addCallbackHandler(&callback);

    if (!device->gotoConfig()) return false;

    device->readEmtsAndDeviceConfiguration();
    XsOutputConfigurationArray configArray;
    configArray.push_back(XsOutputConfiguration(XDI_EulerAngles, sensorOutputRateHz));
    configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, sensorOutputRateHz));

    if (!device->setOutputConfiguration(configArray)) return false;

    if (!device->gotoMeasurement()) return false;

    return true;
}

void shutdownXsens(XsControl* control, XsDevice* device) {
    if (device) device->stopRecording();
    if (control) control->closePort(device->portInfo().portName().toStdString());
    if (control) control->destruct();
}
