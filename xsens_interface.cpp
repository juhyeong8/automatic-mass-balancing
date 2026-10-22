// ===== xsens_interface.cpp =====
#include "xsens_interface.hpp"
#include <iostream>
#include <cassert>

Journaller* gJournal = 0;

CallbackHandler::CallbackHandler(size_t maxBufferSize)
    : m_maxNumberOfPacketsInBuffer(maxBufferSize), m_numberOfPacketsInBuffer(0)
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
    m_packetBuffer.push_back(*packet);
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
    if (!packetAvailable()) return false;
    XsDataPacket packet = getNextPacket();
    if (packet.containsOrientation()) {
        XsEuler euler = packet.orientationEuler();
        rpy[0] = static_cast<float>(euler.roll());
        rpy[1] = static_cast<float>(euler.pitch());
        rpy[2] = static_cast<float>(euler.yaw());
    } else {
        return false;
    }
    if (packet.containsCalibratedGyroscopeData()) {
        XsVector rate = packet.calibratedGyroscopeData();
        pqr[0] = static_cast<float>(rate[0]);
        pqr[1] = static_cast<float>(rate[1]);
        pqr[2] = static_cast<float>(rate[2]);
    } else {
        return false;
    }
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
    configArray.push_back(XsOutputConfiguration(XDI_EulerAngles, 100));
    configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 100));

    if (!device->setOutputConfiguration(configArray)) return false;

    if (!device->gotoMeasurement()) return false;

    return true;
}

void shutdownXsens(XsControl* control, XsDevice* device) {
    if (device) device->stopRecording();
    if (control) control->closePort(device->portInfo().portName().toStdString());
    if (control) control->destruct();
}
