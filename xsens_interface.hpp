// ===== xsens_interface.hpp =====
#pragma once
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xstime.h>
#include <xscommon/xsens_mutex.h>
#include <list>
#include <cstdint>

struct XsensReceiveStats {
    uint64_t packets = 0;
    uint64_t orientation = 0;
    uint64_t gyro = 0;
    uint64_t complete = 0;
    uint64_t dropped = 0;
    uint64_t errors = 0;
    int lastError = 0;
};

class CallbackHandler : public XsCallback
{
public:
    CallbackHandler(size_t maxBufferSize = 5);
    virtual ~CallbackHandler();
    bool packetAvailable() const;
    XsDataPacket getNextPacket();
    bool getEulerAngles(float* rpy);
    bool getBodyRate(float* pqr);
    bool getStates(float* rpy, float* pqr);
    XsensReceiveStats receiveStats() const;

protected:
    void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) override;
    void onError(XsDevice*, XsResultValue error) override;

private:
    mutable xsens::Mutex m_mutex;
    XsensReceiveStats m_stats;
    size_t m_maxNumberOfPacketsInBuffer;
    size_t m_numberOfPacketsInBuffer;
    std::list<XsDataPacket> m_packetBuffer;
};

bool setupXsens(XsControl*& control, XsDevice*& device, CallbackHandler& callback);
void shutdownXsens(XsControl* control, XsDevice* device);