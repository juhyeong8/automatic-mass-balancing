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

protected:
    void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) override;

private:
    mutable xsens::Mutex m_mutex;
    size_t m_maxNumberOfPacketsInBuffer;
    size_t m_numberOfPacketsInBuffer;
    std::list<XsDataPacket> m_packetBuffer;
};

bool setupXsens(XsControl*& control, XsDevice*& device, CallbackHandler& callback);
void shutdownXsens(XsControl* control, XsDevice* device);