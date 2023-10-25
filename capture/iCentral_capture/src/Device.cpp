#include "Device.h"

#include <limits>

VIO::capture::DeviceGenIApi::DeviceGenIApi(
    const std::tuple<std::string, std::string, unsigned int>& descriptor):
    _descriptor(descriptor),
    _devHandle(nullptr) {}

VIO::capture::DeviceGenIApi::~DeviceGenIApi() {
    close();
}

bool VIO::capture::DeviceGenIApi::open() {
    if(!_devHandle) {
        auto cameraIndex = std::get<2>(_descriptor);
        auto ret = IMV_CreateHandle(&_devHandle, _IMV_ECreateHandleMode::modeByIndex, (void*)&cameraIndex);
        if (IMV_OK != ret)
		{
			_devHandle = nullptr;
		}
        // Open camera
		ret = IMV_Open(_devHandle);
        bool camReady = false;
		if (IMV_OK == ret){
            // Set the response timeout interval of camera sends command to the API. Only for Gige device(unit：ms)
		    //ret = IMV_GIGE_SetAnswerTimeout(_devHandle, 1000);
		    if (IMV_OK == ret){
                // Set packet timeout, only for Gige device(unit：ms)
                //ret = IMV_GIGE_SetInterPacketTimeout(_devHandle, 50);
		        if (IMV_OK == ret){
			        // Set the single resend maximum packet number, only for Gige device
		            //ret = IMV_GIGE_SetSingleResendMaxPacketNum(_devHandle, 50);
		            if (IMV_OK == ret){
			    		// Set the maximum lost packet number, only for Gige device
		                //ret = IMV_GIGE_SetMaxLostPacketNum(_devHandle, 80);
		                if (IMV_OK == ret){
                            camReady = true;
		                }
                    }
                }
            }
        }
        if(!camReady) {
            close();
        }
    }
    return _devHandle;
}

bool VIO::capture::DeviceGenIApi::close() {
    stopAcquisition();
    auto ret = IMV_Close(_devHandle);
    _devHandle = nullptr;
	if (IMV_OK != ret)
	{
        return false;
	}
    return true;
 }

 const std::string& VIO::capture::DeviceGenIApi::getInterfaceId() const {
    return std::get<0>(_descriptor);
 }

 const std::string& VIO::capture::DeviceGenIApi::getId() const {
    return std::get<1>(_descriptor);
 }

bool VIO::capture::DeviceGenIApi::startAcquisition()
{
    if((_devHandle || open()) && IMV_IsOpen(_devHandle)) {
        auto ret = IMV_StartGrabbing(_devHandle);
        return ret == IMV_OK;
    }
    return false;
}

/*bool VIO::capture::Device::startAcquisition(
    const std::vector<std::shared_ptr<unsigned char>>& pool,
    size_t size) {
    return false;
}*/

void VIO::capture::DeviceGenIApi::stopAcquisition() {
    if(_devHandle && IMV_IsGrabbing(_devHandle)) {
        IMV_StopGrabbing(_devHandle);
    }
}

bool VIO::capture::DeviceGenIApi::nextFrame(FrameCallback& callback) {
    return nextFrame(callback, std::numeric_limits<unsigned int>::max());
}

bool VIO::capture::DeviceGenIApi::nextFrame(FrameCallback& callback,
                                     unsigned int timeout) {
    if(_devHandle && IMV_IsGrabbing(_devHandle)) {
        IMV_Frame frame;
        //FrameHeader header;
        auto ret = IMV_GetFrame (_devHandle, &frame, timeout);
        if(ret != IMV_OK) {
            return false;
        }
        frame.frameInfo.timeStamp <<= 3;
        callback(frame.pData, &frame.frameInfo);
        ret = IMV_ReleaseFrame(_devHandle, &frame);
        return ret == IMV_OK;
    }
    return false;
}
