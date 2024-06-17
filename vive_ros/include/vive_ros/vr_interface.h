#ifndef _VR_INTERFACE_H_
#define _VR_INTERFACE_H_

#include <openvr.h>
#include <boost/function.hpp>


typedef boost::function<void(const std::string&)> DebugMsgCallback;
typedef boost::function<void(const std::string&)> InfoMsgCallback;
typedef boost::function<void(const std::string&)> ErrorMsgCallback;


const int BUTTON_NUM = 4;
const int AXES_NUM = 3;

class VRInterface
{
    public:
        VRInterface();
        ~VRInterface();

        bool init();
        void shutdown();

        void update();
        void updateCalibration();

        void handleInput(vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t& state);
        void triggerHapticPulse(vr::TrackedDeviceIndex_t unControllerDeviceIndex, uint32_t unAxisId, int usDurationMicroSec);

        int getDeviceMatrix(int index, double pMat[3][4]);
        int getDeviceVel(int index, double lin_vel[3], double ang_vel[3]);
        bool isDeviceConnected(int index);

        void setErrorMsgCallback(ErrorMsgCallback fn);
        void setInfoMsgCallback(InfoMsgCallback fn);
        void setDebugMsgCallback(DebugMsgCallback fn);

        vr::IVRSystem *pHMD_;
        vr::TrackedDevicePose_t device_poses_[vr::k_unMaxTrackedDeviceCount];


    private:
        vr::IVRChaperone *pChaperone_;
        uint max_devices_;

        DebugMsgCallback debug_;
        InfoMsgCallback info_;
        ErrorMsgCallback error_;

        vr::ChaperoneCalibrationState cal_state_;
        float play_area_[2];
        vr::HmdQuad_t play_quat_;

        std::string getTrackedDeviceString(vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prp, vr::TrackedPropertyError *prpErr = NULL);

};

    


#endif 