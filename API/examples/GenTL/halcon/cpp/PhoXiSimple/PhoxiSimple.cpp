#include <halconcpp/HalconCpp.h>

int main(int argc, char* argv[]) {
    // Local control variables
    HalconCpp::HTuple hv_AcqName;
    HalconCpp::HTuple hv_AcqHandle;
    HalconCpp::HTuple hv_DeviceInfo, hv_DeviceValues;

    HalconCpp::HTuple hv_IsPhoXiControlRunning;
    HalconCpp::HTuple hv_PhotoneoAPIVersion;
    HalconCpp::HTuple hv_PhotoneoDeviceID;
    HalconCpp::HTuple hv_PhotoneoDeviceType;
    HalconCpp::HTuple hv_IsAcquiring;
    HalconCpp::HTuple hv_IsConnected;
    HalconCpp::HTuple hv_PhotoneoDeviceFirmwareVersion;
    HalconCpp::HTuple hv_PhotoneoDeviceVariant;
    HalconCpp::HTuple hv_PhotoneoDeviceFeatures;

    // Local iconic variables
    HalconCpp::HObject ho_ImageData;
    HalconCpp::HObject ho_TextureImage;
    HalconCpp::HObject ho_Texture, ho_PointCloud, ho_Normals, ho_DepthMap, ho_ConfidenceMap;
    HalconCpp::HObject ho_Region, ho_Contours;

    HalconCpp::HTuple hv_Data;

    try {
        hv_AcqName = "GenICamTL";

        // PhoXiControl must be running at this point
        InfoFramegrabber(hv_AcqName, "device", &hv_DeviceInfo, &hv_DeviceValues);

        // Connect to scanner via index in hv_DeviceValues[idx] or "PhotoneoTL_DEV_<device_id>"
        HalconCpp::HTuple hv_device_id = "hv_DeviceValues[0]";
        //HalconCpp::HTuple hv_device_id = "PhotoneoTL_DEV_ABC-000";

        OpenFramegrabber("GenICamTL", 0, 0, 0, 0, 0, 0, "progressive", -1, "default", -1,
            "false", "default", hv_device_id, 0, -1, &hv_AcqHandle);

        GetFramegrabberParam(hv_AcqHandle, "IsPhoXiControlRunning", &hv_IsPhoXiControlRunning);
        GetFramegrabberParam(hv_AcqHandle, "PhotoneoAPIVersion", &hv_PhotoneoAPIVersion);
        GetFramegrabberParam(hv_AcqHandle, "PhotoneoDeviceID", &hv_PhotoneoDeviceID);
        GetFramegrabberParam(hv_AcqHandle, "PhotoneoDeviceType", &hv_PhotoneoDeviceType);
        GetFramegrabberParam(hv_AcqHandle, "IsAcquiring", &hv_IsAcquiring);
        GetFramegrabberParam(hv_AcqHandle, "IsConnected", &hv_IsConnected);
        GetFramegrabberParam(hv_AcqHandle, "PhotoneoDeviceFirmwareVersion", &hv_PhotoneoDeviceFirmwareVersion);
        GetFramegrabberParam(hv_AcqHandle, "PhotoneoDeviceVariant", &hv_PhotoneoDeviceVariant);
        GetFramegrabberParam(hv_AcqHandle, "PhotoneoDeviceFeatures", &hv_PhotoneoDeviceFeatures);

        std::cout << "IsPhoXiControlRunning:         " << hv_IsPhoXiControlRunning.I() << std::endl;
        std::cout << "PhotoneoAPIVersion:            " << hv_PhotoneoAPIVersion.S() << std::endl;
        std::cout << "PhotoneoDeviceID:              " << hv_PhotoneoDeviceID.S() << std::endl;
        std::cout << "PhotoneoDeviceType:            " << hv_PhotoneoDeviceType.S() << std::endl;
        std::cout << "IsAcquiring:                   " << hv_IsAcquiring.I() << std::endl;
        std::cout << "IsConnected:                   " << hv_IsConnected.I() << std::endl;
        std::cout << "PhotoneoDeviceFirmwareVersion: " << hv_PhotoneoDeviceFirmwareVersion.S() << std::endl;
        std::cout << "PhotoneoDeviceVariant:         " << hv_PhotoneoDeviceVariant.S() << std::endl;
        std::cout << "PhotoneoDeviceFeatures:        " << hv_PhotoneoDeviceFeatures.S() << std::endl;

        SetFramegrabberParam(hv_AcqHandle, "LogoutAfterDisconnect", true);

        SetFramegrabberParam(hv_AcqHandle, "WaitForAccept", true);
        SetFramegrabberParam(hv_AcqHandle, "WaitForGrabbingEnd", true);

        // Need to set `UseFixedDataOrdering` to false in order for HALCON to receive multipart data.
        // Our devices can send up to 8 different output data which can be enabled / disabled by the user.
        // In case of disabled - empty data are sent, but HALCON interprets it as invalid data.
        SetFramegrabberParam(hv_AcqHandle, "UseFixedDataOrdering", false);

        // Order of transfered images is fixed.
        // By disabling some items, the order for `SelectObj` is altered.
        // Note: EventMap is a MotionCam - 3D feature only
        // Note: ColorCameraImage is a MotionCam - 3D Color feature only
        SetFramegrabberParam(hv_AcqHandle, "SendTexture", true);
        SetFramegrabberParam(hv_AcqHandle, "SendPointCloud", true);
        SetFramegrabberParam(hv_AcqHandle, "SendNormalMap", false);
        SetFramegrabberParam(hv_AcqHandle, "SendDepthMap", true);
        SetFramegrabberParam(hv_AcqHandle, "SendConfidenceMap", false);
        SetFramegrabberParam(hv_AcqHandle, "SendEventMap", false);
        SetFramegrabberParam(hv_AcqHandle, "SendColorCameraImage", false);

        // Software trigger
        // ----------------
        SetFramegrabberParam(hv_AcqHandle, "PhotoneoTriggerMode", "Software");

        // Trigger frame by calling property's setter.
        // Must call TriggerFrame before every fetch.

        for (int i = 0; i <= 5; i++) {
            // Trigger first frame
            SetFramegrabberParam(hv_AcqHandle, "TriggerFrame", -1);
            // Grab first frame (image only)
            GrabImage(&ho_TextureImage, hv_AcqHandle);

            // Trigger second frame
            SetFramegrabberParam(hv_AcqHandle, "TriggerFrame", -1);
            // Grab second frame (image only)
            GrabImage(&ho_TextureImage, hv_AcqHandle);

            // Trigger third frame
            SetFramegrabberParam(hv_AcqHandle, "TriggerFrame", -1);
            // Grab third frame (including 3D data)
            GrabData(&ho_ImageData, &ho_Region, &ho_Contours, hv_AcqHandle, &hv_Data);

            // Do something with data
            // Order of objects must be preserved
            SelectObj(ho_ImageData, &ho_Texture, 1);
            SelectObj(ho_ImageData, &ho_PointCloud, 2);
            SelectObj(ho_ImageData, &ho_DepthMap, 3);
        }

        // Freerun
        // -------
        SetFramegrabberParam(hv_AcqHandle, "PhotoneoTriggerMode", "Freerun");

        // Note: Acquisition is stopped and started when calling `grab_image/grab_data`.

        for (int i = 0; i <= 5; i++) {
            // Grab first frame (image only)
            GrabImage(&ho_TextureImage, hv_AcqHandle);

            // Grab second frame (image only)
            GrabImage(&ho_TextureImage, hv_AcqHandle);

            // Grab third frame (including 3D data)
            GrabData(&ho_ImageData, &ho_Region, &ho_Contours, hv_AcqHandle, &hv_Data);

            // Do something with data
            // Order of objects must be preserved
            SelectObj(ho_ImageData, &ho_Texture, 1);
            SelectObj(ho_ImageData, &ho_PointCloud, 2);
            SelectObj(ho_ImageData, &ho_DepthMap, 3);

        }
    } catch (HalconCpp::HException &ex) {
        std::cerr << ex.ErrorMessage() << std::endl;
    }

    CloseFramegrabber(hv_AcqHandle);

    return 0;
}
