/**
 @file      voxel3d.h
 @brief     libvoxel3d APIs for 5Voxel 5VHiRab device
 @author    Jackie Lee
 @copyright Copyright (c) 2025 5Voxel Co., Ltd.
*/

#ifndef __VOXEL3d_H__
#define __VOXEL3d_H__

#ifdef PLAT_WINDOWS
#pragma once

#ifdef LIBVOXEL3D_EXPORTS
#define VOXEL3D_API_DLL __declspec(dllexport)
#elif LIBVOXEL3D_STATIC
#define VOXEL3D_API_DLL
#else
#define VOXEL3D_API_DLL __declspec(dllimport)
#endif

#else /* PLAT_LINUX */
#define VOXEL3D_API_DLL
#endif /* PLAT_WINDOWS */

#define TOF_CAM_VID                   "3558"
#define TOF_CAM_PID                   "1006"

#define TOF_CAM_DEV_NAME              "HE-2 ToF Decoder"
#define RGB_CAM_DEV_NAME              "HE-2 RGB Decoder"
#define FLIR_CAM_DEV_NAME             "FLIR"

#define TOF_DEPTH_WIDTH               (640)
#define TOF_DEPTH_HEIGHT              (480)
#define TOF_DEPTH_PIXELS              (TOF_DEPTH_WIDTH * TOF_DEPTH_HEIGHT)
#define TOF_IR_WIDTH                  TOF_DEPTH_WIDTH
#define TOF_IR_HEIGHT                 TOF_DEPTH_HEIGHT
#define TOF_IR_PIXELS                 TOF_DEPTH_PIXELS
#define TOF_DEPTH_ONLY_FRAME_SIZE     (TOF_DEPTH_PIXELS * sizeof(unsigned short))
#define TOF_IR_ONLY_FRAME_SIZE        TOF_DEPTH_ONLY_FRAME_SIZE
#define TOF_DEPTH_IR_FRAME_SIZE       (TOF_DEPTH_ONLY_FRAME_SIZE + TOF_IR_ONLY_FRAME_SIZE)

#define MAX_SUPPORTED_CAMERA_MODULE   (12)
#define MAX_PRODUCT_NAME_LEN          (128)
#define MAX_PRODUCT_SN_LEN            (128)
#define MAX_DEV_NAME_LEN              (128)

#define MAX_FW_VER_LEN                (16)
#define MAX_FW_BUILD_DATE_LEN         (16)


#define FLIR_WIDTH                    (160)
#define FLIR_HEIGHT                   (120)
#define FLIR_PIXELS                   (FLIR_WIDTH * FLIR_HEIGHT)
#define FLIR_FRAME_SIZE               (FLIR_PIXELS * sizeof(float))


#define RGB_WIDTH                     (1920)
#define RGB_HEIGHT                    (1080)
#define RGB_PIXELS                    (RGB_WIDTH * RGB_HEIGHT)

/**
 * @brief  Structure used in voxel3d_tof_read_camera_info() to read out camera info from device
 */
struct CameraInfo {
    float focalLengthFx;
    float focalLengthFy;
    float principalPointCx;
    float principalPointCy;
    float K1;
    float K2;
    float P1;
    float P2;
    float K3;
    float K4;
    float K5;
    float K6;
};

struct CamResolution {
    unsigned int width;
    unsigned int height;
};

/**
 * @brief  Structure used in voxel3d_tof_scan() to fill up with scanned device number
 *         and product serial number for each device
 */
struct VOXEL3D_API_DLL CamDevInfo {
    int num_of_devices = 0;
    char product_sn[MAX_SUPPORTED_CAMERA_MODULE][MAX_PRODUCT_SN_LEN] = {};
    char dev_name[MAX_SUPPORTED_CAMERA_MODULE][MAX_DEV_NAME_LEN] = {};
    CamResolution resolution[MAX_SUPPORTED_CAMERA_MODULE] = {};
};

/**
 * @brief  Structure used in voxel3d_set_rectifyType() to set alignment between depth and other devices
 */
enum RectifyType
{
    NONE = 0,
    RGB2TOF = 1,
    FLIR2TOF = 2,
};

typedef struct
{
    unsigned int imu_ts;
    float imu_accel[3];
    float imu_gyro[3];
} IMU_DATA;

/**
 * @brief       Perform the scan of 5Voxel 5VHiRab devices
 * @param[out]  CamDevInfo: structure to store the scanned result
 * @return      > 0: number of device(s) found
 * @return      others: can't find device
 */
extern "C" VOXEL3D_API_DLL int voxel3d_scan(CamDevInfo *cam_dev_info);


/**
 * @brief       Perform the initialization of ToF on sepcific 5Voxel 5VHiRab device
 * @warning     This function has to be called before voxel3d_tof_queryframe(),
 *              otherwise the query will fail
 * @param[in]   dev_sn  device S/N, which can be read from the label on 5VHiRab device
 *                      or from result of voxel3d_scan(). Input S/N with NULL pointer
 *                      or empty string will initialize the 1st scanned device
 * @return      true    found device and init successfully
 * @return      < 0     can't find device or data error
 */
extern "C" VOXEL3D_API_DLL int voxel3d_tof_init(char *dev_sn);


/**
 * @brief       Grab a depth & ir frame from 5Voxel 5VHiRab camera
 * @warning     Call voxel3d_tof_init() to initialize specific device before query
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  depthmap: pointer of user-allocated buffer for Depth frame storage
 *                        Buffer size shall be TOF_DEPTH_ONLY_FRAME_SIZE (in bytes)
 * @param[out]  irmap: pointer of user-allocated buffer for IR frame storage
 *                     Buffer size shall be TOF_IR_ONLY_FRAME_SIZE (in bytes)
 * @return      > 0: current frame count (1 ~ UINT_MAX)
 * @return      = 0: no new frame from device
 */
extern "C" VOXEL3D_API_DLL unsigned int voxel3d_tof_queryframe(char* dev_sn,
                                                           unsigned short *depthmap,
                                                           unsigned short *irmap);


/**
 * @brief       Generate pointcloud data based on input deptpmap and the
 *              calibration parameters from 5Voxel 5VHiRab camera
 * @details     The unit of x/y/z is meter
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[in]   depthmap: pointer of Depth frame filled by voxel3d_tof_queryframe()
 * @param[out]  xyz: pointer of user-allocated buffer for pointcloud frame storage
 * @return      > 0: pixels of pointcloud xyz filled in xyz buffer
 * @return      <= 0: failed to generate pointcloud
 */
extern "C" VOXEL3D_API_DLL int voxel3d_tof_generatePointCloud(char* dev_sn,
                                                              unsigned short *depthmap,
                                                              float *xyz);


/**
 * @brief       Release the resource allocated for ToF on 5Voxel 5VHiRab device
 * @warning     This function has to be called before program exit
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 */
extern "C" VOXEL3D_API_DLL void voxel3d_tof_release(char* dev_sn);


/**
 * @brief       Perform the initialization of Lepton camera on 5Voxel 5VHiRab device
 * @note        Support 160x120 FLIR Leption 3.5
 * @warning     This function has to be called before voxel3d_lepton3_queryframe(),
 *              otherwise the query will fail
 * @param[in]   dev_sn  device S/N, which can be read from the label on 5VHiRab device
 *                      or from result of voxel3d_scan(). Input S/N with NULL pointer
 *                      or empty string will initialize the 1st scanned device
 * @return      true    found device and init successfully
 * @return      < 0     can't find device or data error
 */
extern "C" VOXEL3D_API_DLL int voxel3d_lepton3_init(char* dev_sn);


/**
 * @brief       Grab a thermal image frame
 * @note        Support 160x120 FLIR Leption 3.5
 * @warning     Call voxel3d_lepton3_init() to initialize specific device before query
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  thermal_img: pointer of user-allocated buffer for thermal image storage
 *                           data type: unsigned char
 *                           buffer size: RectifyType != FLIR2TOF, FLIR_WIDTH X FLIR_HEIGHT
 *                                        RectifyType = FLIR2TOF, TOF_DEPTH_WIDTH X TOF_DEPTH_HEIGHT
 * @return      > 0: current frame count (1 ~ UINT_MAX)
 * @return      = 0: no new frame from device
 */
extern "C" VOXEL3D_API_DLL unsigned int voxel3d_lepton3_queryframe(char* dev_sn,
                                                                   float* thremal_map);


/**
 * @brief       Release the resource allocated for Lepton camera on 5Voxel 5VHiRab device
 * @warning     This function has to be called before program exit
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 */
extern "C" VOXEL3D_API_DLL void voxel3d_lepton3_release(char* dev_sn);


/**
 * @brief       Perform the initialization of RGB on 5Voxel 5VHiRab device
 * @warning     This function has to be called before voxel3d_rgb_queryframe(),
 *              otherwise the query will fail
 * @param[in]   dev_sn  device S/N, which can be read from the label on 5VHiRab device
 *                      or from result of voxel3d_scan(). Input S/N with NULL pointer
 *                      or empty string will initialize the 1st scanned device
 * @return      true    found device and init successfully
 * @return      < 0     can't find device or data error
 */
extern "C" VOXEL3D_API_DLL int voxel3d_rgb_init(char* dev_sn);


/**
 * @brief       Grab a rgb frame from 5Voxel 5VHiRab device
 * @warning     Call voxel3d_rgb_init() to initialize specific device before query
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  rgb_map: pointer of user-allocated buffer for Depth frame storage
 *                       data type: unsigned char
 *                       buffer size: RectifyType != RGB2TOF, RGB_WIDTH X RGB_HEIGHT
 *                                    RectifyType = RGB2TOF, TOF_DEPTH_WIDTH X TOF_DEPTH_HEIGHT
 * @return      > 0: current frame count (1 ~ UINT_MAX)
 * @return      = 0: no new frame from device
 */
extern "C" VOXEL3D_API_DLL unsigned int voxel3d_rgb_queryframe(char* dev_sn, 
                                                               unsigned char* rgb_map);


/**
  * @brief       Release the resource allocated for RGB on 5Voxel 5VHiRab device
  * @warning     This function has to be called before program exit
  * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
  *                      initialize the 1st scanned device
  */
extern "C" VOXEL3D_API_DLL void voxel3d_rgb_release(char* dev_sn);


/**
 * @brief       Release all cameras associated with the input device S/N
 * @note        User can either release single camera (tof, rgb, lepton3) by calling
 *              its release fucntion or call this to release all in one shot.
 *              Suggest to call this if user wants to release everything before program
 *              exit.
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 */
extern "C" VOXEL3D_API_DLL void voxel3d_release(char* dev_sn);


/**
 * @brief       Grab IMU data from 5Voxel 5VHiRab device
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  imu_data: pointer of uesr-allocated buffer to store imu data
 * @return      true: buffer shall be filled with related imu data
 * @return      0: no IMU data available
 */
extern "C" VOXEL3D_API_DLL int voxel3d_read_imu_data(char* dev_sn, IMU_DATA* imu_data);


/**
 * @brief       Grab ToF camera info from 5Voxel 5VHiRab device
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  camera_info: pointer of uesr-allocated buffer to store camera info
 * @return      true: buffer shall be filled with related camera info
 * @return      < 0: failed to get camera info from library/device or error on inputa
 *                   parameter
 */
extern "C" VOXEL3D_API_DLL int voxel3d_tof_read_camera_info(char* dev_sn,
                                                            CameraInfo *cam_info);


/**
 * @brief       Get current ToF confidence threshold value from 5voxel 5VHiRab device
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @return      >= 0: confidence threshold value read from camera
 * @return      < 0: failed to get confidence threshold
 */
extern "C" VOXEL3D_API_DLL int voxel3d_tof_get_conf_threshold(char* dev_sn);


/**
 * @brief       Set run-time ToF confidence threshold to 5voxel 5VHiRab device
 * @warning
 *     1. Call this function after voxel3d_tof_init() is completed and successfully,
 *        otherwise, it returns false
 *     2. Corresponding pixel depth will be zero if its confidence is lower than threshold
 *     3. Configured threshold will go back to default when camera is power-cycled
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[in]   conf_threshold: 0~4095
 * @return      true: set confidence threshold successfully
 * @return      < 0: failed to set confidence threshold
 */
extern "C" VOXEL3D_API_DLL int voxel3d_tof_set_conf_threshold(char* dev_sn,
                                                              unsigned int conf_threshold);


/**
 * @brief       Set ToF auto exposure mode to 5voxel 5VHiRab device
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              therwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[in]   enable: 0 -> disable auto exposure, others -> enable auto exposure
 * @return      true: set auto exposure mode successfully
 * @return      < 0: failed to set auto exposure mode
 */
extern "C" VOXEL3D_API_DLL int voxel3d_tof_set_auto_exposure_mode(char* dev_sn,
                                                                  unsigned int enable);


/**
 * @brief       Get ToF HFoV from 5voxel 5VHiRab device
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @return      > 0: calculated ToF HFoV
 * @return      <=0: failed to get ToF HFoV
 */
extern "C" VOXEL3D_API_DLL float voxel3d_tof_get_depth_hfov(char* dev_sn);


/**
 * @brief       Get ToF VFoV from 5voxel 5VHiRab device
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @return      > 0: calculated ToF VFoV
 * @return      <=0: failed to get ToF VFoV
 */
extern "C" VOXEL3D_API_DLL float voxel3d_tof_get_depth_vfov(char* dev_sn);


/**
 * @brief       Grab thermal info from 5voxel 5VHiRab device
 * @warning     Call this function after voxel3d_lepton3_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  camera_params: pointer of uesr-allocated buffer to store camera info
 * @return      true: buffer shall be filled with related camera info
 * @return      < 0: failed to get camera info from library/device or error on inputa
 *                   parameter
 */
extern "C" VOXEL3D_API_DLL int voxel3d_lepton3_read_camera_info(char* dev_sn,
                                                                CameraInfo* cam_info);


/**
 * @brief       Get rgb camera intrinsic & distortion information from 5voxel 5VHiRab device
 * @warning     This function has to be called after voxel3d_rgb_init()
 * @param[in]   dev_sn  device S/N, which can be read from the label on 5VHiRab device
 *                      or from result of voxel3d_scan(). Input S/N with NULL pointer
 *                      or empty string will initialize the 1st scanned device
 * @param[out]  cam_info camera intrinsic & distortion information
 * @return      true:  get camera information successfully
 * @return      false: failed to read camera information from device
 */
extern "C" VOXEL3D_API_DLL int voxel3d_rgb_read_camera_info(char* dev_sn,
                                                            CameraInfo * cam_info);


/**
 * @brief       Set 5voxel 5VHiRab device rectified mode
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[in]   inputType: 0 -> NONE, 
 *                         1 -> RGB - TOF (RGB output will become 640x480, aligned with ToF)
 *                         2 -> THERMAL(FLIR) - TOF (FLIR output will become 640x480, aligned with ToF)
 * @return      true:  set device alignment type successfully
 * @return      false: failed to set rectified mode
 */
extern "C" VOXEL3D_API_DLL int voxel3d_set_rectifyType(char* dev_sn, int inputType);


/**
 * @brief       Read out camera F/W version
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  fw_ver: pointer of user-allocated buffer to store fw version string
 * @param[in]   max_len: length of user-allocated buffer
 * @return      true: buffer shall be filled with F/W version string
 * @return      < 0: failed to get F/W version from device or error on input parameters
 */
extern "C" VOXEL3D_API_DLL int voxel3d_read_fw_version(char* dev_sn,
                                                       char* fw_ver,
                                                       unsigned int max_len);


/**
 * @brief       Read out camera F/W build date
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  fw_build_date: pointer of user-allocated buffer to store fw build date
 *                    string
 * @param[in]   max_len: length of user-allocated buffer
 * @return      true: buffer shall be filled with F/W build date string
 * @return      < 0: failed to get F/W build date from device or error on input parameters
 */
extern "C" VOXEL3D_API_DLL int voxel3d_read_fw_build_date(char* dev_sn,
                                                          char* fw_build_date,
                                                          unsigned int max_len);


/**
 * @brief       Read out library version
 * @param[out]  lib_version: pointer of user-allocated buffer to store library version
 *                           string
 * @param[in]   max_len: length of user-allocated buffer
 * @return      true: buffer shall be filled with library version string
 * @return      false: failed to get library version
 */
extern "C" VOXEL3D_API_DLL int voxel3d_read_lib_version(char* lib_version,
                                                        int max_len);


/**
 * @brief       Read out library build date
 * @param[out]  lib_build_date: pointer of user-allocated buffer to store library build date
 *                              string
 * @param[in]   max_len: length of user-allocated buffer
 * @return      true: buffer shall be filled with library build date string
 * @return      false: failed to get library build date
 */
extern "C" VOXEL3D_API_DLL int voxel3d_read_lib_build_date(char* lib_build_date,
                                                           int max_len);


/**
 * @brief       5voxel 5VHiRab device firmware upgrade utility
 * @warning     Device firmware upgrade utility allows both upgrade to new version of firmware
 *              and also downgrade to older version. User can use voxel3d_read_fw_version() to
 *              confirm the firmware version running on device. It can also be used to confirm
 *              if the firmware version on device after upgrade.
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[in]   file_path: point to the location of the new firmware image string
 * @return      true: completed sending specific firmware to device for upgrade successfully
 * @return      < 0: upgrade failure
 */
extern "C" VOXEL3D_API_DLL int voxel3d_dev_fw_upgrade(char* dev_sn,
    char* file_path,
    unsigned char (*fw_upgrade_cb)(int state, unsigned int percent_complete));


/**
 * @brief       Function to poll the state and 5voxel 5VHiRab device firmware upgrade utility
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  state: reference of the state varaible for API to write current fw download state
 *              state: < 0 (error), 0: (initial), 1 (downloading), 2 (complete)
 * @param[out]  percent_complete: reference of the percentage varaible for API to write current percentage of completion
 *              percent_complete: 0 ~ 100
 * @return      true: In upgrade procedure
 * @return      < 0: Not in upgrade procedure
 *              Note: while output < 0, state & percent_complete can be used to know if the previous upgrade
 *                    had error or completed withtout failure
 */
extern "C" VOXEL3D_API_DLL int voxel3d_dev_fw_upgrade_state_poll(char* dev_sn, int& state,
    unsigned int& percent_complete);


#endif /* __VOXEL3d_H__ */

