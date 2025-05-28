/**
 @file      voxel3d_app.cpp
 @brief     5VHiRab all-in-one RGB-D & thermal camera example
 @author    Yushan Chen
 @copyright Copyright (c) 2025 5Voxel Co., Ltd.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>             /* getopt_long() */
#include <errno.h>
#include<iostream>
#ifdef PLAT_WINDOWS
#include <windows.h>
#else
#include <unistd.h>
#endif /* PLAT_WINDOWS */

#include "opencv2/core/version.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "voxel3d.h"

#define TOOLS_VER_MAJOR         (1)
#define TOOLS_VER_MINOR         (5)

#define FLIR_DISP_WIDTH         (320)
#define FLIR_DISP_HEIGHT        (240)

#define M_PI                    (3.141592653589793f)

#ifdef PLAT_WINDOWS
#define SleepSeconds(x)        Sleep(x * 1000)
#else /* PLAT_LINUX */
#define SleepSeconds(x)        sleep(x)
#endif /* PLAT_WINDOWS */

#define BYTE_SWAP_16(x)  \
           (((unsigned short)(x & 0x00ff) << 8) + ((unsigned short)(x & 0xff00) >> 8))

static int              auto_exposure_mode = 0;
static int              conf_threshold = 5;
static int              found_tof_device = 0, found_flir_device = 0, found_rgb_device = 0;
static unsigned short   tof_depth[TOF_DEPTH_PIXELS];
static unsigned short   tof_ir[TOF_IR_PIXELS];
static unsigned short   tof_flir[FLIR_PIXELS];
static float            pointCloudXYZ[TOF_DEPTH_PIXELS * 3];
static int              iWaitKey = 0;

static bool m_doRGBDRectify = false;
static bool m_doTDRectify = false;


static int track_mouse_position = 1;
static int mouse_x = TOF_DEPTH_WIDTH >> 1, mouse_y = TOF_DEPTH_HEIGHT >> 1;


static int track_thermal_mouse_position = 1;
static int thermal_mouse_x = FLIR_DISP_WIDTH >> 1, thermal_mouse_y = FLIR_DISP_HEIGHT >> 1;


using namespace std;
using namespace cv;

cv::Mat depthraw = cv::Mat(TOF_DEPTH_HEIGHT >> 1, TOF_DEPTH_WIDTH, CV_16UC1, cv::Scalar(0));
cv::Mat depth = cv::Mat(TOF_DEPTH_HEIGHT, TOF_DEPTH_WIDTH, CV_16UC1, cv::Scalar(0));
cv::Mat depth_tmp = cv::Mat(TOF_DEPTH_HEIGHT, TOF_DEPTH_WIDTH, CV_16UC1, cv::Scalar(0));
cv::Mat confraw = cv::Mat(TOF_DEPTH_HEIGHT >> 1, TOF_DEPTH_WIDTH, CV_16UC1, cv::Scalar(0));
cv::Mat conf = cv::Mat(TOF_DEPTH_HEIGHT, TOF_DEPTH_WIDTH, CV_16UC1, cv::Scalar(0));

cv::Mat rectify_rgb = cv::Mat(TOF_DEPTH_HEIGHT, TOF_DEPTH_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
cv::Mat rgb = cv::Mat(RGB_HEIGHT, RGB_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
cv::Mat rectify_flir = cv::Mat(TOF_DEPTH_HEIGHT, TOF_DEPTH_WIDTH, CV_32FC1, cv::Scalar(0));
cv::Mat flir = cv::Mat(FLIR_HEIGHT, FLIR_WIDTH, CV_32FC1, cv::Scalar(0));

static void errno_exit(const char *s)
{
    fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
    exit(EXIT_FAILURE);
}

static unsigned char fw_upgrade_cb(int state, unsigned int percent_complete)
{
    switch (state) {
    case 0:
        printf("\n\rFW upgrade - state: Initial             ");
        break;
    case 1:
        printf("\n\rFW upgrade - state: Downloading (%d%%)", percent_complete);
        break;
    case 2:
        printf("\n\rFW upgrade - state: Complete            ");
        break;
    default:
        printf("\n\rFW upgrade - state: Error               ");
        break;
    }

    return (1);
}

void ToFCallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        track_mouse_position = 0;
    }
    else if (event == EVENT_RBUTTONDOWN)
    {
        track_mouse_position = 1;
    }
    else if (event == EVENT_MBUTTONDOWN)
    {
    }
    else if (event == EVENT_MOUSEMOVE)
    {
        if (track_mouse_position) {
            mouse_x = x;
            mouse_y = y;
        }
    }
}

void FlirCallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        track_mouse_position = 0;
    }
    else if (event == EVENT_RBUTTONDOWN)
    {
        track_mouse_position = 0;
    }
    else if (event == EVENT_MBUTTONDOWN)
    {
    }
    else if (event == EVENT_MOUSEMOVE)
    {
        if (track_mouse_position) {
            mouse_x = x;
            mouse_y = y;
        }
    }
}

void RGBCallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        track_mouse_position = 0;
    }
    else if (event == EVENT_RBUTTONDOWN)
    {
        track_mouse_position = 1;
    }
    else if (event == EVENT_MBUTTONDOWN)
    {
    }
    else if (event == EVENT_MOUSEMOVE)
    {
        if (track_mouse_position) {
            mouse_x = x;
            mouse_y = y;
        }
    }
}


static void mainloop(char* dev_sn)
{
    int pcl_pixels = 0;
    int tof_center_pixel_loc = TOF_DEPTH_WIDTH * (TOF_DEPTH_HEIGHT / 2) +
                           (TOF_DEPTH_WIDTH >> 1);
    int flir_center_pixel_loc = FLIR_WIDTH * (FLIR_HEIGHT >> 1) +
        (FLIR_WIDTH >> 1);
    int rgb_center_pixel_loc = RGB_WIDTH * (RGB_HEIGHT >> 1) + (RGB_WIDTH >> 1);

    Mat colorMap, conf8u, depth8U, flir8U;
    IMU_DATA imu_data = { 0, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f };

    cv::Mat operateMat = cv::Mat::zeros(200, 400, CV_8U);
    cv::putText(operateMat, "Press F to get RGB-D", cv::Point(10, 60), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 255, 255));
    cv::putText(operateMat, "Press T to get Thermal-D", cv::Point(10, 80), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 255, 255));
    cv::imshow("operate", operateMat);

    while (iWaitKey != 27) {
        if (iWaitKey == 'f')
        {           
            m_doRGBDRectify = !m_doRGBDRectify;
            m_doTDRectify = false;
            voxel3d_set_rectifyType(dev_sn, RectifyType::RGB2TOF);

            if (m_doRGBDRectify)
            {
                std::cout << "open RGB-D" << std::endl;
            }
            else {
                std::cout << "close RGB-D" << std::endl;
            }
        }
        else if (iWaitKey == 't')
        {
            m_doRGBDRectify = false;
            m_doTDRectify = !m_doTDRectify;
            voxel3d_set_rectifyType(dev_sn, RectifyType::FLIR2TOF);
            if (m_doTDRectify)
            {
                std::cout << "open Thermal-D" << std::endl;
            }
            else {
                std::cout << "close Thermal-D" << std::endl;
            }
        }

        if (!m_doRGBDRectify && !m_doTDRectify)
        {
            voxel3d_set_rectifyType(dev_sn, RectifyType::NONE);
        }

        if (found_tof_device) {
            unsigned int ret = voxel3d_tof_queryframe(dev_sn, depth.ptr<unsigned short>(0), conf.ptr<unsigned short>(0));
            if (ret) {
                pcl_pixels = voxel3d_tof_generatePointCloud(
                    dev_sn,
                    depth.ptr<unsigned short>(0),
                    pointCloudXYZ);

                conf.convertTo(conf8u, CV_8UC1, 255.0 / 1024.f);
                putText(conf8u, format("%d", conf.at<unsigned short>(mouse_y, mouse_x)), Point(mouse_x, mouse_y), 1, 1, Scalar(255, 255, 255));
                imshow("IR", conf8u);

                for (int ix = 0; ix < TOF_DEPTH_WIDTH * TOF_DEPTH_HEIGHT; ix++) {
                    depth_tmp.at<unsigned short>(ix) = depth.at<unsigned short>(ix) % 1024;
                }
                depth_tmp.convertTo(depth8U, CV_8UC1, 255.0 / 1024);
                applyColorMap(depth8U, colorMap, COLORMAP_JET);
                int pcl_idx = (mouse_y * TOF_DEPTH_WIDTH + mouse_x) * 3;
                putText(colorMap, format("(%.3f, %.3f, %.3f)", pointCloudXYZ[pcl_idx], pointCloudXYZ[pcl_idx + 1], pointCloudXYZ[pcl_idx + 2]),
                    Point(mouse_x, mouse_y), 1, 1, Scalar(255, 255, 255));
                imshow("Depth", colorMap);
               
                if (found_rgb_device)
                {
                    if (m_doRGBDRectify)
                    {
                        ret = voxel3d_rgb_queryframe(dev_sn, rectify_rgb.ptr<uchar>(0));
                        
                    }
                    else {
                        ret = voxel3d_rgb_queryframe(dev_sn, rgb.ptr<uchar>(0));
                        cv::resize(rgb, rectify_rgb, rectify_rgb.size());
                    }
                    putText(rectify_rgb, format("%d, %d, %d", rectify_rgb.at<cv::Vec3b>(mouse_y, mouse_x)(0), rectify_rgb.at<cv::Vec3b>(mouse_y, mouse_x)(1), rectify_rgb.at<cv::Vec3b>(mouse_y, mouse_x)(2)), Point(mouse_x, mouse_y), 1, 1, Scalar(255, 255, 255));

                    imshow("RGB", rectify_rgb);
                }

                if (found_flir_device) {
                    if (m_doTDRectify)
                    {
                        ret = voxel3d_lepton3_queryframe(dev_sn, rectify_flir.ptr<float>(0));                        
                    }
                    else
                    {
                        ret = voxel3d_lepton3_queryframe(dev_sn, flir.ptr<float>(0));                       
                        cv::resize(flir, rectify_flir, rectify_flir.size());               
                    }
                    rectify_flir.convertTo(flir8U, CV_8UC1, 255.0 / 40.0);
                    putText(flir8U, format("%f", rectify_flir.at<float>(mouse_y, mouse_x)), Point(mouse_x, mouse_y), 1, 1, Scalar(0, 0, 0));
                    imshow("Thermal", flir8U);
                }
            }
        }

        //set the callback function for any mouse event
        setMouseCallback("Depth", ToFCallBackFunc, NULL);

        if (voxel3d_read_imu_data(dev_sn, &imu_data)) {
            printf("IMU TS = %ld, ACC (%.4f, %.4f, %.4f), GYRO (%.4f, %.4f, %.4f)\n",
                imu_data.imu_ts, imu_data.imu_accel[0], imu_data.imu_accel[1], imu_data.imu_accel[2],
                imu_data.imu_gyro[0], imu_data.imu_gyro[1], imu_data.imu_gyro[2]);
        }

        iWaitKey = waitKey(5);
    }
    return;
}

static void usage(FILE *fp, int argc, char **argv)
{
    fprintf(fp,
         "Usage: %s [options]\n\n"
         "Version %d.%d\n"
         "Options:\n"
         "-h | --help             Print this message\n"
         "-A | --set_auto_expo    set auto exposure mode\n"
         "-b | --build_date       show firmware build date\n"
         "-i | --show_info        show device info\n"
         "-S | --scan_dev         scan devices and list device S/N\n"
         "-s | --dev_sn           specify device S/N to access\n"
         "-t | --get_conf         get confidence threshold\n"
         "-T | --set_conf         set confidence threshold\n"
         "-u | --fw_upgrade       device firmware upgrade\n"
         "-v | --version          show lib & firmware version\n"
         "\n",
         argv[0], TOOLS_VER_MAJOR, TOOLS_VER_MINOR);
}

static const char short_options[] = "hA:bpiSs:tT:u:v";

static const struct option
long_options[] = {
    { "help",              no_argument,       NULL, 'h' },
    { "set_auto_expo",     required_argument, NULL, 'A' },
    { "build_date",        no_argument,       NULL, 'b' },
    { "prod_sn",           no_argument,       NULL, 'p' },
    { "show_info",         no_argument,       NULL, 'i' },
    { "scan_dev",          no_argument,       NULL, 'S' },
    { "dev_sn",            required_argument, NULL, 's' },
    { "get_conf",          no_argument,       NULL, 't' },
    { "set_conf",          required_argument, NULL, 'T' },
    { "fw_upgrade",        required_argument, NULL, 'u' },
    { "version",           no_argument,       NULL, 'v' },
    { 0, 0, 0, 0 }
};

int main(int argc, char **argv)
{
    int wait_time = 60;
    char data[64];
    int  result;
    CamDevInfo devInfo;
    char dev_sn[MAX_PRODUCT_SN_LEN] = {'\0'};

    
    for (;;) {
        int idx;
        int c;

        c = getopt_long(argc, argv,
                        short_options, long_options, &idx);

        if (-1 == c)
            break;

        switch (c) {
        case 0:
            break;

        case 'h':
            usage(stdout, argc, argv);
            exit(EXIT_SUCCESS);

        case 'A':
            errno = 0;
            auto_exposure_mode = strtol(optarg, NULL, 0);
            if (errno)
                errno_exit(optarg);
            found_tof_device = voxel3d_tof_init(dev_sn);
            if (found_tof_device > 0) {
                result = voxel3d_tof_set_auto_exposure_mode(dev_sn, auto_exposure_mode);
                if (result > 0) {
                    printf("Set Auto Exposure Mode : %d\n", auto_exposure_mode);
                }
            }
            voxel3d_tof_release(dev_sn);
            exit(EXIT_SUCCESS);

        case 'b':
            found_tof_device = voxel3d_tof_init(dev_sn);
            if (found_tof_device > 0) {
                memset(data, 0x0, sizeof(data));
                voxel3d_read_fw_build_date(dev_sn, data, sizeof(data));
                printf("FW build date : %s\n", data);
            }
            voxel3d_tof_release(dev_sn);
            exit(EXIT_SUCCESS);

        case 'i':
        {
            float vfov = 0, hfov = 0;
            found_tof_device = voxel3d_tof_init(dev_sn);
            if (found_tof_device > 0) {
                hfov = voxel3d_tof_get_depth_hfov(dev_sn);
                printf("HFoV : %f (%f degree)\n", hfov, hfov * 180 / M_PI);
                vfov = voxel3d_tof_get_depth_vfov(dev_sn);
                printf("VFoV : %f (%f degree)\n", vfov, vfov * 180 / M_PI);
            }
            voxel3d_tof_release(dev_sn);
            exit(EXIT_SUCCESS);

            break;
        }

        case 's':
            strncpy(dev_sn, optarg, MAX_PRODUCT_SN_LEN);
            break;

        case 'S':
            voxel3d_scan(&devInfo);
            if (devInfo.num_of_devices > 0) {
                printf("Found %d devices\n", devInfo.num_of_devices);
                for (int ix = 0; ix < devInfo.num_of_devices; ix++) {
                    printf("%d: Name = %s, SN = %s, W = %d, H = %d\n",
                        ix, devInfo.dev_name[ix], devInfo.product_sn[ix],
                        devInfo.resolution[ix].width, devInfo.resolution[ix].height);
                }
            }
            else {
                printf("Can't find any 5Voxel device\n");
            }
            exit(EXIT_SUCCESS);

        case 't':
            found_tof_device = voxel3d_tof_init(dev_sn);
            if (found_tof_device > 0) {
                result = voxel3d_tof_get_conf_threshold(dev_sn);
                if (result >= 0) {
                    printf("Confidence Threshold : %d\n", result);
                }
                else {
                    printf("Get confidence threshold failed (%d)\n", result);
                }
            }
            voxel3d_tof_release(dev_sn);
            exit(EXIT_SUCCESS);

        case 'T':
            errno = 0;
            conf_threshold = strtol(optarg, NULL, 0);
            if (errno)
                errno_exit(optarg);

            found_tof_device = voxel3d_tof_init(dev_sn);
            if (found_tof_device > 0) {
                result = voxel3d_tof_set_conf_threshold(dev_sn, conf_threshold);
                if (result > 0) {
                    printf("Set Confidence Threshold : %d\n", conf_threshold);
                }
                else {
                    printf("Set confidence threshold failed (%d)\n", result);
                }
            }
            voxel3d_tof_release(dev_sn);
            exit(EXIT_SUCCESS);

        case 'u':
            found_tof_device = voxel3d_tof_init(dev_sn);
            if (found_tof_device > 0) {
                memset(data, 0x0, sizeof(data));
                voxel3d_read_fw_version(dev_sn, data, sizeof(data));
                printf("--------------------------------------------------------\n");
                printf("Before F/W upgrade\n");
                printf("F/W file     : %s\n", optarg);
                printf("F/W version  : %s\n", data);
                printf("--------------------------------------------------------\n\n");
                result = voxel3d_dev_fw_upgrade(dev_sn, optarg, fw_upgrade_cb);
                voxel3d_release(dev_sn);
                if (result < 0) {
                    printf("5HiRab FW upgrade failed\n");
                    exit(EXIT_SUCCESS);
                }

                printf("Firmware upgrade completed. Need manual poewr cycle to take effect\n");
                voxel3d_release(dev_sn);
                exit(EXIT_SUCCESS);
            }
            break;

        case 'v':
            found_tof_device = voxel3d_tof_init(dev_sn);
            if (found_tof_device > 0) {
                int ret;
                SleepSeconds(1);
                memset(data, 0x0, sizeof(data));
                ret = voxel3d_read_lib_version(data, sizeof(data));
                if (ret < 0) {
                    printf("Failed to read share library version\n");
                }
                else {
                    printf("Share library version : %s\n", data);
                }

                memset(data, 0x0, sizeof(data));
                ret = voxel3d_read_fw_version(dev_sn, data, sizeof(data));
                if (ret < 0) {
                    printf("Failed to read 5VHiRab F/W version\n");
                }
                else {
                    printf("5VHiRab F/W version   : %s\n", data);
                }
            }
            voxel3d_tof_release(dev_sn);
            exit(EXIT_SUCCESS);

        default:
            usage(stderr, argc, argv);
            exit(EXIT_SUCCESS);
        }
    }
    
    /*
     * Start device
     */
    found_tof_device = voxel3d_tof_init(dev_sn);
    found_flir_device = voxel3d_lepton3_init(dev_sn);
    found_rgb_device = voxel3d_rgb_init(dev_sn);

    m_doRGBDRectify = false;
    m_doTDRectify = false;

    voxel3d_set_rectifyType(dev_sn, RectifyType::NONE);
    /*
     * main loop function
     */
    if (found_tof_device > 0 || found_flir_device > 0 || found_rgb_device > 0) {
        mainloop(dev_sn);
    }

    /*
     * Stop device
     */
    if (found_tof_device > 0) {
        voxel3d_tof_release(dev_sn);
    }

    if (found_flir_device > 0) {
        voxel3d_lepton3_release(dev_sn);
    }

    if (found_rgb_device > 0)
    {
        voxel3d_rgb_release(dev_sn);
    }

    return 0;
}

