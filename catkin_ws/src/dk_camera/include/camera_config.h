//
// Created by slam on 2020/6/30.
//

#ifndef DK_CAMERA_CAMERA_CONFIG_H
#define DK_CAMERA_CAMERA_CONFIG_H

#include <map>
#include <string>
#include <vector>
#include "k4a/k4atypes.h"

enum class CameraId: uint8_t
{
    UP = 0,
    CENTER = 1,
    DOWN = 2
};

class CameraConfig
{
public:
    static std::map<std::string, CameraId> camera_sn_map_;
};

enum EXPOSURE_LEVEL:int {
    //level    value    exposure time(us)
    LEVEL_AUTO  = 0,
    LEVEL_1     = 488,     //500
    LEVEL_2     = 977,     //1250
    LEVEL_3     = 1953,    //2500
    LEVEL_4     = 3906,    //10000/8330
    LEVEL_6     = 7813,    //20000/16670
    LEVEL_7     = 15625,   //30000/33330
    LEVEL_8     = 31250,   //40000/41670
    LEVEL_9     = 62500,   //50000
    LEVEL_10    = 125000,  //60000/66670
    LEVEL_11    = 250000,  //80000/83330
    LEVEL_12    = 500000,  //100000
    LEVEL_13    = 1000000, //120000/116670
};

// What: camera info struct
struct DKCamera
{
    int scan_id;
    int camera_id;
    std::string color_path;
    std::string depth_path;
    std::string imu_path;
    std::vector<std::string> color;
    std::vector<std::string> depth;
    std::vector<std::string> imu;
    std::vector<int> angle;
};

// 单次scan信息
struct SingleScan
{
    int id;
//    DKCamera rs1, rs2, rs3;
    std::vector<DKCamera> rs_camera;
};

struct Project
{
    std::string project_name;
    std::vector<SingleScan> scans;
};

const int DKNUMBER = 3;

#endif //DK_CAMERA_CAMERA_CONFIG_H
