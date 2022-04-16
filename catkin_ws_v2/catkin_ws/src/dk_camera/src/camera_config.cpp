//
// Created by slam on 2020/6/30.
//

#include "../include/camera_config.h"

std::map<std::string, CameraId> CameraConfig::camera_sn_map_ = {
    {"000233602412", CameraId::UP},
    {"000325402912", CameraId::CENTER},
    {"000173302912", CameraId::DOWN},
    // {"000457501212",CameraId::UP},
    // {"000382101212",CameraId::CENTER},
    // {"000446701212",CameraId::DOWN}
};