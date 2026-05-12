#ifndef ZENOH_BRIDGE_H
#define ZENOH_BRIDGE_H

#include "robot_types.h"

// Этот блок говорит C++, что функции внутри написаны на C
#ifdef __cplusplus
extern "C" {
#endif

int zenoh_init_lidar();
void zenoh_publish_lidar(lidar_scan_t* scan);

#ifdef __cplusplus
}
#endif

#endif
