#ifndef POINT_LIO_FFI_H
#define POINT_LIO_FFI_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* ABI v1. All arrays xyzw for quaternions, row-major for covariance.
 * Functions return 0=ok, 1=initializing, 2=not stationary, 3=needs IMU,
 * 4=old time, 5=IMU gap, 6=buffer full, 7=bad input, 8=numerical, 9=panic.
 * Handle is exclusively owned; never call concurrently. Pointers must be valid
 * and aligned for the documented object/slice lifetime. No Rust ABI types. */
typedef struct {
  double translation[3], quaternion[4];
  double min_range, max_range, voxel_size, filter_size_map;
  uint64_t map_capacity;
} PlioConfig;
typedef struct {
  double time, gyro[3], acceleration[3];
} PlioImu;
typedef struct {
  double position[3], offset;
  float intensity;
} PlioPoint;
typedef struct {
  double time, position[3], quaternion[4], velocity_body[3],
      angular_velocity[3];
  double pose_covariance[36], twist_covariance[36];
  uint64_t matches, map_points, map_voxels;
} PlioOutput;
uint32_t plio_abi_version(void);
void *plio_create(const PlioConfig *config);
void plio_destroy(void *handle);
int32_t plio_reset(void *handle);
int32_t plio_push_imu(void *handle, const PlioImu *imu);
/* Returns initialized filter time or status 1. */
int32_t plio_state_time(void *handle, double *time);
int32_t plio_scan(void *handle, double start, double end,
                  const PlioPoint *points, size_t count, PlioOutput *output);
#ifdef __cplusplus
}
#endif
#endif
