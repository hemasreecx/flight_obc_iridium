#pragma once
#include <stdint.h>

namespace log_format
{

constexpr uint8_t COMMIT_INCOMPLETE = 0;
constexpr uint8_t COMMIT_COMPLETE   = 1;

#pragma pack(push, 1)
struct Record
{
    uint32_t counter;
    uint32_t gps_time; // sim
    int32_t  latitude; // sim
    int32_t  longitude; // sim
    int32_t  altitude; // sim

    int16_t thermocouples[20]; //sim

    int16_t acc3_x;
    int16_t acc3_y;
    int16_t acc3_z;

    int16_t imu_acc_x;
    int16_t imu_acc_y;
    int16_t imu_acc_z;
    int16_t imu_gyro_x;
    int16_t imu_gyro_y;
    int16_t imu_gyro_z;

    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;

    uint16_t battery_voltage; //sim
    int16_t  battery_current; //sim

    int16_t imu_temperature; 

    uint8_t commit;
};
#pragma pack(pop)

constexpr uint32_t RECORD_SIZE = sizeof(Record);

inline void init_record(Record& r)
{
    r = {};
}

inline bool is_valid(const Record& r)
{
    return r.commit == COMMIT_COMPLETE;
}

} // namespace log_format

