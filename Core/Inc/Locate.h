#ifndef __LOCATE_H
#define __LOCATE_H

#include <stdint.h>

typedef struct {
    uint8_t gga_valid;      // GGA 数据有效标志
    float latitude;         // 纬度 (度)
    float longitude;        // 经度 (度)
    float altitude;         // 海拔高度 (米)
    uint8_t fix_quality;    // 定位质量: 0=无效,1=单点,2=差分,4=RTK固定,5=RTK浮点
    uint8_t sat_num;        // 参与定位卫星数
    float hdop;             // 水平精度因子

    uint8_t rmc_valid;      // RMC数据有效标志
    float speed_knot;       // 速度 (节)
    float speed_kmh;        // 速度 (公里/小时)
    float track_angle;      // 航向角 (度)
    uint8_t utc_hour;       // UTC时
    uint8_t utc_min;        // UTC分
    uint8_t utc_sec;        // UTC秒
    uint16_t utc_millisec;  // UTC毫秒
} GNSS_Data_t;

void parse_gnss_sentence(const char* sentence, uint16_t len);
void get_gnss_data(GNSS_Data_t* data);
void print_gnss_data(void);
void send_gnss_data_uart1(const GNSS_Data_t* data);
void parse_BESTNAVA(const char* message);

#endif // __LOCATE_H
