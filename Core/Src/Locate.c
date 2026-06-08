#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "locate.h"
#include "usart.h"
#include "atk_mw1278d_uart.h"

GNSS_Data_t gnss_data = {0};

// 字符串转浮点数辅助函数
float parse_float(const char* str, int len) {
    char buf[32];
    if (len > 31) len = 31;
    strncpy(buf, str, len);
    buf[len] = '\0';
    return atof(buf);
}

// 解析纬度/纬度方向 (格式: DDMM.MMMMMMM)
float parse_latitude(const char* lat_str, char dir) {
    float lat = atof(lat_str);
    int degrees = (int)(lat / 100);
    float minutes = lat - (degrees * 100);
    float decimal_lat = degrees + minutes / 60.0;
    
    if (dir == 'S') decimal_lat = -decimal_lat;
    return decimal_lat;
}

// 解析经度 (格式: DDDMM.MMMMMMM)
float parse_longitude(const char* lon_str, char dir) {
    float lon = atof(lon_str);
    int degrees = (int)(lon / 100);
    float minutes = lon - (degrees * 100);
    float decimal_lon = degrees + minutes / 60.0;
    
    if (dir == 'W') decimal_lon = -decimal_lon;
    return decimal_lon;
}

// 解析UTC时间
void parse_utc_time(const char* time_str, GNSS_Data_t* data) {
    if (strlen(time_str) >= 6) {
        char hour_str[3] = {0}, min_str[3] = {0}, sec_str[3] = {0};
        char millisec_str[4] = {0};
        
        strncpy(hour_str, time_str, 2);
        strncpy(min_str, time_str + 2, 2);
        strncpy(sec_str, time_str + 4, 2);
        
        data->utc_hour = atoi(hour_str);
        data->utc_min = atoi(min_str);
        data->utc_sec = atoi(sec_str);
        
        // 解析毫秒（如果有）
        if (strlen(time_str) > 7) {
            strncpy(millisec_str, time_str + 7, 3);
            data->utc_millisec = atoi(millisec_str);
        }
    }
}

// 解析GNGGA消息
// $GNGGA,044818.00,3943.50882459,N,11610.06980763,E,1,20,1.2,38.1010,M,-9.1630,M,,*59
void parse_GNGGA(const char* sentence) {
    char* token;
    char buffer[256];
    strncpy(buffer, sentence, sizeof(buffer) - 1);
    
    int field_index = 0;
    char* save_ptr;
    
    token = strtok_r(buffer, ",", &save_ptr);
    
    while (token != NULL && field_index <= 12) {
        switch(field_index) {
            case 1:  // UTC时间
                parse_utc_time(token, &gnss_data);
                break;
            case 2:  // 纬度 (格式: DDMM.MMMMM) — 只取值，方向由 case 3 处理
                if (strlen(token) > 0) {
                    gnss_data.latitude = parse_latitude(token, 'N');
                }
                break;
            case 3:  // 纬度方向 N/S
                if (token[0] == 'S') gnss_data.latitude = -gnss_data.latitude;
                break;
            case 4:  // 经度 (格式: DDDMM.MMMMM) — 只取值，方向由 case 5 处理
                if (strlen(token) > 0) {
                    gnss_data.longitude = parse_longitude(token, 'E');
                }
                break;
            case 5:  // 经度方向 E/W
                if (token[0] == 'W') gnss_data.longitude = -gnss_data.longitude;
                break;
            case 6:  // 定位质量
                gnss_data.fix_quality = atoi(token);
                break;
            case 7:  // 卫星数量
                gnss_data.sat_num = atoi(token);
                break;
            case 8:  // HDOP
                gnss_data.hdop = atof(token);
                break;
            case 9:  // 海拔高度
                gnss_data.altitude = atof(token);
                break;
            default:
                break;
        }
        field_index++;
        token = strtok_r(NULL, ",", &save_ptr);
    }
    
    gnss_data.gga_valid = (gnss_data.fix_quality > 0);
}

// 解析GNRMC消息
// $GNRMC,044818.00,A,3943.50882459,N,11610.06980763,E,0.010,304.6,300526,7.2,W,A,C*50
void parse_GNRMC(const char* sentence) {
    char* token;
    char buffer[256];
    strncpy(buffer, sentence, sizeof(buffer) - 1);
    
    int field_index = 0;
    char* save_ptr;
    char lat_dir = 'N';  // 纬度方向
    char lon_dir = 'E';  // 经度方向
    
    token = strtok_r(buffer, ",", &save_ptr);
    
    while (token != NULL && field_index <= 12) {
        switch(field_index) {
            case 1:  // UTC时间
                parse_utc_time(token, &gnss_data);
                break;
            case 2:  // 状态 A=有效 V=无效
                gnss_data.rmc_valid = (token[0] == 'A');
                break;
            case 3:  // 纬度
                if (strlen(token) > 0 && gnss_data.rmc_valid) {
                    gnss_data.latitude = parse_latitude(token, lat_dir);
                }
                break;
            case 4:  // 纬度方向
                if (token[0] == 'S' || token[0] == 'N')
                    lat_dir = token[0];
                break;
            case 5:  // 经度
                if (strlen(token) > 0 && gnss_data.rmc_valid) {
                    gnss_data.longitude = parse_longitude(token, lon_dir);
                }
                break;
            case 6:  // 经度方向
                if (token[0] == 'E' || token[0] == 'W')
                    lon_dir = token[0];
                break;
            case 7:  // 速度(节)
                gnss_data.speed_knot = atof(token);
                gnss_data.speed_kmh = gnss_data.speed_knot * 1.852;  // 节转km/h
                break;
            case 8:  // 航向角
                gnss_data.track_angle = atof(token);
                break;
            default:
                break;
        }
        field_index++;
        token = strtok_r(NULL, ",", &save_ptr);
    }
}

// NMEA语句校验
uint8_t nmea_checksum(const char* sentence) {
    const char* p = sentence;
    
    // 跳过所有前导 $ (处理双$$情况)
    while (*p == '$') p++;
    
    uint8_t checksum = 0;
    while (*p != '*' && *p != '\0' && *p != '\r' && *p != '\n') {
        checksum ^= *p;
        p++;
    }
    
    // 读取消息中的校验和
    if (*p == '*') {
        p++;
        char hex_str[3] = {0};
        hex_str[0] = *p++;
        hex_str[1] = *p;
        uint8_t msg_checksum = (uint8_t)strtol(hex_str, NULL, 16);
        
        return (checksum == msg_checksum);
    }
    
    return 0;
}

// 主解析函数 - 在串口接收中断中调用
void parse_gnss_sentence(const char* sentence, uint16_t len) {
    if (len < 6) return;
    
    // 校验和验证
    if (!nmea_checksum(sentence)) {
        return;  // 校验失败
    }
    
    // 根据消息类型解析
    if (strstr(sentence, "$GNGGA") != NULL || strstr(sentence, "$GPGGA") != NULL) {
        parse_GNGGA(sentence);
    }
    else if (strstr(sentence, "$GNRMC") != NULL || strstr(sentence, "$GPRMC") != NULL) {
        parse_GNRMC(sentence);
    }
}

// 获取解析后的位置速度数据
void get_gnss_data(GNSS_Data_t* data) {
    memcpy(data, &gnss_data, sizeof(GNSS_Data_t));
}

// 打印位置速度信息（调试用）
void print_gnss_data(void) {
    printf("\r\n========== GNSS Data ==========\r\n");
    printf("Time: %02d:%02d:%02d.%03d\r\n", 
        gnss_data.utc_hour, gnss_data.utc_min, gnss_data.utc_sec, gnss_data.utc_millisec);
    printf("Position: %.8f, %.8f\r\n", gnss_data.latitude, gnss_data.longitude);
    printf("Altitude: ");
    {
        int alt_int = (int)gnss_data.altitude;
        int alt_frac = (int)((gnss_data.altitude - alt_int) * 100);
        if (alt_frac < 0) alt_frac = -alt_frac;
        printf("%d.%02d m\r\n", alt_int, alt_frac);
    }
    printf("Fix Quality: %d ", gnss_data.fix_quality);
    switch(gnss_data.fix_quality) {
        case 1: printf("(Single Point)\r\n"); break;
        case 2: printf("(DGPS)\r\n"); break;
        case 4: printf("(RTK Fixed)\r\n"); break;
        case 5: printf("(RTK Float)\r\n"); break;
        default: printf("(Invalid)\r\n"); break;
    }
    printf("Satellites: %d\r\n", gnss_data.sat_num);
    printf("HDOP: ");
    {
        int hdop_int = (int)gnss_data.hdop;
        int hdop_frac = (int)((gnss_data.hdop - hdop_int) * 100);
        if (hdop_frac < 0) hdop_frac = -hdop_frac;
        printf("%d.%02d\r\n", hdop_int, hdop_frac);
    }
    printf("Speed: ");
    {
        int sk_int = (int)gnss_data.speed_kmh;
        int sk_frac = (int)((gnss_data.speed_kmh - sk_int) * 100);
        if (sk_frac < 0) sk_frac = -sk_frac;
        int sn_int = (int)gnss_data.speed_knot;
        int sn_frac = (int)((gnss_data.speed_knot - sn_int) * 100);
        if (sn_frac < 0) sn_frac = -sn_frac;
        printf("%d.%02d km/h (%d.%02d knots)\r\n", sk_int, sk_frac, sn_int, sn_frac);
    }
    printf("Heading: ");
    {
        int hdg_int = (int)gnss_data.track_angle;
        int hdg_frac = (int)((gnss_data.track_angle - hdg_int) * 10);
        if (hdg_frac < 0) hdg_frac = -hdg_frac;
        printf("%d.%01d deg\r\n", hdg_int, hdg_frac);
    }
    printf("================================\r\n");
}

void send_gnss_data_uart1(const GNSS_Data_t* data) {
    if (data == NULL) {
        return;
    }

    /* 手动拆解浮点数，避免 newlib-nano 不支持 %f */
    int lat_int = (int)data->latitude;
    int lat_frac = (int)((data->latitude - lat_int) * 100000000);
    if (lat_frac < 0) lat_frac = -lat_frac;

    int lon_int = (int)data->longitude;
    int lon_frac = (int)((data->longitude - lon_int) * 100000000);
    if (lon_frac < 0) lon_frac = -lon_frac;

    int spd_int = (int)data->speed_kmh;
    int spd_frac = (int)((data->speed_kmh - spd_int) * 100);
    if (spd_frac < 0) spd_frac = -spd_frac;

    int hdg_int = (int)data->track_angle;
    int hdg_frac = (int)((data->track_angle - hdg_int) * 10);
    if (hdg_frac < 0) hdg_frac = -hdg_frac;

    atk_mw1278d_uart_printf("UTC Time: %02d:%02d:%02d.%03d\r\n",
        data->utc_hour, data->utc_min, data->utc_sec, data->utc_millisec);
    atk_mw1278d_uart_printf("Latitude: %d.%08d\r\n", lat_int, lat_frac);
    atk_mw1278d_uart_printf("Longitude: %d.%08d\r\n", lon_int, lon_frac);
    atk_mw1278d_uart_printf("Speed: %d.%02d km/h\r\n", spd_int, spd_frac);
    atk_mw1278d_uart_printf("Heading: %d.%01d deg\r\n", hdg_int, hdg_frac);
    atk_mw1278d_uart_printf("--------------------------------\r\n");
}

// 如果需要RTK高精度位置（固定解），解析BESTNAVA消息
// #BESTNAVA,97,GPS,FINE,2190,364622000,0,0,18,9;SOL_COMPUTED,SINGLE,40.07898353385,116.23662959156,60.2103,... 
void parse_BESTNAVA(const char* message) {
    // BESTNAVA是Unicore二进制/ASCII格式，以#开头
    // 格式: #BESTNAVA,header;SOL_COMPUTED,SINGLE,lat,lon,height,...
    char* p = strchr(message, ';');
    if (p == NULL) return;
    p++; // 跳过';'
    
    char* token;
    char buffer[256];
    strncpy(buffer, p, sizeof(buffer) - 1);
    
    int field_index = 0;
    char* save_ptr;
    token = strtok_r(buffer, ",", &save_ptr);
    
    while (token != NULL && field_index <= 8) {
        switch(field_index) {
            case 1:  // 位置类型 (SINGLE, NARROW_INT等)
                if (strcmp(token, "NARROW_INT") == 0) {
                    gnss_data.fix_quality = 4;  // RTK固定解
                } else if (strcmp(token, "NARROW_FLOAT") == 0) {
                    gnss_data.fix_quality = 5;  // RTK浮点解
                }
                break;
            case 2:  // 纬度
                gnss_data.latitude = atof(token);
                break;
            case 3:  // 经度
                gnss_data.longitude = atof(token);
                break;
            case 4:  // 高度
                gnss_data.altitude = atof(token);
                break;
        }
        field_index++;
        token = strtok_r(NULL, ",", &save_ptr);
    }
}