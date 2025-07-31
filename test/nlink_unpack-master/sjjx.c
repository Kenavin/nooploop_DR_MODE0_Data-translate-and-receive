#include <stdio.h>
#include <stdint.h>
#include <windows.h>

#pragma pack(push, 1)
typedef struct
{
    uint8_t role;
    uint8_t id;
    uint8_t dis[3]; // 24位有符号整数（小端）
    uint8_t fp_rssi;
    uint8_t rx_rssi;
    uint32_t system_time;
    uint8_t reserved[2];
} nlt_nodeframe2_node_raw_t;

typedef struct
{
    uint8_t id;
    uint8_t role;
    uint32_t local_time;
    uint32_t system_time;
    float voltage;
    uint16_t valid_node_count;
    nlt_nodeframe2_node_raw_t nodes[16]; // 假设最多16个节点
} nlt_nodeframe2_result_t;
#pragma pack(pop)

// 声明外部解包函数
extern int g_nlt_nodeframe2_UnpackData(const uint8_t *data, size_t length);
extern nlt_nodeframe2_result_t g_nlt_nodeframe2_result;

#define BUFFER_SIZE 4096
#define FRAME_HEADER 0xA55A

// 状态机状态定义
typedef enum
{
    STATE_HEADER_1,
    STATE_HEADER_2,
    STATE_LENGTH,
    STATE_DATA
} parser_state_t;

// CRC16校验（参考网页1实现）
uint16_t crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

int main()
{
    HANDLE hSerial;
    DCB dcbSerialParams = {0};
    COMMTIMEOUTS timeouts = {0};

    // 打开COM4（根据实际情况修改）
    hSerial = CreateFile("\\\\.\\COM4", GENERIC_READ, 0, NULL,
                         OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    // 配置串口参数（参考网页2、12）
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    GetCommState(hSerial, &dcbSerialParams);
    dcbSerialParams.BaudRate = 921600;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    SetCommState(hSerial, &dcbSerialParams);

    // 设置超时（参考网页12）
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    SetCommTimeouts(hSerial, &timeouts);

    // 接收缓冲区
    uint8_t buffer[BUFFER_SIZE] = {0};
    size_t buffer_len = 0;
    parser_state_t state = STATE_HEADER_1;
    uint16_t expected_length = 0;

    while (1)
    {
        DWORD bytesRead;
        if (ReadFile(hSerial, buffer + buffer_len,
                     BUFFER_SIZE - buffer_len, &bytesRead, NULL))
        {
            buffer_len += bytesRead;

            // 状态机解析（参考网页1、3、11）
            while (buffer_len > 0)
            {
                switch (state)
                {
                case STATE_HEADER_1:
                    if (buffer[0] == (FRAME_HEADER >> 8))
                    {
                        state = STATE_HEADER_2;
                    }
                    memmove(buffer, buffer + 1, --buffer_len);
                    break;

                case STATE_HEADER_2:
                    if (buffer[0] == (FRAME_HEADER & 0xFF))
                    {
                        state = STATE_LENGTH;
                    }
                    else
                    {
                        state = STATE_HEADER_1;
                    }
                    memmove(buffer, buffer + 1, --buffer_len);
                    break;

                case STATE_LENGTH:
                    if (buffer_len >= 2)
                    {
                        expected_length = *(uint16_t *)buffer;
                        state = STATE_DATA;
                        memmove(buffer, buffer + 2, buffer_len -= 2);
                    }
                    else
                    {
                        return 0; // 等待更多数据
                    }
                    break;

                case STATE_DATA:
                    if (buffer_len >= expected_length + 2)
                    { // 数据+CRC
                        uint16_t crc = crc16(buffer, expected_length);
                        uint16_t received_crc = *(uint16_t *)(buffer + expected_length);

                        if (crc == received_crc)
                        {
                            if (g_nlt_nodeframe2_UnpackData(buffer, expected_length))
                            {
                                const nlt_nodeframe2_result_t *res = &g_nlt_nodeframe2_result;

                                // 打印解析结果（参考用户原始代码）
                                printf("System ID: %d, Role: %d\n", res->id, res->role);
                                printf("Local time: %u ms, System time: %u ms\n",
                                       res->local_time, res->system_time);
                                printf("Voltage: %.2f V\n", res->voltage);

                                for (int i = 0; i < res->valid_node_count; ++i)
                                {
                                    printf("Node %d: RSSI=%d\n",
                                           i, res->nodes[i].rx_rssi);
                                }
                            }
                        }

                        // 移动缓冲区
                        size_t processed = expected_length + 2;
                        memmove(buffer, buffer + processed, buffer_len - processed);
                        buffer_len -= processed;
                        state = STATE_HEADER_1;
                    }
                    break;
                }
            }
        }
    }

    CloseHandle(hSerial);
    return 0;
}