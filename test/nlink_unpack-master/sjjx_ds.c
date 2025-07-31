#include <stdio.h>
#include <stdint.h>
#include <windows.h>
#include "nlink_linktrack_nodeframe2.h"

#define BUFFER_SIZE 1024
#define MAX_PACKET_SIZE 256

// Windows非标波特率设置（需管理员权限）
BOOL SetCustomBaudRate(HANDLE hCom, DWORD baudRate)
{
    typedef BOOL(WINAPI * DCB_FUNC)(HANDLE, DWORD, LPDCB);
    static DCB_FUNC func = NULL;
    static BOOL initialized = FALSE;

    if (!initialized)
    {
        HMODULE hLib = LoadLibrary("kernel32.dll");
        func = (DCB_FUNC)GetProcAddress(hLib, "SetCommState");
        initialized = TRUE;
    }

    DCB dcb;
    if (!GetCommState(hCom, &dcb))
        return FALSE;
    dcb.BaudRate = baudRate; // 直接设置数值波特率
    return func(hCom, &dcb);
}

int main()
{
    HANDLE hSerial;
    char portName[] = "COM6"; // 修改为实际串口号

    // 打开串口
    hSerial = CreateFileA(portName, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                          OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (hSerial == INVALID_HANDLE_VALUE)
    {
        printf("打开串口失败！错误码: %d\n", GetLastError());
        return 1;
    }

    // 配置串口参数
    DCB dcb = {0};
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(hSerial, &dcb))
    {
        printf("获取串口参数失败！错误码: %d\n", GetLastError());
        CloseHandle(hSerial);
        return 1;
    }

    // 设置非标波特率921600（需要管理员权限）
    if (!SetCustomBaudRate(hSerial, 921600))
    {
        printf("警告：非标波特率设置失败，尝试标准波特率...\n");
        dcb.BaudRate = CBR_115200; // 回退到标准波特率
    }

    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;
    dcb.fOutxCtsFlow = FALSE;              // 禁用CTS流控
    dcb.fRtsControl = RTS_CONTROL_DISABLE; // 禁用RTS

    if (!SetCommState(hSerial, &dcb))
    {
        printf("配置串口参数失败！错误码: %d\n", GetLastError());
        CloseHandle(hSerial);
        return 1;
    }

    // 设置超时（立即返回模式）
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = MAXDWORD; // 非阻塞模式
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
    SetCommTimeouts(hSerial, &timeouts);

    // 初始化接收缓冲区
    uint8_t buffer[BUFFER_SIZE] = {0};
    size_t buffer_len = 0;

    printf("开始读取串口数据...\n");

    while (1)
    {
        DWORD bytesRead = 0;
        uint8_t chunk[MAX_PACKET_SIZE] = {0};

        // 批量读取数据
        if (ReadFile(hSerial, chunk, sizeof(chunk), &bytesRead, NULL) && bytesRead > 0)
        {
            // 调试输出原始数据
            printf("收到原始数据(%d字节): ", bytesRead);
            for (DWORD i = 0; i < bytesRead; ++i)
            {
                printf("%02X ", chunk[i]);
            }
            printf("\n");

            // 将新数据追加到缓冲区
            if (buffer_len + bytesRead > BUFFER_SIZE)
            {
                printf("缓冲区溢出，重置缓冲区\n");
                buffer_len = 0;
            }
            memcpy(buffer + buffer_len, chunk, bytesRead);
            buffer_len += bytesRead;

            // 协议解析循环
            while (buffer_len >= 2)
            { // 至少需要包头长度
                // 查找包头（示例协议头：0x55 0xAA）
                int pkt_start = -1;
                for (size_t i = 0; i < buffer_len - 1; ++i)
                {
                    if (buffer[i] == 0x55 && buffer[i + 1] == 0xAA)
                    { // 修改为实际协议头
                        pkt_start = i;
                        break;
                    }
                }

                if (pkt_start == -1)
                { // 未找到包头
                    buffer_len = 0;
                    break;
                }

                if (pkt_start > 0)
                { // 丢弃包头前的无效数据
                    printf("丢弃 %d 字节无效数据\n", pkt_start);
                    memmove(buffer, buffer + pkt_start, buffer_len - pkt_start);
                    buffer_len -= pkt_start;
                }

                // 检查数据包长度（假设第3字节为长度字段）
                if (buffer_len < 3)
                    break;
                uint8_t pkt_length = buffer[2]; // 修改为实际长度字段位置

                if (pkt_length < 5 || pkt_length > MAX_PACKET_SIZE)
                { // 长度校验
                    printf("非法数据包长度 %d，丢弃包头\n", pkt_length);
                    memmove(buffer, buffer + 2, buffer_len - 2);
                    buffer_len -= 2;
                    continue;
                }

                if (buffer_len < pkt_length)
                    break; // 数据不完整

                // 调用解包函数
                if (g_nlt_nodeframe2.UnpackData(buffer, pkt_length))
                {
                    // 打印解包结果
                    printf("解包成功！ID: %d, 角色: %d, 节点数: %d\n",
                           g_nlt_nodeframe2.result.id,
                           g_nlt_nodeframe2.result.role,
                           g_nlt_nodeframe2.result.valid_node_count);
                }
                else
                {
                    printf("解包失败！\n");
                }

                // 移除已处理数据
                memmove(buffer, buffer + pkt_length, buffer_len - pkt_length);
                buffer_len -= pkt_length;
            }
        }
        else
        {
            // 错误处理
            DWORD err = GetLastError();
            if (err != ERROR_IO_PENDING)
            {
                printf("读取错误！错误码: %d\n", err);
                ClearCommError(hSerial, &err, NULL);
            }
        }
    }

    CloseHandle(hSerial);
    return 0;
}