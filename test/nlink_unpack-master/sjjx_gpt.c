#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <windows.h>

#include "nlink_linktrack_nodeframe2.h"
#include "nlink_utils.h"

#undef NLINK_PROTOCOL_LENGTH
#define NLINK_PROTOCOL_LENGTH(X) ((size_t)(((uint8_t *)(X))[2] | ((uint8_t *)(X))[3] << 8))

// 打开串口
HANDLE open_serial_port(const char *port_name, DWORD baud_rate)
{
  HANDLE hSerial = CreateFile(port_name,
                              GENERIC_READ | GENERIC_WRITE,
                              0,
                              0,
                              OPEN_EXISTING,
                              0,
                              0);
  if (hSerial == INVALID_HANDLE_VALUE)
  {
    printf("Error opening serial port %s\n", port_name);
    return INVALID_HANDLE_VALUE;
  }

  DCB dcbSerialParams = {0};
  dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

  if (!GetCommState(hSerial, &dcbSerialParams))
  {
    printf("Error getting serial state\n");
    CloseHandle(hSerial);
    return INVALID_HANDLE_VALUE;
  }

  dcbSerialParams.BaudRate = baud_rate;
  dcbSerialParams.ByteSize = 8;
  dcbSerialParams.StopBits = ONESTOPBIT;
  dcbSerialParams.Parity = NOPARITY;
  dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE;

  if (!SetCommState(hSerial, &dcbSerialParams))
  {
    printf("Error setting serial state\n");
    CloseHandle(hSerial);
    return INVALID_HANDLE_VALUE;
  }

  COMMTIMEOUTS timeouts = {0};
  timeouts.ReadIntervalTimeout = 50;
  timeouts.ReadTotalTimeoutConstant = 50;
  timeouts.ReadTotalTimeoutMultiplier = 10;

  SetCommTimeouts(hSerial, &timeouts);

  return hSerial;
}

#define BUFFER_SIZE 4096

int main()
{
  // 打开COM8，波特率921600
  HANDLE hSerial = open_serial_port("\\\\.\\COM12", 921600);
  if (hSerial == INVALID_HANDLE_VALUE)
  {
    return EXIT_FAILURE;
  }

  uint8_t recv_buffer[BUFFER_SIZE];
  size_t recv_len = 0;

  printf("Start receiving NodeFrame2 data from COM8...\n");

  // ====== 帧率统计相关 ======
  DWORD last_time = GetTickCount();
  int frame_count = 0;
  // ========================

  while (1)
  {
    DWORD bytes_read = 0;
    if (ReadFile(hSerial, recv_buffer + recv_len, BUFFER_SIZE - recv_len, &bytes_read, NULL))
    {
      recv_len += bytes_read;

      // 尝试从接收缓冲区中提取完整帧
      size_t i = 0;
      while (i + 4 < recv_len)
      {
        if (recv_buffer[i] == g_nlt_nodeframe2.frame_header &&
            recv_buffer[i + 1] == g_nlt_nodeframe2.function_mark)
        {
          uint16_t length = recv_buffer[i + 2] | (recv_buffer[i + 3] << 8);
          size_t total_frame_len = 4 + length; // 帧头4字节 + payload

          if (i + total_frame_len <= recv_len)
          {
            uint8_t *frame_data = &recv_buffer[i];

            if (g_nlt_nodeframe2.UnpackData(frame_data, total_frame_len))
            {
              // 成功解包
              nlt_nodeframe2_result_t *result = &g_nlt_nodeframe2.result;
              printf("\n==== NodeFrame2 Unpack Success ====\n");
              printf("System Time: %u ms\n", result->system_time);
              printf("Local Time: %u ms\n", result->local_time);
              printf("ID: %d, Role: %d\n", result->id, result->role);
              printf("Voltage: %.2f V\n", result->voltage);
              // printf("Position: x=%.3f m, y=%.3f m, z=%.3f m\n",
              //        result->pos_3d[0], result->pos_3d[1], result->pos_3d[2]);
              // printf("Velocity: x=%.3f m/s, y=%.3f m/s, z=%.3f m/s\n",
              //        result->vel_3d[0], result->vel_3d[1], result->vel_3d[2]);
              // printf("IMU Gyro: x=%.3f, y=%.3f, z=%.3f\n",
              //        result->imu_gyro_3d[0], result->imu_gyro_3d[1], result->imu_gyro_3d[2]);
              // printf("IMU Acc: x=%.3f, y=%.3f, z=%.3f\n",
              //        result->imu_acc_3d[0], result->imu_acc_3d[1], result->imu_acc_3d[2]);
              // printf("Quaternion: [%.3f, %.3f, %.3f, %.3f]\n",
              //        result->quaternion[0], result->quaternion[1], result->quaternion[2], result->quaternion[3]);
              // printf("Angle (deg): x=%.2f, y=%.2f, z=%.2f\n",
              //        result->angle_3d[0], result->angle_3d[1], result->angle_3d[2]);
              printf("Valid nodes: %d\n", result->valid_node_count);

              for (int j = 0; j < result->valid_node_count; ++j)
              {
                nlt_nodeframe2_node_t *node = result->nodes[j];
                if (node)
                {
                  printf("Node[%d]: role=%d id=%d dis=%.3f m, fp_rssi=%.1f dB, rx_rssi=%.1f dB\n",
                         j, node->role, node->id, node->dis,
                         node->fp_rssi, node->rx_rssi);
                }
              }
              printf("======================================\n");

              // ====== 帧率统计：成功解包 +1 ======
              frame_count++;
              // ==================================
            }
            else
            {
              printf("NodeFrame2 unpack error (checksum fail?)\n");
            }

            // 处理完这一包
            i += total_frame_len;
          }
          else
          {
            // 包还没接完整
            break;
          }
        }
        else
        {
          // 不是包头，跳过
          ++i;
        }
      }

      // 清理已处理的数据
      if (i > 0)
      {
        memmove(recv_buffer, recv_buffer + i, recv_len - i);
        recv_len -= i;
      }
    }
    else
    {
      printf("Error reading from serial port\n");
      break;
    }

    // ====== 每秒输出一次帧率 ======
    DWORD now = GetTickCount();
    if (now - last_time >= 1000)
    {
      printf("\n*** Current FPS: %d frames/sec ***\n", frame_count);
      frame_count = 0;
      last_time = now;
    }
    // ==================================

    // Sleep(1); // 轻轻睡眠，降低CPU占用
  }

  CloseHandle(hSerial);
  return EXIT_SUCCESS;
}
