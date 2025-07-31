#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <errno.h>

#include "nlink_linktrack_nodeframe2.h"
#include "nlink_linktrack_nodeframe0.h"
#include "nlink_utils.h"

#undef NLINK_PROTOCOL_LENGTH
#define NLINK_PROTOCOL_LENGTH(X) ((size_t)(((uint8_t *)(X))[2] | ((uint8_t *)(X))[3] << 8))

#define BUFFER_SIZE 409600

// 打开串口
int open_serial_port(const char *port_name, int baud_rate)
{
    int fd = open(port_name, O_RDWR | O_NOCTTY);
    if (fd == -1)
    {
        perror("Error opening serial port");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);

    // 设置波特率
    speed_t speed;
    switch (baud_rate)
    {
    case 921600:
        speed = B921600;
        break;
    case 115200:
        speed = B115200;
        break;
    case 9600:
        speed = B9600;
        break;
    default:
        printf("Unsupported baud rate!\n");
        close(fd);
        return -1;
    }
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= CREAD | CLOCAL;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;

    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &options) != 0)
    {
        perror("Error setting serial port attributes");
        close(fd);
        return -1;
    }

    return fd;
}

// 获取当前时间戳（毫秒）
unsigned long get_millis()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000UL) + (tv.tv_usec / 1000UL);
}

int main()
{
    int fd = open_serial_port("/dev/ttyCH343USB1", 921600);
    if (fd == -1)
    {
        return EXIT_FAILURE;
    }

    uint8_t recv_buffer[BUFFER_SIZE];
    size_t recv_len = 0;

    printf("Start receiving NodeFrame data from /dev/ttyCH343USB0...\n");

    unsigned long last_time = get_millis();
    int frame_count = 0;

    while (1)
    {
        ssize_t bytes_read = read(fd, recv_buffer + recv_len, BUFFER_SIZE - recv_len);
        
        if (bytes_read > 0)
        {
            recv_len += bytes_read;
            if (recv_len >= BUFFER_SIZE)
            {
                printf("⚠️ 警告：串口缓冲区满，强制清空\n");
                recv_len = 0;  // 丢弃所有数据，重新开始
            }

            size_t i = 0;
            while (i + 4 < recv_len)
            {
                if (recv_buffer[i] == 0x55)
                {
                    uint8_t function_mark = recv_buffer[i + 1];
                    uint16_t length = recv_buffer[i + 2] | (recv_buffer[i + 3] << 8);
                    size_t total_frame_len = 4 + length;

                    if (i + total_frame_len <= recv_len)
                    {
                        uint8_t *frame_data = &recv_buffer[i];
                        int unpacked = 0;

                        if (function_mark == g_nlt_nodeframe2.function_mark)
                        {
                            if (g_nlt_nodeframe2.UnpackData(frame_data, total_frame_len))
                            {
                                nlt_nodeframe2_result_t *result = &g_nlt_nodeframe2.result;
                                // printf("\n==== NodeFrame2 Unpack Success ====\n");
                                // printf("System Time: %u ms\n", result->system_time);
                                // printf("Local Time: %u ms\n", result->local_time);
                                // printf("ID: %d, Role: %d\n", result->id, result->role);
                                // printf("Voltage: %.2f V\n", result->voltage);
                                // printf("Valid nodes: %d\n", result->valid_node_count);

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
                                frame_count++;
                                unpacked = 1;
                            }
                            else
                            {
                                printf("NodeFrame2 unpack error (checksum fail?)\n");
                            }
                        }
                        else if (function_mark == g_nlt_nodeframe0.function_mark)
                        {
                            if (g_nlt_nodeframe0.UnpackData(frame_data, total_frame_len))
                            {
                                nlt_nodeframe0_result_t *result = &g_nlt_nodeframe0.result;
                                printf("\n==== NodeFrame0 Unpack Success ====\n");
                                printf("ID: %d, Role: %d\n", result->id, result->role);
                                printf("Valid nodes: %d\n", result->valid_node_count);

                                for (int j = 0; j < result->valid_node_count; ++j)
                                {
                                    nlt_nodeframe0_node_t *node = result->nodes[j];
                                    if (node)
                                    {
                                        printf("Node[%d]: role=%d id=%d data_len=%d\n",
                                               j, node->role, node->id, node->data_length);
                                    }
                                }
                                printf("======================================\n");
                                frame_count++;
                                unpacked = 1;
                            }
                            else
                            {
                                printf("NodeFrame0 unpack error (checksum fail?)\n");
                            }
                        }

                        if (!unpacked)
                        {
                            printf("Unknown or invalid frame detected.\n");
                        }

                        i += total_frame_len;
                    }
                    else
                    {
                        break;
                    }
                }
                else
                {
                    ++i;
                }
            }

            if (i > 0)
            {
                memmove(recv_buffer, recv_buffer + i, recv_len - i);
                recv_len -= i;
            }
        }
        else if (bytes_read == 0)
        {
            usleep(1000); // 1ms delay
        }
        else
        {
            perror("Error reading from serial port");
            break;
        }

        unsigned long now = get_millis();
        if (now - last_time >= 1000)
        {
            printf("\n*** Current FPS: %d frames/sec ***\n", frame_count);
            frame_count = 0;
            last_time = now;
        }
    }

    close(fd);
    return EXIT_SUCCESS;
}