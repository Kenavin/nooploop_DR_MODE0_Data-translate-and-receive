CC = gcc
CFLAGS = -g -Wall
TARGET = sjjc_imu_linux

SRCS = sjjc_imu_linux.c \
       nlink_linktrack_nodeframe0.c \
	   nlink_linktrack_nodeframe2.c \
       nlink_utils.c

OBJS = $(SRCS:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f *.o $(TARGET)
