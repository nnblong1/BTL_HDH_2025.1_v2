# BTL_HDH_2025.1_v2
# Manual Unicam DMA Driver - Hướng dẫn Cài đặt và Sử dụng

## Mục lục
1. [Yêu cầu Hệ thống](#yêu-cầu-hệ-thống)
2. [Chuẩn bị Môi trường](#chuẩn-bị-môi-trường)
3. [Biên dịch Driver](#biên-dịch-driver)
4. [Cài đặt](#cài-đặt)
5. [Cấu hình System](#cấu-hình-system)
6. [Kiểm tra và Debug](#kiểm-tra-và-debug)
7. [Sử dụng Driver](#sử-dụng-driver)
8. [Xử lý Sự cố](#xử-lý-sự-cố)

---

## Yêu cầu Hệ thống

### Phần cứng
- **Board**: Raspberry Pi 4B (BCM2711)
- **Camera**: Raspberry Pi Camera Module v2 (IMX219)
- **RAM**: Tối thiểu 2GB (khuyến nghị 4GB)
- **MicroSD**: Ít nhất 16GB

### Phần mềm
- **OS**: Raspberry Pi OS (64-bit) hoặc tương đương
- **Kernel**: 5.10+ (khuyến nghị 6.1+)
- **Compiler**: gcc, make, device-tree-compiler
- **Headers**: Linux kernel headers

---

## Chuẩn bị Môi trường

### 1. Cập nhật hệ thống

```bash
sudo apt update
sudo apt upgrade -y
```

### 2. Cài đặt công cụ phát triển

```bash
# Cài đặt kernel headers
sudo apt install -y raspberrypi-kernel-headers

# Cài đặt build tools
sudo apt install -y build-essential git device-tree-compiler

# Kiểm tra version kernel
uname -r
```

### 3. Kiểm tra phần cứng

```bash
# Kiểm tra camera được nhận diện
vcgencmd get_camera

# Kết quả mong đợi: supported=1 detected=1

# Kiểm tra I2C
sudo i2cdetect -y 10

# Nên thấy địa chỉ 0x10 (IMX219)
```

---

## Biên dịch Driver

### 1. Tạo cấu trúc thư mục

```bash
mkdir -p ~/manual-unicam-driver
cd ~/manual-unicam-driver
```

### 2. Copy các file source code

Tạo các file sau với nội dung từ artifacts:

```bash
# Tạo file headers
touch bcm2835-unicam-regs.h
touch imx219-regs.h

# Tạo file driver chính
touch manual-unicam-driver.c

# Tạo Device Tree Overlay
touch manual-unicam-overlay.dts

# Tạo Makefile
touch Makefile
```

**Chú ý**: Copy nội dung từ các artifacts đã tạo vào các file tương ứng.

### 3. Biên dịch

```bash
# Build tất cả
make

# Output mong đợi:
# - manual-unicam.ko (kernel module)
# - manual-unicam.dtbo (device tree overlay)
```

### 4. Kiểm tra module

```bash
# Xem thông tin module
modinfo manual-unicam.ko

# Kết quả hiển thị:
# - filename
# - description
# - author
# - license
# - depends
```

---

## Cài đặt

### 1. Cài đặt kernel module và overlay

```bash
# Sử dụng Makefile
sudo make install

# Hoặc cài đặt thủ công:
sudo cp manual-unicam.ko /lib/modules/$(uname -r)/kernel/drivers/media/platform/
sudo cp manual-unicam.dtbo /boot/overlays/
sudo depmod -a
```

### 2. Cấu hình boot

Chỉnh sửa `/boot/config.txt`:

```bash
sudo nano /boot/config.txt
```

Thêm các dòng sau:

```ini
# Tắt camera driver mặc định
camera_auto_detect=0

# Tắt standard V4L2 driver
dtoverlay=vc4-kms-v3d,noaudio
dtoverlay=vc4-fkms-v3d

# Bật custom driver
dtoverlay=manual-unicam

# Tăng CMA memory cho DMA
cma=256M

# Đảm bảo I2C được bật
dtparam=i2c_arm=on
```

### 3. Tạo device permissions

Tạo file `/etc/udev/rules.d/99-unicam.rules`:

```bash
sudo nano /etc/udev/rules.d/99-unicam.rules
```

Thêm nội dung:

```
SUBSYSTEM=="unicam", MODE="0666"
KERNEL=="unicam0", MODE="0666"
```

### 4. Reboot hệ thống

```bash
sudo reboot
```

---

## Cấu hình System

### 1. Kiểm tra CMA

```bash
# Xem CMA available
dmesg | grep -i cma

# Kết quả mong đợi:
# Memory: xxxM/xxxM available, xxxM reserved, xxxM cma-reserved
```

### 2. Điều chỉnh GPU memory (tùy chọn)

Trong `/boot/config.txt`:

```ini
# Tăng GPU memory nếu cần
gpu_mem=256
```

### 3. Optimize performance

```bash
# Disable unnecessary services
sudo systemctl disable bluetooth
sudo systemctl disable hciuart

# Tăng priority cho interrupt handler
echo -1 | sudo tee /proc/sys/kernel/sched_rt_runtime_us
```

---

## Kiểm tra và Debug

### 1. Kiểm tra module đã load

```bash
# Xem module loaded
lsmod | grep unicam

# Xem device node
ls -l /dev/unicam*

# Kết quả mong đợi:
# crw-rw-rw- 1 root root 244, 0 Jan  6 10:00 /dev/unicam0
```

### 2. Xem kernel log

```bash
# Xem log driver
dmesg | grep -i unicam

# Xem log realtime
sudo dmesg -w | grep -i unicam
```

### 3. Kiểm tra hardware resources

```bash
# Xem interrupt
cat /proc/interrupts | grep unicam

# Xem memory mapping
cat /proc/iomem | grep unicam
```

### 4. Debug mode

Để bật debug output, load module với parameter:

```bash
sudo rmmod manual-unicam
sudo insmod manual-unicam.ko debug=1
```

---

## Sử dụng Driver

### 1. Build test application

```bash
# Compile user-space test
gcc -o test_unicam test_unicam.c -Wall -O2

# Chạy với quyền sudo
sudo ./test_unicam -n 100
```

### 2. Các chế độ test

```bash
# Capture 50 frames
sudo ./test_unicam -n 50

# Test streaming 10 giây
sudo ./test_unicam -t 10

# Capture và save frames
sudo ./test_unicam -n 100 -s

# Chỉ định device khác
sudo ./test_unicam -d /dev/unicam0 -n 50
```

### 3. Sử dụng từ Python

```python
#!/usr/bin/env python3
import fcntl
import mmap
import struct
import numpy as np

# IOCTL commands
UNICAM_IOC_START_STREAM = 0x00005501
UNICAM_IOC_STOP_STREAM = 0x00005502
UNICAM_IOC_GET_BUFFER = 0x80085503

# Open device
fd = open('/dev/unicam0', 'r+b', buffering=0)

# Get buffer size
buffer_size = struct.unpack('Q', fcntl.ioctl(fd, UNICAM_IOC_GET_BUFFER, 
                                             struct.pack('Q', 0)))[0]
print(f"Buffer size: {buffer_size} bytes")

# Memory map
mm = mmap.mmap(fd.fileno(), buffer_size, mmap.MAP_SHARED, 
               mmap.PROT_READ | mmap.PROT_WRITE)

# Start streaming
fcntl.ioctl(fd, UNICAM_IOC_START_STREAM)

try:
    # Read frames
    for i in range(100):
        # Read frame info
        frame_data = fd.read(8)
        frame_count, write_ptr = struct.unpack('II', frame_data)
        
        # Calculate frame offset
        frame_size = 640 * 480 * 2
        offset = ((frame_count - 1) * frame_size) % buffer_size
        
        # Get frame from memory map
        mm.seek(offset)
        raw_data = mm.read(frame_size)
        
        # Convert to numpy array
        frame = np.frombuffer(raw_data, dtype=np.uint16)
        frame = frame.reshape((480, 640))
        
        print(f"Frame {i}: count={frame_count}, "
              f"mean={frame.mean():.1f}")
        
finally:
    # Stop streaming
    fcntl.ioctl(fd, UNICAM_IOC_STOP_STREAM)
    mm.close()
    fd.close()
```

---

## Xử lý Sự cố

### Vấn đề 1: Module không load được

**Triệu chứng**:
```
insmod: ERROR: could not insert module manual-unicam.ko: Invalid module format
```

**Giải pháp**:
```bash
# Rebuild với đúng kernel version
make clean
make KERNELDIR=/lib/modules/$(uname -r)/build

# Kiểm tra module info
modinfo manual-unicam.ko | grep vermagic
```

### Vấn đề 2: Device node không tạo được

**Triệu chứng**: `/dev/unicam0` không tồn tại

**Giải pháp**:
```bash
# Kiểm tra module loaded
lsmod | grep unicam

# Xem dmesg để tìm lỗi
dmesg | tail -50

# Tạo device node thủ công
MAJOR=$(awk '$2=="unicam0" {print $1}' /proc/devices)
sudo mknod /dev/unicam0 c $MAJOR 0
sudo chmod 666 /dev/unicam0
```

### Vấn đề 3: Camera không detect

**Triệu chứng**: "Failed to read sensor model ID"

**Giải pháp**:
```bash
# Kiểm tra camera connection
vcgencmd get_camera

# Kiểm tra I2C
sudo i2cdetect -y 10

# Kiểm tra ribbon cable
# - Đảm bảo cable cắm đúng hướng
# - Contacts màu xanh hướng về phía Ethernet port

# Test I2C communication
sudo i2cget -y 10 0x10 0x00 w

# Kết quả mong đợi: 0x1902 (model ID của IMX219)
```

### Vấn đề 4: DMA allocation failed

**Triệu chứng**: "Failed to allocate DMA buffer"

**Giải pháp**:
```bash
# Tăng CMA trong /boot/config.txt
cma=512M

# Hoặc thêm vào cmdline.txt
sudo nano /boot/cmdline.txt
# Thêm: cma=512M

# Reboot
sudo reboot

# Kiểm tra CMA
dmesg | grep cma
```

### Vấn đề 5: Frame capture không hoạt động

**Triệu chứng**: Frame count không tăng

**Giải pháp**:
```bash
# Kiểm tra interrupt
cat /proc/interrupts | grep unicam

# Interrupt count phải tăng khi streaming

# Kiểm tra clock
vcgencmd measure_clock arm
vcgencmd measure_clock core

# VPU clock phải >= 250MHz

# Enable debug log
echo 8 | sudo tee /proc/sys/kernel/printk
dmesg -w | grep unicam
```

### Vấn đề 6: Performance thấp

**Giải pháp**:
```bash
# Tắt unnecessary services
sudo systemctl disable bluetooth
sudo systemctl disable wifi

# Tăng CPU governor
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Disable CPU throttling
sudo vcgencmd force_turbo 1

# Kiểm tra temperature
vcgencmd measure_temp
```

---

## Các lệnh hữu ích

```bash
# Load driver
sudo modprobe manual-unicam

# Unload driver
sudo modprobe -r manual-unicam

# View module parameters
systool -v -m manual_unicam

# Monitor interrupt rate
watch -n 1 'cat /proc/interrupts | grep unicam'

# Monitor memory usage
watch -n 1 'free -m'

# Capture kernel trace
sudo trace-cmd record -e unicam
sudo trace-cmd report

# Performance profiling
sudo perf record -g ./test_unicam -n 1000
sudo perf report
```

---

## Tài liệu tham khảo

- BCM2711 ARM Peripherals: https://datasheets.raspberrypi.com/bcm2711/bcm2711-peripherals.pdf
- IMX219 Datasheet: https://www.sony-semicon.co.jp/products/common/pdf/IMX219PQ_ProductBrief.pdf
- Linux Driver Development: https://www.kernel.org/doc/html/latest/driver-api/
- V4L2 API: https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/
- DMA API: https://www.kernel.org/doc/html/latest/core-api/dma-api.html

---

## Liên hệ và Hỗ trợ

Nếu gặp vấn đề, vui lòng:

1. Kiểm tra kernel log: `dmesg | grep unicam`
2. Xem system log: `journalctl -xe`
3. Tạo bug report với đầy đủ log và thông tin hệ thống

---

**Copyright © 2026 - Manual Unicam DMA Driver Project**