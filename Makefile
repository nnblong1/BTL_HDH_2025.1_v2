# Makefile for Manual Unicam DMA Driver
# Raspberry Pi 4B - BCM2711


# Object files
obj-m += manual-unicam-driver.o
manual-unicam-driver-objs := src/manual-unicam-driver.o

# Kernel source directory
# Default to currently running kernel
KERNELDIR ?= /lib/modules/$(shell uname -r)/build

# Architecture
ARCH ?= arm64
CROSS_COMPILE ?=

# Current directory
PWD := $(shell pwd)

# Build flags
ccflags-y := -DDEBUG -Wall -Wextra

# Default target: build kernel module
all: module overlay

# Build kernel module
module:
	@echo "Building manual-unicam kernel module..."
	$(MAKE) -C $(KERNELDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

# Compile device tree overlay
overlay: manual-unicam-overlay.dts
	@echo "Compiling device tree overlay..."
	dtc -@ -I dts -O dtb -o manual-unicam.dtbo manual-unicam-overlay.dts

# Clean build artifacts
clean:
	@echo "Cleaning build artifacts..."
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean
	rm -f manual-unicam.dtbo
	rm -f *.o *.ko *.mod.c *.mod *.order *.symvers
	rm -rf .tmp_versions

# Install module and overlay
install: all
	@echo "Installing kernel module..."
	sudo cp $(MODULE_NAME).ko /lib/modules/$(shell uname -r)/kernel/drivers/media/platform/
	sudo depmod -a
	@echo "Installing device tree overlay..."
	sudo cp manual-unicam.dtbo /boot/overlays/
	@echo ""
	@echo "Installation complete!"
	@echo "Add 'dtoverlay=manual-unicam' to /boot/config.txt and reboot"

# Uninstall module and overlay
uninstall:
	@echo "Uninstalling kernel module..."
	sudo rm -f /lib/modules/$(shell uname -r)/kernel/drivers/media/platform/$(MODULE_NAME).ko
	sudo depmod -a
	@echo "Uninstalling device tree overlay..."
	sudo rm -f /boot/overlays/manual-unicam.dtbo
	@echo "Uninstallation complete!"

# Load module manually (for testing)
load:
	@echo "Loading manual-unicam module..."
	sudo insmod $(MODULE_NAME).ko
	@echo "Module loaded. Check dmesg for output."

# Unload module
unload:
	@echo "Unloading manual-unicam module..."
	sudo rmmod $(MODULE_NAME)
	@echo "Module unloaded."

# Show kernel log (for debugging)
log:
	dmesg | tail -50 | grep -i unicam

# Check module info
info:
	modinfo $(MODULE_NAME).ko

# Code formatting (requires clang-format)
format:
	@echo "Formatting source code..."
	clang-format -i *.c *.h

# Static analysis (requires sparse)
check:
	@echo "Running static analysis..."
	$(MAKE) -C $(KERNELDIR) M=$(PWD) C=2 CF="-D__CHECK_ENDIAN__"

# Help target
help:
	@echo "Manual Unicam Driver Build System"
	@echo ""
	@echo "Targets:"
	@echo "  all        - Build module and overlay (default)"
	@echo "  module     - Build kernel module only"
	@echo "  overlay    - Compile device tree overlay"
	@echo "  clean      - Remove build artifacts"
	@echo "  install    - Install module and overlay to system"
	@echo "  uninstall  - Remove module and overlay from system"
	@echo "  load       - Load module (for testing)"
	@echo "  unload     - Unload module"
	@echo "  log        - Show recent kernel log entries"
	@echo "  info       - Display module information"
	@echo "  format     - Format source code"
	@echo "  check      - Run static analysis"
	@echo ""
	@echo "Variables:"
	@echo "  KERNELDIR  - Kernel source directory"
	@echo "  ARCH       - Target architecture (default: arm64)"
	@echo "  CROSS_COMPILE - Cross-compiler prefix"

.PHONY: all module overlay clean install uninstall load unload log info format check help