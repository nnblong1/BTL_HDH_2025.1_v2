# Makefile for Virtual Unicam Driver

obj-m := unicam-virtual-sensor.o

KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

all: driver test

driver:
	make -C $(KDIR) M=$(PWD) modules

test: test-virtual.c
	gcc -o test test.c -Wall

clean:
	make -C $(KDIR) M=$(PWD) clean
	rm -f test-virtual *.pgm

insmod:
	sudo insmod unicam-virtual-sensor.ko
	
rmmod:
	sudo rmmod unicam-virtual-sensor

dmesg:
	dmesg | tail 

log:
	dmesg -w | grep -E "virtual-unicam|unicam"
	watch:
	sudo dmesg -w | grep -E "standalone|unicam"

# Check if module is loaded
status:
	@echo "Module status:"
	@lsmod | grep -E "unicam|standalone" || echo "Module not loaded"
	@echo ""
	@echo "Device status:"
	@ls -l /dev/unicam* 2>/dev/null || echo "No device found"

# Run test program
run-test:
	@if [ ! -f test-virtual ]; then \
		echo "Building test program first..."; \
		make test; \
	fi
	@echo "Running test (requires root)..."
	sudo ./test-virtual

# Quick test: build, load, test, unload
quick-test: clean module install
	@sleep 1
	@make log
	@echo ""
	@echo "Press Enter to run test..."
	@read dummy
	@make run-test
	@echo ""
	@echo "Test complete. Unloading module..."
	@make uninstall

.PHONY: all module test clean install uninstall reinstall log watch status run-test quick-test