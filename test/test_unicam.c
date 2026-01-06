/*
 * User-space Test Application for Manual Unicam Driver
 * 
 * This program demonstrates how to:
 * - Open the driver device
 * - Memory map the DMA buffer
 * - Start/stop streaming
 * - Read frame data
 * - Save frames to disk
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <signal.h>
#include <time.h>

/* IOCTL Commands - must match driver definitions */
#define UNICAM_IOC_MAGIC 'U'
#define UNICAM_IOC_START_STREAM  _IO(UNICAM_IOC_MAGIC, 1)
#define UNICAM_IOC_STOP_STREAM   _IO(UNICAM_IOC_MAGIC, 2)
#define UNICAM_IOC_GET_BUFFER    _IOR(UNICAM_IOC_MAGIC, 3, unsigned long)

/* Frame configuration */
#define FRAME_WIDTH  640
#define FRAME_HEIGHT 480
#define BYTES_PER_PIXEL 2  /* RAW10 unpacked to 16-bit */
#define FRAME_SIZE (FRAME_WIDTH * FRAME_HEIGHT * BYTES_PER_PIXEL)

/* Global variables */
static volatile int running = 1;
static int fd = -1;
static void *mapped_buffer = NULL;
static size_t buffer_size = 0;

/*
 * Signal handler for clean shutdown
 */
void signal_handler(int signo)
{
	if (signo == SIGINT || signo == SIGTERM) {
		printf("\nReceived signal %d, stopping...\n", signo);
		running = 0;
	}
}

/*
 * Get current timestamp in milliseconds
 */
uint64_t get_timestamp_ms(void)
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (uint64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

/*
 * Save frame to PGM file (Portable Gray Map)
 * RAW10 data is saved as 16-bit grayscale
 */
int save_frame_pgm(const char *filename, const uint16_t *data)
{
	FILE *fp = fopen(filename, "wb");
	if (!fp) {
		perror("Failed to open file");
		return -1;
	}
	
	/* Write PGM header */
	fprintf(fp, "P5\n%d %d\n65535\n", FRAME_WIDTH, FRAME_HEIGHT);
	
	/* Write pixel data (convert to big-endian for PGM) */
	for (int i = 0; i < FRAME_WIDTH * FRAME_HEIGHT; i++) {
		uint16_t pixel = data[i];
		/* Swap bytes for big-endian */
		uint8_t bytes[2] = {pixel >> 8, pixel & 0xff};
		fwrite(bytes, 1, 2, fp);
	}
	
	fclose(fp);
	return 0;
}

/*
 * Save frame to RAW file (just raw binary data)
 */
int save_frame_raw(const char *filename, const void *data, size_t size)
{
	FILE *fp = fopen(filename, "wb");
	if (!fp) {
		perror("Failed to open file");
		return -1;
	}
	
	size_t written = fwrite(data, 1, size, fp);
	fclose(fp);
	
	return (written == size) ? 0 : -1;
}

/*
 * Calculate simple statistics on frame data
 */
void analyze_frame(const uint16_t *data, int num_pixels)
{
	uint32_t sum = 0;
	uint16_t min = 65535, max = 0;
	
	for (int i = 0; i < num_pixels; i++) {
		uint16_t pixel = data[i];
		sum += pixel;
		if (pixel < min) min = pixel;
		if (pixel > max) max = pixel;
	}
	
	double average = (double)sum / num_pixels;
	
	printf("  Min: %5u  Max: %5u  Avg: %.1f\n", min, max, average);
}

/*
 * Main capture loop
 */
int capture_loop(int num_frames)
{
	uint32_t frame_info[2];
	uint32_t last_frame_count = 0;
	uint64_t start_time, current_time;
	int frames_captured = 0;
	int ret;
	
	printf("\nStarting capture of %d frames...\n", num_frames);
	start_time = get_timestamp_ms();
	
	while (running && frames_captured < num_frames) {
		/* Read frame information */
		ret = read(fd, frame_info, sizeof(frame_info));
		if (ret < 0) {
			perror("Failed to read frame info");
			return -1;
		}
		
		uint32_t frame_count = frame_info[0];
		uint32_t write_ptr = frame_info[1];
		
		/* Check if new frame arrived */
		if (frame_count > last_frame_count) {
			frames_captured++;
			
			/* Calculate frame offset in circular buffer */
			uint32_t frame_offset = (frame_count - 1) * FRAME_SIZE;
			frame_offset %= buffer_size;
			
			/* Get pointer to frame data in mapped memory */
			uint16_t *frame_data = (uint16_t *)((uint8_t *)mapped_buffer + frame_offset);
			
			current_time = get_timestamp_ms();
			double elapsed = (current_time - start_time) / 1000.0;
			double fps = frames_captured / elapsed;
			
			printf("Frame %4d: count=%u ptr=0x%08x offset=0x%08x (%.1f fps)\n",
			       frames_captured, frame_count, write_ptr, frame_offset, fps);
			
			/* Analyze frame data */
			analyze_frame(frame_data, FRAME_WIDTH * FRAME_HEIGHT);
			
			/* Save every 10th frame */
			if (frames_captured % 10 == 0) {
				char filename[256];
				snprintf(filename, sizeof(filename), 
					"frame_%04d.pgm", frames_captured);
				
				printf("  Saving to %s\n", filename);
				if (save_frame_pgm(filename, frame_data) < 0) {
					fprintf(stderr, "Failed to save frame\n");
				}
			}
			
			last_frame_count = frame_count;
		}
		
		/* Small sleep to avoid busy waiting */
		usleep(1000); /* 1ms */
	}
	
	current_time = get_timestamp_ms();
	double total_time = (current_time - start_time) / 1000.0;
	double avg_fps = frames_captured / total_time;
	
	printf("\nCapture complete:\n");
	printf("  Frames: %d\n", frames_captured);
	printf("  Time: %.2f seconds\n", total_time);
	printf("  Average FPS: %.1f\n", avg_fps);
	
	return 0;
}

/*
 * Test continuous streaming
 */
int test_streaming(int duration_sec)
{
	uint32_t frame_info[2];
	uint32_t last_frame_count = 0;
	uint64_t start_time = get_timestamp_ms();
	uint64_t end_time = start_time + duration_sec * 1000;
	int frame_count = 0;
	
	printf("\nTesting streaming for %d seconds...\n", duration_sec);
	
	while (running && get_timestamp_ms() < end_time) {
		if (read(fd, frame_info, sizeof(frame_info)) < 0) {
			perror("Failed to read frame info");
			return -1;
		}
		
		if (frame_info[0] > last_frame_count) {
			frame_count++;
			last_frame_count = frame_info[0];
			
			if (frame_count % 100 == 0) {
				double elapsed = (get_timestamp_ms() - start_time) / 1000.0;
				double fps = frame_count / elapsed;
				printf("  %d frames (%.1f fps)\n", frame_count, fps);
			}
		}
		
		usleep(100);
	}
	
	double total_time = (get_timestamp_ms() - start_time) / 1000.0;
	double avg_fps = frame_count / total_time;
	
	printf("\nStreaming test complete:\n");
	printf("  Total frames: %d\n", frame_count);
	printf("  Average FPS: %.1f\n", avg_fps);
	
	return 0;
}

/*
 * Print usage information
 */
void print_usage(const char *prog_name)
{
	printf("Usage: %s [OPTIONS]\n", prog_name);
	printf("\nOptions:\n");
	printf("  -d DEVICE    Device path (default: /dev/unicam0)\n");
	printf("  -n FRAMES    Number of frames to capture (default: 100)\n");
	printf("  -t SECONDS   Duration to test streaming (default: off)\n");
	printf("  -s           Save frames to disk\n");
	printf("  -h           Show this help message\n");
	printf("\nExamples:\n");
	printf("  %s -n 50              # Capture 50 frames\n", prog_name);
	printf("  %s -t 10              # Test streaming for 10 seconds\n", prog_name);
	printf("  %s -n 100 -s          # Capture 100 frames and save\n", prog_name);
}

/*
 * Main function
 */
int main(int argc, char *argv[])
{
	const char *device = "/dev/unicam0";
	int num_frames = 100;
	int test_duration = 0;
	int save_frames = 0;
	int opt;
	int ret = 0;
	
	/* Parse command line arguments */
	while ((opt = getopt(argc, argv, "d:n:t:sh")) != -1) {
		switch (opt) {
		case 'd':
			device = optarg;
			break;
		case 'n':
			num_frames = atoi(optarg);
			break;
		case 't':
			test_duration = atoi(optarg);
			break;
		case 's':
			save_frames = 1;
			break;
		case 'h':
			print_usage(argv[0]);
			return 0;
		default:
			print_usage(argv[0]);
			return 1;
		}
	}
	
	/* Setup signal handlers */
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	
	printf("Manual Unicam Test Application\n");
	printf("================================\n");
	printf("Device: %s\n", device);
	
	/* Open device */
	fd = open(device, O_RDWR);
	if (fd < 0) {
		perror("Failed to open device");
		fprintf(stderr, "Make sure:\n");
		fprintf(stderr, "  1. Driver is loaded (lsmod | grep unicam)\n");
		fprintf(stderr, "  2. Device node exists (ls -l /dev/unicam0)\n");
		fprintf(stderr, "  3. You have permissions (sudo)\n");
		return 1;
	}
	
	printf("Device opened successfully\n");
	
	/* Get buffer size */
	if (ioctl(fd, UNICAM_IOC_GET_BUFFER, &buffer_size) < 0) {
		perror("Failed to get buffer size");
		ret = 1;
		goto cleanup;
	}
	
	printf("DMA buffer size: %zu bytes (%zu MB)\n", 
	       buffer_size, buffer_size / (1024 * 1024));
	printf("Max frames in buffer: %zu\n", buffer_size / FRAME_SIZE);
	
	/* Memory map the DMA buffer */
	mapped_buffer = mmap(NULL, buffer_size, PROT_READ | PROT_WRITE,
			     MAP_SHARED, fd, 0);
	if (mapped_buffer == MAP_FAILED) {
		perror("Failed to mmap buffer");
		ret = 1;
		goto cleanup;
	}
	
	printf("Buffer mapped at %p\n", mapped_buffer);
	
	/* Start streaming */
	printf("\nStarting stream...\n");
	if (ioctl(fd, UNICAM_IOC_START_STREAM) < 0) {
		perror("Failed to start streaming");
		ret = 1;
		goto cleanup_mmap;
	}
	
	printf("Stream started!\n");
	
	/* Wait a moment for sensor to stabilize */
	sleep(1);
	
	/* Run test */
	if (test_duration > 0) {
		ret = test_streaming(test_duration);
	} else {
		ret = capture_loop(num_frames);
	}
	
	/* Stop streaming */
	printf("\nStopping stream...\n");
	if (ioctl(fd, UNICAM_IOC_STOP_STREAM) < 0) {
		perror("Failed to stop streaming");
	}
	
	printf("Stream stopped\n");

cleanup_mmap:
	if (mapped_buffer != MAP_FAILED) {
		munmap(mapped_buffer, buffer_size);
	}

cleanup:
	if (fd >= 0) {
		close(fd);
	}
	
	printf("\nTest complete!\n");
	return ret;
}