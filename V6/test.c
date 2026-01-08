/*
 * Test program for Virtual Unicam Driver (CPU Optimized)
 * Low CPU usage version for capturing many frames
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <errno.h>
#include <poll.h>
#include <signal.h>

#define UNICAM_IOC_MAGIC 'U'
#define UNICAM_IOC_START_STREAM  _IO(UNICAM_IOC_MAGIC, 1)
#define UNICAM_IOC_STOP_STREAM   _IO(UNICAM_IOC_MAGIC, 2)
#define UNICAM_IOC_GET_BUFFER    _IOR(UNICAM_IOC_MAGIC, 3, unsigned long)

#define FRAME_WIDTH  640
#define FRAME_HEIGHT 480
#define FRAME_SIZE   (FRAME_WIDTH * FRAME_HEIGHT * 2)

/* CPU optimization modes */
#define MODE_FULL_ANALYSIS 0    /* Full brightness + change detection */
#define MODE_SAMPLE_ONLY   1    /* Sample-based quick check */
#define MODE_MINIMAL       2    /* No analysis, just count frames */

/* Timeout constants */
#define POLL_TIMEOUT_MS 5000
#define MAX_RETRIES 3

static volatile int running = 1;

/* Signal handler for clean exit */
void signal_handler(int signum)
{
	printf("\n[!] Received signal %d, stopping...\n", signum);
	running = 0;
}

/* Save frame as PGM image (10-bit data scaled to 8-bit) */
void save_frame_as_pgm(const char *filename, uint16_t *data, int width, int height)
{
	FILE *fp;
	int x, y;
	
	fp = fopen(filename, "wb");
	if (!fp) {
		perror("fopen");
		return;
	}
	
	fprintf(fp, "P5\n%d %d\n255\n", width, height);
	
	for (y = 0; y < height; y++) {
		for (x = 0; x < width; x++) {
			uint16_t pixel = data[y * width + x];
			uint8_t pixel_8bit = (pixel >> 2) & 0xFF;
			fwrite(&pixel_8bit, 1, 1, fp);
		}
	}
	
	fclose(fp);
	printf("✓ Saved: %s\n", filename);
}

/* FAST: Sample-based brightness (only check a few pixels) */
uint32_t calculate_brightness_fast(uint16_t *data, int width, int height)
{
	uint64_t sum = 0;
	int samples = 0;
	int step = 50; /* Sample every 50 pixels */
	int i;
	int total = width * height;
	
	for (i = 0; i < total; i += step) {
		sum += data[i];
		samples++;
	}
	
	return (samples > 0) ? (uint32_t)(sum / samples) : 0;
}

/* VERY FAST: Just check a few corner pixels */
uint32_t calculate_brightness_minimal(uint16_t *data, int width, int height)
{
	uint64_t sum = 0;
	
	/* Sample only 9 points (corners, center, midpoints) */
	sum += data[0];                           /* Top-left */
	sum += data[width - 1];                   /* Top-right */
	sum += data[(height - 1) * width];        /* Bottom-left */
	sum += data[(height - 1) * width + width - 1]; /* Bottom-right */
	sum += data[(height / 2) * width + width / 2]; /* Center */
	sum += data[(height / 2) * width];        /* Middle-left */
	sum += data[(height / 2) * width + width - 1]; /* Middle-right */
	sum += data[width / 2];                   /* Top-center */
	sum += data[(height - 1) * width + width / 2]; /* Bottom-center */
	
	return (uint32_t)(sum / 9);
}

/* FAST: Quick change detection (sample-based) */
int frame_changed_fast(uint16_t *frame1, uint16_t *frame2, int size)
{
	int differences = 0;
	int i;
	int step = 500; /* Check every 500 pixels instead of 100 */
	
	for (i = 0; i < size / 2; i += step) {
		if (frame1[i] != frame2[i]) {
			differences++;
			if (differences > 5) return 1; /* Early exit */
		}
	}
	
	return differences > 2;
}

/* VERY FAST: Minimal change detection */
int frame_changed_minimal(uint16_t *frame1, uint16_t *frame2)
{
	/* Just check if ANY of the sample points changed */
	return (frame1[0] != frame2[0]) || 
	       (frame1[1000] != frame2[1000]) ||
	       (frame1[5000] != frame2[5000]);
}

void print_usage(const char *prog)
{
	printf("Usage: %s [frames] [mode] [save_images]\n", prog);
	printf("  frames: Number of frames to capture (default: 1000)\n");
	printf("  mode: CPU usage mode (default: 2)\n");
	printf("    0 = Full analysis (high CPU, detailed stats)\n");
	printf("    1 = Sample-based (medium CPU)\n");
	printf("    2 = Minimal (low CPU, fast)\n");
	printf("  save_images: 0=no, 1=yes (default: 0)\n");
	printf("\nExamples:\n");
	printf("  %s 10000              # 10k frames, minimal mode, no save\n", prog);
	printf("  %s 10000 2 0          # Same as above\n", prog);
	printf("  %s 1000 1 1           # 1k frames, medium CPU, save images\n", prog);
	printf("  %s 100 0 1            # 100 frames, full analysis, save images\n", prog);
}

int main(int argc, char *argv[])
{
	int fd;
	void *buffer;
	size_t buf_size;
	uint32_t frame_info[2];
	uint32_t last_frame_count = 0;
	int capture_count = 1000;
	int cpu_mode = MODE_MINIMAL;
	int save_images = 0;
	int i = 0;
	uint16_t *prev_frame = NULL;
	struct timeval start_time, current_time;
	struct pollfd pfd;
	int consecutive_timeouts = 0;
	int frames_skipped = 0;
	
	/* Setup signal handler */
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	
	/* Parse arguments */
	if (argc > 1) {
		if (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0) {
			print_usage(argv[0]);
			return 0;
		}
		capture_count = atoi(argv[1]);
	}
	if (argc > 2) {
		cpu_mode = atoi(argv[2]);
		if (cpu_mode < 0 || cpu_mode > 2) {
			printf("Invalid mode. Use 0, 1, or 2\n");
			return 1;
		}
	}
	if (argc > 3) {
		save_images = atoi(argv[3]);
	}
	
	const char *mode_names[] = {"FULL", "SAMPLE", "MINIMAL"};
	
	printf("=== Virtual Unicam Test (CPU Optimized) ===\n");
	printf("Target frames: %d\n", capture_count);
	printf("CPU mode: %s (%d)\n", mode_names[cpu_mode], cpu_mode);
	printf("Image saving: %s\n\n", save_images ? "YES (first 3)" : "NO");
	
	/* Open device */
	printf("[1] Opening /dev/unicam0...\n");
	fd = open("/dev/unicam0", O_RDWR | O_NONBLOCK);
	if (fd < 0) {
		perror("open /dev/unicam0");
		printf("Make sure the driver is loaded: sudo insmod unicam-virtual-sensor.ko\n");
		return 1;
	}
	printf("✓ Device opened\n\n");
	
	/* Get buffer size */
	printf("[2] Getting buffer...\n");
	if (ioctl(fd, UNICAM_IOC_GET_BUFFER, &buf_size) < 0) {
		perror("ioctl UNICAM_IOC_GET_BUFFER");
		close(fd);
		return 1;
	}
	printf("✓ Buffer: %zu bytes (%.1f MB)\n\n", 
		   buf_size, buf_size / (1024.0 * 1024.0));
	
	/* Map buffer */
	printf("[3] Mapping buffer...\n");
	buffer = mmap(NULL, buf_size, PROT_READ, MAP_SHARED, fd, 0);
	if (buffer == MAP_FAILED) {
		perror("mmap");
		close(fd);
		return 1;
	}
	printf("✓ Mapped at %p\n\n", buffer);
	
	/* Allocate prev_frame only if needed */
	if (cpu_mode < MODE_MINIMAL) {
		prev_frame = malloc(FRAME_SIZE);
		if (!prev_frame) {
			perror("malloc");
			munmap(buffer, buf_size);
			close(fd);
			return 1;
		}
		memset(prev_frame, 0, FRAME_SIZE);
	}
	
	/* Start streaming */
	printf("[4] Starting stream...\n");
	if (ioctl(fd, UNICAM_IOC_START_STREAM) < 0) {
		perror("ioctl UNICAM_IOC_START_STREAM");
		if (prev_frame) free(prev_frame);
		munmap(buffer, buf_size);
		close(fd);
		return 1;
	}
	printf("✓ Streaming started\n\n");
	
	printf("[5] Capturing frames (Ctrl+C to stop)...\n");
	printf("────────────────────────────────────────────\n");
	
	pfd.fd = fd;
	pfd.events = POLLIN;
	
	gettimeofday(&start_time, NULL);
	
	while (i < capture_count && running) {
		int ret = poll(&pfd, 1, POLL_TIMEOUT_MS);
		
		if (ret < 0) {
			if (errno == EINTR) continue;
			perror("poll");
			break;
		} else if (ret == 0) {
			consecutive_timeouts++;
			printf("[!] Timeout (%d/%d)\n", consecutive_timeouts, MAX_RETRIES);
			if (consecutive_timeouts >= MAX_RETRIES) {
				printf("[ERROR] Too many timeouts\n");
				break;
			}
			continue;
		}
		
		consecutive_timeouts = 0;
		
		ssize_t bytes_read = read(fd, frame_info, sizeof(frame_info));
		
		if (bytes_read < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
			perror("read");
			break;
		} else if (bytes_read != sizeof(frame_info)) {
			continue;
		}
		
		uint32_t frame_count = frame_info[0];
		uint32_t write_offset = frame_info[1];
		
		if (frame_count > last_frame_count) {
			/* Check for skipped frames */
			int skip = frame_count - last_frame_count - 1;
			if (skip > 0) {
				frames_skipped += skip;
			}
			
			uint32_t current_offset = (write_offset >= FRAME_SIZE) ? 
									  (write_offset - FRAME_SIZE) : 
									  (buf_size - FRAME_SIZE);
			
			uint16_t *frame_data = (uint16_t *)((uint8_t *)buffer + current_offset);
			
			/* Process frame based on CPU mode */
			if (cpu_mode == MODE_FULL_ANALYSIS) {
				/* Full analysis - HIGH CPU */
				if (i < 10 || i % 100 == 0) {
					gettimeofday(&current_time, NULL);
					double elapsed = (current_time.tv_sec - start_time.tv_sec) + 
									 (current_time.tv_usec - start_time.tv_usec) / 1000000.0;
					
					uint32_t brightness = calculate_brightness_fast(frame_data, FRAME_WIDTH, FRAME_HEIGHT);
					int changed = prev_frame ? frame_changed_fast(prev_frame, frame_data, FRAME_SIZE) : 1;
					
					printf("Frame #%5u | %.2fs | Offset: 0x%08x | Bright: %4u | %s\n",
						   frame_count, elapsed, current_offset, brightness,
						   changed ? "CHANGED" : "similar");
				}
				if (prev_frame) {
					memcpy(prev_frame, frame_data, FRAME_SIZE);
				}
			} else if (cpu_mode == MODE_SAMPLE_ONLY) {
				/* Sample-based - MEDIUM CPU */
				if (i < 10 || i % 250 == 0) {
					gettimeofday(&current_time, NULL);
					double elapsed = (current_time.tv_sec - start_time.tv_sec) + 
									 (current_time.tv_usec - start_time.tv_usec) / 1000000.0;
					
					uint32_t brightness = calculate_brightness_minimal(frame_data, FRAME_WIDTH, FRAME_HEIGHT);
					
					printf("Frame #%5u | %.2fs | Bright: %4u\n",
						   frame_count, elapsed, brightness);
				}
			} else {
				/* Minimal mode - LOW CPU */
				if (i % 500 == 0) {
					gettimeofday(&current_time, NULL);
					double elapsed = (current_time.tv_sec - start_time.tv_sec) + 
									 (current_time.tv_usec - start_time.tv_usec) / 1000000.0;
					
					printf("Frame #%5u | %.2fs | FPS: %.1f\n",
						   frame_count, elapsed, i / elapsed);
				}
			}
			
			/* Save images if requested */
			if (save_images && i < 3) {
				char filename[64];
				snprintf(filename, sizeof(filename), "frame_%03d.pgm", i);
				save_frame_as_pgm(filename, frame_data, FRAME_WIDTH, FRAME_HEIGHT);
			}
			
			last_frame_count = frame_count;
			i++;
		}
	}
	
	printf("────────────────────────────────────────────\n");
	
	gettimeofday(&current_time, NULL);
	double total_time = (current_time.tv_sec - start_time.tv_sec) + 
						(current_time.tv_usec - start_time.tv_usec) / 1000000.0;
	
	printf("\n[6] Statistics:\n");
	printf("  Frames captured: %d\n", i);
	printf("  Frames skipped: %d\n", frames_skipped);
	printf("  Total time: %.2f seconds\n", total_time);
	printf("  Average FPS: %.2f\n", i / total_time);
	printf("  Expected FPS: 30.0\n");
	printf("  CPU mode: %s\n\n", mode_names[cpu_mode]);
	
	/* Stop streaming */
	printf("[7] Stopping...\n");
	if (ioctl(fd, UNICAM_IOC_STOP_STREAM) < 0) {
		perror("ioctl UNICAM_IOC_STOP_STREAM");
	} else {
		printf("✓ Stopped\n\n");
	}
	
	/* Cleanup */
	printf("[8] Cleanup...\n");
	if (prev_frame) free(prev_frame);
	munmap(buffer, buf_size);
	close(fd);
	printf("✓ Done\n\n");
	
	printf("════════════════════════════════════════════\n");
	if (i >= capture_count) {
		printf("✓ Test completed: %d frames in %.1fs\n", i, total_time);
	} else {
		printf("⚠ Stopped early: %d/%d frames\n", i, capture_count);
	}
	printf("════════════════════════════════════════════\n");
	
	return (i >= capture_count) ? 0 : 1;
}