#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/poll.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <stdint.h>
#include <time.h>

#define OUTPUT_BUFFER_COUNT  4
#define CAPTURE_BUFFER_COUNT 4

long find_next_start_code(uint8_t *buf, long offset, long size) {
    for (long i = offset; i + 3 < size; i++) {
        if (buf[i] == 0x00 && buf[i + 1] == 0x00) {
            if (buf[i + 2] == 0x01)
                return i;
            else if (i + 3 < size && buf[i + 2] == 0x00 && buf[i + 3] == 0x01)
                return i;
        }
    }
    return -1;
}

uint8_t *extract_one_frame(uint8_t *buf, long size, long *offset, long *frame_size) {
    if (*offset >= size)
        return NULL;

    long start = *offset;

    long next = find_next_start_code(buf, start, size);
    if (next == -1) {
        *frame_size    = size - start;
        *offset        = size;
        uint8_t *frame = malloc(*frame_size);
        memcpy(frame, buf + start, *frame_size);
        return frame;
    }

    if (next != start)
        start = next;

    long next_frame = find_next_start_code(buf, start + 4, size);  // skip start code
    if (next_frame == -1)
        next_frame = size;

    *frame_size    = next_frame - start;
    *offset        = next_frame;
    uint8_t *frame = malloc(*frame_size);
    memcpy(frame, buf + start, *frame_size);
    return frame;
}

struct video {
    long      num_frames;
    uint8_t **frames;
    long     *fsizes;
};

static int is_slice(uint8_t *frame) {
    uint8_t kf;
    if (frame[2] == 0) {
        kf = frame[4];
    } else {
        kf = frame[3];
    }
    kf = kf & 0x1f;
    return kf >= 1 && kf <= 5;
}

struct video get_video_frames(char *filename) {
    struct video v;
    memset(&v, 0, sizeof(struct video));

    FILE *f = fopen(filename, "rb");
    if (!f) {
        perror("fopen");
        return v;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    uint8_t *buf = malloc(size);
    fread(buf, 1, size, f);
    fclose(f);

    long offset     = 0;
    long frame_size = 0;
    int  frame_idx  = 0;

    while (offset < size) {
        uint8_t *frame = extract_one_frame(buf, size, &offset, &frame_size);
        if (!frame)
            break;
        if (is_slice(frame)) {
            frame_idx++;
        }
        free(frame);
    }
    v.num_frames = frame_idx;

    v.frames          = malloc((frame_idx) * sizeof(uint8_t *));
    v.fsizes          = malloc((frame_idx) * sizeof(long));
    offset            = 0;
    frame_idx         = 0;
    uint8_t *acc      = NULL;
    long     acc_size = 0;

    while (offset < size) {
        long     fsize;
        uint8_t *frame = extract_one_frame(buf, size, &offset, &fsize);
        if (!frame)
            break;

        // Collect frames
        acc = realloc(acc, acc_size + fsize);
        memcpy(acc + acc_size, frame, fsize);
        acc_size += fsize;

        if (is_slice(frame)) {
            // Slice
            v.frames[frame_idx] = acc;
            v.fsizes[frame_idx] = acc_size;
            frame_idx++;

            acc      = NULL;
            acc_size = 0;
        }

        free(frame);
    }
    free(buf);
    return v;
}

struct buffer {
    void  *start;
    size_t length;
    size_t bytesused;
    size_t mem_offset;
};

struct buffer outbufs[OUTPUT_BUFFER_COUNT];
struct buffer capbufs[CAPTURE_BUFFER_COUNT];
struct video  v_data;
size_t        current_frame;
int           fd;

void change_resolution() {
    struct v4l2_format gfmt;
    memset(&gfmt, 0, sizeof(gfmt));
    gfmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(fd, VIDIOC_G_FMT, &gfmt) < 0) {
        perror("G_FMT CAPTURE");
        return;
    }

    int  fcc        = gfmt.fmt.pix_mp.pixelformat;
    char fcc_str[5] = {(char)(fcc & 0xFF), (char)((fcc >> 8) & 0xFF), (char)((fcc >> 16) & 0xFF),
                       (char)((fcc >> 24) & 0xFF), '\0'};

    printf("Capture format: width=%d, height=%d, pixelformat=%s\n", gfmt.fmt.pix_mp.width,
           gfmt.fmt.pix_mp.height, fcc_str);
}

void poll_ev() {
    struct pollfd pfd[1];
    pfd[0].fd     = fd;
    pfd[0].events = POLLIN | POLLOUT | POLLPRI;
    poll(pfd, 1, -1);
    if (pfd[0].revents) {
        if (pfd[0].revents & POLLIN) {
            // printf("POLLIN\n");
        }
        if (pfd[0].revents & POLLOUT) {
            // printf("POLLOUT\n");
        }
        if (pfd[0].revents & POLLPRI) {
            // printf("POLLPRI\n");

            struct v4l2_event ev;
            memset(&ev, 0, sizeof(struct v4l2_event));
            if (ioctl(fd, VIDIOC_DQEVENT, &ev) < 0) {
                perror("DQEVENT");
                return;
            }
            // printf("Event %d observed\n", ev.type);
            if (ev.type == V4L2_EVENT_SOURCE_CHANGE) {
                change_resolution();
            }
        }
    }
}

void wcap2file(int i) {
    FILE *f = fopen("output.nv12", "wb");
    if (!f) {
        perror("fopen output");
        return;
    }
    fwrite(capbufs[i].start, 1, capbufs[i].bytesused, f);
    fclose(f);
}

int q_output() {
    if (current_frame == v_data.num_frames) {
        printf("All frames queued\n");
        return 1;
    }
    size_t fid           = current_frame++;
    size_t i             = fid % OUTPUT_BUFFER_COUNT;
    outbufs[i].bytesused = v_data.fsizes[fid];
    memcpy(outbufs[i].start, v_data.frames[fid], v_data.fsizes[fid]);
    struct v4l2_buffer buf;
    struct v4l2_plane  planes[1];
    memset(&buf, 0, sizeof(buf));
    memset(planes, 0, sizeof(planes));
    planes[0].bytesused    = outbufs[i].bytesused;
    planes[0].length       = outbufs[i].length;
    planes[0].m.mem_offset = outbufs[i].mem_offset;
    buf.type               = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    buf.memory             = V4L2_MEMORY_MMAP;
    buf.index              = i;
    buf.m.planes           = planes;
    buf.length             = 1;
    if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
        perror("QBUF OUTPUT");
        return 2;
    }
    // printf("OUTPUT QBUF index=%d, bytesused=%d, frameid=%ld/%ld\n", buf.index,
    //        buf.m.planes[0].bytesused, fid, v_data.num_frames);
    return 0;
}

void q_capture(int i) {
    struct v4l2_buffer buf;
    struct v4l2_plane  planes[1];
    memset(&buf, 0, sizeof(buf));
    memset(planes, 0, sizeof(planes));
    planes[0].bytesused    = capbufs[i].bytesused;
    planes[0].length       = capbufs[i].length;
    planes[0].m.mem_offset = capbufs[i].mem_offset;
    buf.type               = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory             = V4L2_MEMORY_MMAP;
    buf.index              = i;
    buf.m.planes           = planes;
    buf.length             = 1;
    if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
        perror("QBUF CAPTURE");
    }
}

int dq_output() {
    // poll_ev();
    struct v4l2_buffer buf;
    struct v4l2_plane  planes[1];
    memset(&buf, 0, sizeof(buf));
    memset(planes, 0, sizeof(planes));
    buf.type     = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    buf.memory   = V4L2_MEMORY_MMAP;
    buf.m.planes = planes;
    buf.length   = 1;
    if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
        perror("DQBUF OUTPUT");
        return 2;
    }
    if (outbufs[buf.index].bytesused != buf.m.planes[0].bytesused) {
        printf("Mismatch OUTPUT DQBUF index=%d, bytesused=%d, expected=%zu\n", buf.index,
               buf.m.planes[0].bytesused, outbufs[buf.index].bytesused);
        return 2;
    }
    // printf("OUTPUT DQBUF index=%d, bytesused=%d\n", buf.index, buf.m.planes[0].bytesused);
    return q_output();
}

void dq_capture() {
    struct v4l2_buffer buf;
    struct v4l2_plane  planes[1];
    memset(&buf, 0, sizeof(buf));
    memset(planes, 0, sizeof(planes));
    buf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory   = V4L2_MEMORY_MMAP;
    buf.m.planes = planes;
    buf.length   = 1;
    if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
        perror("DQBUF CAPTURE");
        return;
    }
    capbufs[buf.index].bytesused = buf.m.planes[0].bytesused;
    // printf("CAPTURE DQBUF index=%d, bytesused=%d\n", buf.index, buf.m.planes[0].bytesused);
    q_capture(buf.index);
}

void streamon_output() {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        perror("STREAMON OUTPUT");
        return;
    }
}

void streamon_capture() {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        perror("STREAMON CAPTURE");
        return;
    }
}
void streamoff_output() {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
        perror("STREAMOFF OUTPUT");
        return;
    }
}

void streamoff_capture() {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
        perror("STREAMOFF CAPTURE");
        return;
    }
}

int main(int argc, char *argv[]) {
    if (argc < 3) {
        printf("Usage: %s <v4l2_device> <input_h264_file>\n", argv[0]);
        return 1;
    }
    fd = open(argv[1], O_RDWR);
    if (fd < 0) {
        perror("open");
        return 1;
    }
    v_data        = get_video_frames(argv[2]);
    current_frame = 0;

    struct v4l2_event_subscription ev_sub;
    memset(&ev_sub, 0, sizeof(struct v4l2_event_subscription));
    ev_sub.type = V4L2_EVENT_SOURCE_CHANGE;
    if (ioctl(fd, VIDIOC_SUBSCRIBE_EVENT, &ev_sub) < 0) {
        perror("SUBSCRIBE EVENT");
        return 1;
    }

    // Set OUTPUT format (H.264)
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type                   = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    fmt.fmt.pix_mp.width       = 1920;
    fmt.fmt.pix_mp.height      = 1080;
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
    fmt.fmt.pix_mp.field       = V4L2_FIELD_NONE;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        perror("S_FMT OUTPUT");
        return 1;
    }

    // Request OUTPUT buffers
    struct v4l2_requestbuffers reqbuf;
    memset(&reqbuf, 0, sizeof(reqbuf));
    reqbuf.count  = OUTPUT_BUFFER_COUNT;
    reqbuf.type   = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    reqbuf.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &reqbuf) < 0) {
        perror("REQBUFS OUTPUT");
        return 1;
    }
    // printf("OUTPUT BUFFER COUNT = %d\n", reqbuf.count);

    // QUERYBUF and mmap OUTPUT buffers
    for (int i = 0; i < OUTPUT_BUFFER_COUNT; i++) {
        struct v4l2_buffer buf;
        struct v4l2_plane  planes[1];
        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));
        buf.type     = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        buf.memory   = V4L2_MEMORY_MMAP;
        buf.index    = i;
        buf.m.planes = planes;
        buf.length   = 1;

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
            perror("QUERYBUF OUTPUT");
            return 1;
        }
        outbufs[i].bytesused  = v_data.fsizes[i];
        outbufs[i].length     = buf.m.planes[0].length;
        outbufs[i].mem_offset = buf.m.planes[0].m.mem_offset;
        outbufs[i].start = mmap(NULL, outbufs[i].length, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                                outbufs[i].mem_offset);
        if (outbufs[i].start == MAP_FAILED) {
            perror("mmap OUTPUT");
            return 1;
        }
    }

    // QBUF first frame to get video info
    q_output();

    // STREAMON
    streamon_output();

    poll_ev();

    // CAPTURE

    // Request CAPTURE buffers
    struct v4l2_requestbuffers capbuf;
    memset(&capbuf, 0, sizeof(capbuf));
    capbuf.count  = OUTPUT_BUFFER_COUNT;
    capbuf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    capbuf.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &capbuf) < 0) {
        perror("REQBUFS CAPTURE");
        return 1;
    }
    // printf("CAPTURE BUFFER COUNT = %d\n", capbuf.count);

    // QUERYBUF and mmap CAPTURE buffers
    for (int i = 0; i < CAPTURE_BUFFER_COUNT; i++) {
        struct v4l2_buffer buf;
        struct v4l2_plane  planes[1];
        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));
        buf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory   = V4L2_MEMORY_MMAP;
        buf.index    = i;
        buf.m.planes = planes;
        buf.length   = 1;

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
            perror("QUERYBUF CAPTURE");
            return 1;
        }
        capbufs[i].bytesused  = buf.m.planes[0].length;
        capbufs[i].length     = buf.m.planes[0].length;
        capbufs[i].mem_offset = buf.m.planes[0].m.mem_offset;
        capbufs[i].start = mmap(NULL, capbufs[i].length, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                                capbufs[i].mem_offset);
        if (capbufs[i].start == MAP_FAILED) {
            perror("mmap CAPTURE");
            return 1;
        }
    }

    // QBUF all CAPTURE buffers
    for (int i = 0; i < CAPTURE_BUFFER_COUNT; i++) {
        q_capture(i);
    }

    // STREAMON CAPTURE
    streamon_capture();

    for (int i = 0; i < 5; ++i) {
        int ret = dq_output();
        if (ret == 1) {
            // Done
            break;
        }
        if (ret == 2) {
            return 1;
        }
        dq_capture();
    }

    streamoff_capture();
    streamoff_output();

    current_frame = 50;  // Seek

    q_output();
    streamon_output();

    for (int i = 0; i < CAPTURE_BUFFER_COUNT; i++) {
        q_capture(i);
    }
    streamon_capture();

    for (int i = 0; i < 100; ++i) {
        int ret = dq_output();
        if (ret == 1) {
            // Done
            break;
        }
        if (ret == 2) {
            return 1;
        }
        dq_capture();
    }

    close(fd);
    return 0;
}
