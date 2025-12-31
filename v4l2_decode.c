#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <errno.h>
#include <stdint.h>

#define INPUT_FILE           "frame.h264"
#define OUTPUT_FILE          "frame.yuv"
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

uint8_t *extract_one_frame(uint8_t *buf, long size, long *offset, long *frame_size,
                           int init_startcode) {
    if (*offset >= size)
        return NULL;

    long start = *offset;

    // 找第一个 slice NAL unit
    long next = find_next_start_code(buf, start, size);
    if (next == -1) {
        *frame_size    = size - start;
        *offset        = size;
        uint8_t *frame = malloc(*frame_size);
        memcpy(frame, buf + start, *frame_size);
        return frame;
    }

    // 如果 next == start，则跳过 start code
    if (next != start)
        start = next;

    // 找下一帧的起始位置（下一个 slice NAL unit）
    long next_frame = find_next_start_code(buf, start + 4, size);  // skip start code
    if (next_frame == -1)
        next_frame = size;

    *frame_size = next_frame - start;
    *offset     = next_frame;
    if (init_startcode) {
        uint8_t *frame = malloc(*frame_size + 6);
        frame[0]       = 0x00;
        frame[1]       = 0x00;
        frame[2]       = 0x00;
        frame[3]       = 0x01;
        frame[4]       = 0x09;
        frame[5]       = 0xf0;
        memcpy(frame + 6, buf + start, *frame_size);
        *frame_size += 6;
        return frame;
    } else {
        uint8_t *frame = malloc(*frame_size);
        memcpy(frame, buf + start, *frame_size);
        return frame;
    }
}

struct video {
    long      num_frames;
    uint8_t **frames;
    long     *fsizes;
};

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
        uint8_t *frame = extract_one_frame(buf, size, &offset, &frame_size, 0);
        if (!frame)
            break;
        frame_idx++;
        free(frame);
    }
    v.num_frames = frame_idx - 3;

    v.frames  = malloc((frame_idx) * sizeof(uint8_t *));
    v.fsizes  = malloc((frame_idx) * sizeof(long));
    offset    = 0;
    frame_idx = 0;
    // Handle first 3 frames (SPS, PPS, IDR)
    long fsize0, fsize1, fsize2, fsize3;
    v.fsizes[0]     = 0;
    uint8_t *frame0 = extract_one_frame(buf, size, &offset, &fsize0, 1);
    v.fsizes[0] += fsize0;
    uint8_t *frame1 = extract_one_frame(buf, size, &offset, &fsize1, 0);
    v.fsizes[0] += fsize1;
    uint8_t *frame2 = extract_one_frame(buf, size, &offset, &fsize2, 0);
    v.fsizes[0] += fsize2;
    uint8_t *frame3 = extract_one_frame(buf, size, &offset, &fsize3, 0);
    v.fsizes[0] += fsize3;
    v.frames[0] = malloc(v.fsizes[0]);
    long pos    = 0;
    memcpy(v.frames[0] + pos, frame0, fsize0);
    pos += fsize0;
    free(frame0);
    memcpy(v.frames[0] + pos, frame1, fsize1);
    pos += fsize1;
    free(frame1);
    memcpy(v.frames[0] + pos, frame2, fsize2);
    pos += fsize2;
    free(frame2);
    memcpy(v.frames[0] + pos, frame3, fsize3);
    pos += fsize3;
    free(frame3);
    frame_idx++;

    while (offset < size) {
        long     fsize;
        uint8_t *frame = extract_one_frame(buf, size, &offset, &fsize, 1);
        if (!frame)
            break;
        v.frames[frame_idx] = frame;
        v.fsizes[frame_idx] = fsize;
        frame_idx++;
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

void dq_output(int fd) {
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
        return;
    }
    printf("OUTPUT DQBUF index=%d, bytesused=%d\n", buf.index, buf.m.planes[0].bytesused);
}

void dq_capture(int fd) {
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
    printf("CAPTURE DQBUF index=%d, bytesused=%d\n", buf.index, buf.m.planes[0].bytesused);
}

int main(int argc, char *argv[]) {
    int   fd = open(argv[1], O_RDWR);
    if (fd < 0) {
        perror("open");
        return 1;
    }
    struct video v  = get_video_frames(argv[2]);

    // 1. Set OUTPUT format (H.264)
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

    // 3. Request OUTPUT buffers
    struct v4l2_requestbuffers reqbuf;
    memset(&reqbuf, 0, sizeof(reqbuf));
    reqbuf.count  = OUTPUT_BUFFER_COUNT;
    reqbuf.type   = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    reqbuf.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &reqbuf) < 0) {
        perror("REQBUFS OUTPUT");
        return 1;
    }

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
        outbufs[i].bytesused  = v.fsizes[i];
        outbufs[i].length     = buf.m.planes[0].length;
        outbufs[i].mem_offset = buf.m.planes[0].m.mem_offset;
    }

    for (int i = 0; i < OUTPUT_BUFFER_COUNT; i++) {
        outbufs[i].start = mmap(NULL, outbufs[i].length, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                                outbufs[i].mem_offset);
        if (outbufs[i].start == MAP_FAILED) {
            perror("mmap OUTPUT");
            return 1;
        }
        memcpy(outbufs[i].start, v.frames[i], v.fsizes[i]);
    }

    // QBUF all OUTPUT buffers
    for (int i = 0; i < OUTPUT_BUFFER_COUNT; i++) {
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
            return 1;
        }
        printf("OUTPUT QBUF index=%d, bytesused=%d\n", buf.index, buf.m.planes[0].bytesused);
    }
    {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
            perror("STREAMON OUTPUT");
            return 1;
        }
    }


    struct v4l2_format gfmt;
    memset(&gfmt, 0, sizeof(gfmt));
    gfmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(fd, VIDIOC_G_FMT, &gfmt) < 0) {
        perror("G_FMT CAPTURE");
        return 1;
    }

    int fcc = gfmt.fmt.pix_mp.pixelformat;
    char fcc_str[5] = {
        (char)(fcc & 0xFF),
        (char)((fcc >> 8) & 0xFF),
        (char)((fcc >> 16) & 0xFF),
        (char)((fcc >> 24) & 0xFF),
        '\0'
    };

    printf("CAPTURE format: width=%d, height=%d, pixelformat=%s\n",
           gfmt.fmt.pix_mp.width, gfmt.fmt.pix_mp.height, fcc_str);

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
        capbufs[i].length     = buf.m.planes[0].length;
        capbufs[i].mem_offset = buf.m.planes[0].m.mem_offset;
    }

    for (int i = 0; i < CAPTURE_BUFFER_COUNT; i++) {
        capbufs[i].start = mmap(NULL, capbufs[i].length, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                                capbufs[i].mem_offset);
        if (capbufs[i].start == MAP_FAILED) {
            perror("mmap CAPTURE");
            return 1;
        }
    }


    // QBUF all CAPTURE buffers
    for (int i = 0; i < CAPTURE_BUFFER_COUNT; i++) {
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
            return 1;
        }
    }
    {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
            perror("STREAMON CAPTURE");
            return 1;
        }
    }

    for (int i = 0; i < OUTPUT_BUFFER_COUNT; i++) {
        dq_output(fd);
    }

    for (int i = 0; i < CAPTURE_BUFFER_COUNT; i++) {
        dq_capture(fd);
    }
    

    // Export data

    // for (int i = 0; i < CAPTURE_BUFFER_COUNT; i++) {
    //     // Write to file
    //     char filename[64];
    //     snprintf(filename, sizeof(filename), "output_frame_%d.nv12", i);
    //     FILE *f = fopen(filename, "wb");
    //     if (!f) {
    //         perror("fopen output");
    //         continue;   
    //     }
    //     fwrite(capbufs[i].start, 1, capbufs[i].bytesused, f);
    //     fclose(f);
    // }

    close(fd);
    return 0;
}
