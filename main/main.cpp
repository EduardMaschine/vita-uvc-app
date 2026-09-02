// ============================================================================
// Real‑time NV12 to RGB renderer for Raspberry Pi (Pi3B+, Pi4, Pi5)
// Uses V4L2 capture + EGL + GLES2 + X11 fullscreen window
//
// Two capture paths:
//   * Pi4/5:   DMA-BUF
//   * Pi3B+:   MMAP + CPU upload via glTexSubImage2D
//
// Command‑line switches:
//   --res-720 / --res-1080 / --res-2160   -> internal render resolution | This has been deprecated in v1.6 since it caused performance issues, traces are being kept in if I want to comeback to this again at some point...
//   --rgb-full / --rgb-limited            -> input YUV range
//   --post-full / --post-limited          -> output RGB range
//   --fps-30 / --fps-60                   -> capture FPS
//   --vita-272 / 488 / 504 / 544 / 720    -> capture resolution presets
//   --filter-nearest / --filter-bilinear  -> filter options
//   --audio                               -> enables audio output from USB over to HDMI by using pw-loopback.
//   --vsync-on / --vsync-off              -> enables / disables in app vsync (if off, it can cause issues in --pi3bp mode at 272 60fps)
//
// Keyboard:
//   F1  ->  NEAREST/BILINEAR filter toggle
//   F2  ->  cycle Vita resolution
//   F3   ->  toggle FPS 30/60
//   F4   ->  toggle pre RGB full/limited
//   F5   ->  toggle post RGB full/limited
//   F6  ->  toggle VSync on/off
//   ESC ->  exit
// ============================================================================

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xatom.h>
#include <X11/keysym.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/select.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <signal.h>
#include <time.h>

extern "C" {
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <linux/videodev2.h>
#include <libdrm/drm_fourcc.h>
}

// GL_EXT_texture_rg fallback (Pi4/Pi5 path)
#ifndef GL_RG_EXT
#define GL_RG_EXT 0x8227
#endif
#ifndef GL_RED_EXT
#define GL_RED_EXT 0x1903
#endif

#define MAX_V4L2_BUFFERS 3


struct V4L2State {
    int fd;
    uint32_t width;
    uint32_t height;
    uint32_t y_stride;
    uint32_t uv_stride;
    uint32_t buf_count;

    // DMA-BUF path (Pi4/Pi5)
    int      dmabuf_fds[MAX_V4L2_BUFFERS];

    // MMAP path (Pi3B+)
    uint8_t* base[MAX_V4L2_BUFFERS];
    size_t   buf_size[MAX_V4L2_BUFFERS];
};

static bool g_vsync_enabled = true;  // default vsync value
static bool g_use_dmabuf   = true;   // default: Pi4/Pi5
static bool g_pi3_mode     = false;  // set by --pi3bp
static bool g_enable_audio = false;  // set by --audio
static pid_t g_loopback_pid = 0;

// Toast system (GL-based bitmap font banner)
static char   g_toast_msg[256] = {0};
static double g_toast_until    = 0.0;

// Forward declarations for toast rendering
static void show_toast(const char* msg);
static void render_toast_banner(GLuint progText,
                                GLint locScreenSize,
                                GLint locTextColor,
                                int screen_w,
                                int screen_h);

static void die(const char* msg) {
    perror(msg);
    exit(1);
}

static void set_fps(int fd, int fps) {
    struct v4l2_streamparm parm{};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (ioctl(fd, VIDIOC_G_PARM, &parm) < 0)
        die("VIDIOC_G_PARM");

    parm.parm.capture.timeperframe.numerator   = 1;
    parm.parm.capture.timeperframe.denominator = fps;

    if (ioctl(fd, VIDIOC_S_PARM, &parm) < 0)
        die("VIDIOC_S_PARM");

    printf("Requested FPS: %d, device timeperframe: %d/%d\n",
           fps,
           parm.parm.capture.timeperframe.numerator,
           parm.parm.capture.timeperframe.denominator);
}

// BLOCKING init: waits until /dev/video0 is available and working
// For Pi4/Pi5: sets up DMA-BUF export
// For Pi3B+: uses MMAP and CPU upload
static V4L2State init_v4l2_blocking(const char* dev, int fps, uint32_t cap_w, uint32_t cap_h) {
    V4L2State v{};
    memset(&v, 0, sizeof(v));
    for (uint32_t i = 0; i < MAX_V4L2_BUFFERS; ++i) {
        v.dmabuf_fds[i] = -1;
        v.base[i]       = nullptr;
        v.buf_size[i]   = 0;
    }

    while (true) {
        v.fd = open(dev, O_RDWR | O_NONBLOCK, 0);
        if (v.fd < 0) {
            fprintf(stderr, "open %s failed (%d: %s), retrying...\n",
                    dev, errno, strerror(errno));
            usleep(500 * 1000); // 500 ms
            continue;
        }

        struct v4l2_format fmt{};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width       = cap_w;
        fmt.fmt.pix.height      = cap_h;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
        fmt.fmt.pix.field       = V4L2_FIELD_NONE;

        if (ioctl(v.fd, VIDIOC_S_FMT, &fmt) < 0) {
            fprintf(stderr, "VIDIOC_S_FMT failed (%d: %s), closing and retrying...\n",
                    errno, strerror(errno));
            close(v.fd);
            v.fd = -1;
            usleep(500 * 1000);
            continue;
        }

        set_fps(v.fd, fps);

        v.width     = fmt.fmt.pix.width;
        v.height    = fmt.fmt.pix.height;
        v.y_stride  = fmt.fmt.pix.bytesperline;
        v.uv_stride = fmt.fmt.pix.bytesperline;

        struct v4l2_requestbuffers req{};
        req.count  = 2; // dual buffering
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (ioctl(v.fd, VIDIOC_REQBUFS, &req) < 0) {
            fprintf(stderr, "VIDIOC_REQBUFS failed (%d: %s), closing and retrying...\n",
                    errno, strerror(errno));
            close(v.fd);
            v.fd = -1;
            usleep(500 * 1000);
            continue;
        }
        if (req.count < 1) {
            fprintf(stderr, "No V4L2 buffers, closing and retrying...\n");
            close(v.fd);
            v.fd = -1;
            usleep(500 * 1000);
            continue;
        }

        if (req.count > MAX_V4L2_BUFFERS)
            req.count = MAX_V4L2_BUFFERS;

        v.buf_count = req.count;

        if (g_use_dmabuf) {
            // Pi4/Pi5: export all buffers as DMA-BUF and queue them
            for (uint32_t i = 0; i < v.buf_count; ++i) {
                struct v4l2_exportbuffer exp{};
                memset(&exp, 0, sizeof(exp));
                exp.type  = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                exp.index = i;
                exp.flags = O_CLOEXEC;

                if (ioctl(v.fd, VIDIOC_EXPBUF, &exp) < 0) {
                    fprintf(stderr, "VIDIOC_EXPBUF failed (%d: %s), closing and retrying...\n",
                            errno, strerror(errno));
                    for (uint32_t j = 0; j < i; ++j) {
                        if (v.dmabuf_fds[j] >= 0)
                            close(v.dmabuf_fds[j]);
                        v.dmabuf_fds[j] = -1;
                    }
                    close(v.fd);
                    v.fd = -1;
                    usleep(500 * 1000);
                    goto retry;
                }

                v.dmabuf_fds[i] = exp.fd;

                struct v4l2_buffer buf{};
                memset(&buf, 0, sizeof(buf));
                buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;
                buf.index  = i;

                if (ioctl(v.fd, VIDIOC_QBUF, &buf) < 0) {
                    fprintf(stderr, "VIDIOC_QBUF (initial) failed (%d: %s), closing and retrying...\n",
                            errno, strerror(errno));
                    for (uint32_t j = 0; j < i + 1; ++j) {
                        if (v.dmabuf_fds[j] >= 0)
                            close(v.dmabuf_fds[j]);
                        v.dmabuf_fds[j] = -1;
                    }
                    close(v.fd);
                    v.fd = -1;
                    usleep(500 * 1000);
                    goto retry;
                }
            }
        } else {
            // Pi3B+: MMAP buffers and queue them
            for (uint32_t i = 0; i < v.buf_count; ++i) {
                struct v4l2_buffer buf{};
                memset(&buf, 0, sizeof(buf));
                buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;
                buf.index  = i;

                if (ioctl(v.fd, VIDIOC_QUERYBUF, &buf) < 0) {
                    fprintf(stderr, "VIDIOC_QUERYBUF failed (%d: %s), closing and retrying...\n",
                            errno, strerror(errno));
                    for (uint32_t j = 0; j < i; ++j) {
                        if (v.base[j]) {
                            munmap(v.base[j], v.buf_size[j]);
                            v.base[j]     = nullptr;
                            v.buf_size[j] = 0;
                        }
                    }
                    close(v.fd);
                    v.fd = -1;
                    usleep(500 * 1000);
                    goto retry;
                }

                void* ptr = mmap(NULL, buf.length,
                                 PROT_READ | PROT_WRITE,
                                 MAP_SHARED, v.fd, buf.m.offset);
                if (ptr == MAP_FAILED) {
                    fprintf(stderr, "mmap failed (%d: %s), closing and retrying...\n",
                            errno, strerror(errno));
                    for (uint32_t j = 0; j < i; ++j) {
                        if (v.base[j]) {
                            munmap(v.base[j], v.buf_size[j]);
                            v.base[j]     = nullptr;
                            v.buf_size[j] = 0;
                        }
                    }
                    close(v.fd);
                    v.fd = -1;
                    usleep(500 * 1000);
                    goto retry;
                }

                v.base[i]     = (uint8_t*)ptr;
                v.buf_size[i] = buf.length;

                if (ioctl(v.fd, VIDIOC_QBUF, &buf) < 0) {
                    fprintf(stderr, "VIDIOC_QBUF (initial) failed (%d: %s), closing and retrying...\n",
                            errno, strerror(errno));
                    for (uint32_t j = 0; j < i + 1; ++j) {
                        if (v.base[j]) {
                            munmap(v.base[j], v.buf_size[j]);
                            v.base[j]     = nullptr;
                            v.buf_size[j] = 0;
                        }
                    }
                    close(v.fd);
                    v.fd = -1;
                    usleep(500 * 1000);
                    goto retry;
                }
            }
        }

        {
            enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (ioctl(v.fd, VIDIOC_STREAMON, &type) < 0) {
                fprintf(stderr, "VIDIOC_STREAMON failed (%d: %s), closing and retrying...\n",
                        errno, strerror(errno));
                if (g_use_dmabuf) {
                    for (uint32_t j = 0; j < v.buf_count; ++j) {
                        if (v.dmabuf_fds[j] >= 0)
                            close(v.dmabuf_fds[j]);
                        v.dmabuf_fds[j] = -1;
                    }
                } else {
                    for (uint32_t j = 0; j < v.buf_count; ++j) {
                        if (v.base[j]) {
                            munmap(v.base[j], v.buf_size[j]);
                            v.base[j]     = nullptr;
                            v.buf_size[j] = 0;
                        }
                    }
                }
                close(v.fd);
                v.fd = -1;
                usleep(500 * 1000);
                continue;
            }
        }

        fprintf(stderr, "V4L2 started (%s): %ux%u (buffers: %u)\n",
                g_use_dmabuf ? "DMA-BUF" : "MMAP",
                v.width, v.height, v.buf_count);
        return v;

    retry:
        memset(&v, 0, sizeof(v));
        for (uint32_t i = 0; i < MAX_V4L2_BUFFERS; ++i) {
            v.dmabuf_fds[i] = -1;
            v.base[i]       = nullptr;
            v.buf_size[i]   = 0;
        }
        continue;
    }
}

static void restart_v4l2(V4L2State &cam, int fps, uint32_t cap_w, uint32_t cap_h, int &uv_w, int &uv_h) {
    fprintf(stderr, "Restarting V4L2 capture (%s)...\n", g_use_dmabuf ? "DMA-BUF" : "MMAP");

    for (uint32_t i = 0; i < cam.buf_count; ++i) {
        if (cam.dmabuf_fds[i] >= 0) {
            close(cam.dmabuf_fds[i]);
            cam.dmabuf_fds[i] = -1;
        }
        if (cam.base[i]) {
            munmap(cam.base[i], cam.buf_size[i]);
            cam.base[i]     = nullptr;
            cam.buf_size[i] = 0;
        }
    }
    cam.buf_count = 0;

    if (cam.fd >= 0) {
        close(cam.fd);
        cam.fd = -1;
    }

    cam = init_v4l2_blocking("/dev/video0", fps, cap_w, cap_h);
    uv_w = cam.width / 2;
    uv_h = cam.height / 2;

    fprintf(stderr, "V4L2 restarted (%s): %ux%u\n",
            g_use_dmabuf ? "DMA-BUF" : "MMAP",
            cam.width, cam.height);
}

// Vertex shader for fullscreen quad (YUV to RGB)
static const char* vs_src = R"(
attribute vec2 aPos;
attribute vec2 aTex;
varying vec2 vTex;
void main() {
    vTex = aTex;
    gl_Position = vec4(aPos, 0.0, 1.0);
}
)";

// Fragment shader: single-pass NV12 to RGB
static const char* fs_yuv_single_pass_src = R"(
#extension GL_EXT_texture_rg : enable
precision mediump float;

varying vec2 vTex;

uniform sampler2D texY;      // NV12 Y plane
uniform sampler2D texUV;     // NV12 UV plane (interleaved)

uniform float y_scale;
uniform float y_offset;
uniform float uv_scale;
uniform float post_scale;
uniform float post_offset;

uniform int pi3_mode;        // 0 = Pi4/Pi5 (R8/RG88), 1 = Pi3B+ (LUMINANCE/LUMINANCE_ALPHA)

void main() {
    float y = texture2D(texY, vTex).r;

    vec2 uv_raw;
    if (pi3_mode == 1) {
        // Pi3B+: texUV is GL_LUMINANCE_ALPHA → U in .r, V in .a
        vec4 uv_tex = texture2D(texUV, vTex);
        uv_raw = vec2(uv_tex.r, uv_tex.a);
    } else {
        // Pi4/Pi5: texUV is GL_RG_EXT → U in .r, V in .g
        uv_raw = texture2D(texUV, vTex).rg;
    }

    vec2 uv = uv_raw - vec2(0.5, 0.5);

    float Y = y * y_scale + y_offset;
    float U = uv.x * uv_scale;
    float V = uv.y * uv_scale;

    vec3 rgb;
    rgb.r = Y + 1.402 * V;
    rgb.g = Y - 0.344136 * U - 0.714136 * V;
    rgb.b = Y + 1.772 * U;

    rgb = rgb * post_scale + post_offset;

    gl_FragColor = vec4(rgb, 1.0);
}
)";

// Simple text shaders (pixel-space to NDC, solid color)
static const char* vs_text_src = R"(
attribute vec2 aPos;
uniform vec2 uScreenSize;
void main() {
    vec2 ndc = (aPos / uScreenSize) * 2.0 - 1.0;
    ndc.y = -ndc.y;
    gl_Position = vec4(ndc, 0.0, 1.0);
}
)";

static const char* fs_text_src = R"(
precision mediump float;
uniform vec4 uColor;
void main() {
    gl_FragColor = uColor;
}
)";

static GLuint compile_shader(GLenum type, const char* src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, NULL);
    glCompileShader(s);

    GLint ok;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetShaderInfoLog(s, sizeof(log), NULL, log);
        fprintf(stderr, "Shader error: %s\n", log);
        exit(1);
    }
    return s;
}

static GLuint create_program(const char* fs_src_in) {
    GLuint vs = compile_shader(GL_VERTEX_SHADER, vs_src);
    GLuint fs = compile_shader(GL_FRAGMENT_SHADER, fs_src_in);

    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);

    glBindAttribLocation(prog, 0, "aPos");
    glBindAttribLocation(prog, 1, "aTex");

    glLinkProgram(prog);

    GLint ok;
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetProgramInfoLog(prog, sizeof(log), NULL, log);
        fprintf(stderr, "Program error: %s\n", log);
        exit(1);
    }

    glDeleteShader(vs);
    glDeleteShader(fs);
    return prog;
}

// Separate program for text (uses vs_text_src / fs_text_src)
static GLuint create_program_text() {
    GLuint vs = compile_shader(GL_VERTEX_SHADER, vs_text_src);
    GLuint fs = compile_shader(GL_FRAGMENT_SHADER, fs_text_src);

    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);

    glBindAttribLocation(prog, 0, "aPos");

    glLinkProgram(prog);

    GLint ok;
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetProgramInfoLog(prog, sizeof(log), NULL, log);
        fprintf(stderr, "Text program error: %s\n", log);
        exit(1);
    }

    glDeleteShader(vs);
    glDeleteShader(fs);
    return prog;
}

// ---------------------------------------------------------------------------
// Minimal 5x7 bitmap font for ASCII subset used in toast messages
// ---------------------------------------------------------------------------

struct Glyph {
    char ch;
    uint8_t rows[7]; // 5 bits used per row
};

static const Glyph g_font[] = {

    // space
    { ' ', { 0x00,0x00,0x00,0x00,0x00,0x00,0x00 } },

    // numbers
    { '0', { 0x0E,0x11,0x13,0x15,0x19,0x11,0x0E } },
    { '1', { 0x04,0x0C,0x04,0x04,0x04,0x04,0x0E } },
    { '2', { 0x0E,0x11,0x01,0x02,0x04,0x08,0x1F } },
    { '3', { 0x1E,0x01,0x01,0x06,0x01,0x01,0x1E } },
    { '4', { 0x02,0x06,0x0A,0x12,0x1F,0x02,0x02 } },
    { '5', { 0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E } },
    { '6', { 0x06,0x08,0x10,0x1E,0x11,0x11,0x0E } },
    { '7', { 0x1F,0x01,0x02,0x04,0x08,0x08,0x08 } },
    { '8', { 0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E } },
    { '9', { 0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C } },

    // upper case letters A-Z
    { 'A', { 0x0E,0x11,0x11,0x1F,0x11,0x11,0x11 } },
    { 'B', { 0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E } },
    { 'C', { 0x0E,0x11,0x10,0x10,0x10,0x11,0x0E } },
    { 'D', { 0x1E,0x11,0x11,0x11,0x11,0x11,0x1E } },
    { 'E', { 0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F } },
    { 'F', { 0x1F,0x10,0x10,0x1E,0x10,0x10,0x10 } },
    { 'G', { 0x0E,0x11,0x10,0x17,0x11,0x11,0x0F } },
    { 'H', { 0x11,0x11,0x11,0x1F,0x11,0x11,0x11 } },
    { 'I', { 0x0E,0x04,0x04,0x04,0x04,0x04,0x0E } },
    { 'J', { 0x01,0x01,0x01,0x01,0x11,0x11,0x0E } },
    { 'K', { 0x11,0x12,0x14,0x18,0x14,0x12,0x11 } },
    { 'L', { 0x10,0x10,0x10,0x10,0x10,0x10,0x1F } },
    { 'M', { 0x11,0x1B,0x15,0x15,0x11,0x11,0x11 } },
    { 'N', { 0x11,0x19,0x15,0x13,0x11,0x11,0x11 } },
    { 'O', { 0x0E,0x11,0x11,0x11,0x11,0x11,0x0E } },
    { 'P', { 0x1E,0x11,0x11,0x1E,0x10,0x10,0x10 } },
    { 'Q', { 0x0E,0x11,0x11,0x11,0x15,0x12,0x0D } },
    { 'R', { 0x1E,0x11,0x11,0x1E,0x14,0x12,0x11 } },
    { 'S', { 0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E } },
    { 'T', { 0x1F,0x04,0x04,0x04,0x04,0x04,0x04 } },
    { 'U', { 0x11,0x11,0x11,0x11,0x11,0x11,0x0E } },
    { 'V', { 0x11,0x11,0x11,0x11,0x11,0x0A,0x04 } },
    { 'W', { 0x11,0x11,0x11,0x15,0x15,0x15,0x0A } },
    { 'X', { 0x11,0x11,0x0A,0x04,0x0A,0x11,0x11 } },
    { 'Y', { 0x11,0x11,0x0A,0x04,0x04,0x04,0x04 } },
    { 'Z', { 0x1F,0x01,0x02,0x04,0x08,0x10,0x1F } },

    // lower case letters a-z
    { 'a', { 0x00,0x00,0x0E,0x01,0x0F,0x11,0x0F } },
    { 'b', { 0x10,0x10,0x1E,0x11,0x11,0x11,0x1E } },
    { 'c', { 0x00,0x00,0x0E,0x10,0x10,0x10,0x0E } },
    { 'd', { 0x01,0x01,0x0F,0x11,0x11,0x11,0x0F } },
    { 'e', { 0x00,0x00,0x0E,0x11,0x1F,0x10,0x0E } },
    { 'f', { 0x06,0x08,0x1C,0x08,0x08,0x08,0x08 } },
    { 'g', { 0x00,0x00,0x0F,0x11,0x11,0x0F,0x01 } },
    { 'h', { 0x10,0x10,0x1E,0x11,0x11,0x11,0x11 } },
    { 'i', { 0x04,0x00,0x0C,0x04,0x04,0x04,0x0E } },
    { 'j', { 0x02,0x00,0x06,0x02,0x02,0x12,0x0C } },
    { 'k', { 0x10,0x10,0x12,0x14,0x18,0x14,0x12 } },
    { 'l', { 0x0C,0x04,0x04,0x04,0x04,0x04,0x0E } },
    { 'm', { 0x00,0x00,0x1A,0x15,0x15,0x11,0x11 } },
    { 'n', { 0x00,0x00,0x1E,0x11,0x11,0x11,0x11 } },
    { 'o', { 0x00,0x00,0x0E,0x11,0x11,0x11,0x0E } },
    { 'p', { 0x00,0x00,0x1E,0x11,0x11,0x1E,0x10 } },
    { 'q', { 0x00,0x00,0x0F,0x11,0x11,0x0F,0x01 } },
    { 'r', { 0x00,0x00,0x16,0x19,0x10,0x10,0x10 } },
    { 's', { 0x00,0x00,0x0F,0x10,0x0E,0x01,0x1E } },
    { 't', { 0x08,0x08,0x1C,0x08,0x08,0x09,0x06 } },
    { 'u', { 0x00,0x00,0x11,0x11,0x11,0x13,0x0D } },
    { 'v', { 0x00,0x00,0x11,0x11,0x11,0x0A,0x04 } },
    { 'w', { 0x00,0x00,0x11,0x11,0x15,0x15,0x0A } },
    { 'x', { 0x00,0x00,0x11,0x0A,0x04,0x0A,0x11 } },
    { 'y', { 0x00,0x00,0x11,0x11,0x0F,0x01,0x0E } },
    { 'z', { 0x00,0x00,0x1F,0x02,0x04,0x08,0x1F } },

    // symbols
    { '!', { 0x04,0x04,0x04,0x04,0x04,0x00,0x04 } },
    { '"', { 0x0A,0x0A,0x0A,0x00,0x00,0x00,0x00 } },
    { '#', { 0x0A,0x0A,0x1F,0x0A,0x1F,0x0A,0x0A } },
    { '$', { 0x04,0x0F,0x14,0x0E,0x05,0x1E,0x04 } },
    { '%', { 0x19,0x19,0x02,0x04,0x08,0x13,0x13 } },
    { '&', { 0x0C,0x12,0x14,0x08,0x15,0x12,0x0D } },
    { '\'', { 0x04,0x04,0x04,0x00,0x00,0x00,0x00 } },
    { '(', { 0x02,0x04,0x08,0x08,0x08,0x04,0x02 } },
    { ')', { 0x08,0x04,0x02,0x02,0x02,0x04,0x08 } },
    { '*', { 0x00,0x04,0x15,0x0E,0x15,0x04,0x00 } },
    { '+', { 0x00,0x04,0x04,0x1F,0x04,0x04,0x00 } },
    { ',', { 0x00,0x00,0x00,0x00,0x00,0x04,0x08 } },
    { '-', { 0x00,0x00,0x00,0x1F,0x00,0x00,0x00 } },
    { '.', { 0x00,0x00,0x00,0x00,0x00,0x06,0x06 } },
    { '/', { 0x01,0x02,0x02,0x04,0x08,0x08,0x10 } },
    { ':', { 0x00,0x04,0x00,0x00,0x00,0x04,0x00 } },
    { ';', { 0x00,0x04,0x00,0x00,0x00,0x04,0x08 } },
    { '<', { 0x02,0x04,0x08,0x10,0x08,0x04,0x02 } },
    { '=', { 0x00,0x1F,0x00,0x1F,0x00,0x00,0x00 } },
    { '>', { 0x08,0x04,0x02,0x01,0x02,0x04,0x08 } },
    { '?', { 0x0E,0x11,0x01,0x02,0x04,0x00,0x04 } },
    { '@', { 0x0E,0x11,0x01,0x0D,0x15,0x15,0x0E } },
    { '[', { 0x0E,0x08,0x08,0x08,0x08,0x08,0x0E } },
    { '\\', { 0x10,0x08,0x08,0x04,0x02,0x02,0x01 } },
    { ']', { 0x0E,0x02,0x02,0x02,0x02,0x02,0x0E } },
    { '^', { 0x04,0x0A,0x11,0x00,0x00,0x00,0x00 } },
    { '_', { 0x00,0x00,0x00,0x00,0x00,0x00,0x1F } },
};


static const Glyph* find_glyph(char c) {
    for (size_t i = 0; i < sizeof(g_font)/sizeof(g_font[0]); ++i) {
        if (g_font[i].ch == c)
            return &g_font[i];
    }
    return nullptr;
}

static void draw_char(GLuint progText,
                      GLint locScreenSize,
                      GLint locTextColor,
                      int screen_w,
                      int screen_h,
                      char c,
                      float x,
                      float y,
                      float scale)
{
    const Glyph* g = find_glyph(c);
    if (!g) return;

    glUseProgram(progText);

    glUniform2f(locScreenSize, (float)screen_w, (float)screen_h);
    glUniform4f(locTextColor, 1.0f, 1.0f, 1.0f, 1.0f);

    glEnableVertexAttribArray(0);

    const float pixel_w = 6.0f * scale;
    const float pixel_h = 6.0f * scale;

    for (int row = 0; row < 7; ++row) {
        uint8_t bits = g->rows[row];
        for (int col = 0; col < 5; ++col) {
            if (bits & (1 << (4 - col))) {
                float px = x + col * pixel_w;
                float py = y + row * pixel_h;

                GLfloat quad[8] = {
                    px,         py,
                    px + pixel_w, py,
                    px,         py + pixel_h,
                    px + pixel_w, py + pixel_h
                };

                glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, quad);
                glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
            }
        }
    }

    glDisableVertexAttribArray(0);
}

static void draw_text(GLuint progText,
                      GLint locScreenSize,
                      GLint locTextColor,
                      int screen_w,
                      int screen_h,
                      const char* msg,
                      float x,
                      float y,
                      float scale,
                      float char_w)   // NEW PARAMETER
{
    float cursor_x = x;

    for (const char* p = msg; *p; ++p) {
        if (*p == ' ') {
            cursor_x += char_w;   // spacing for space
            continue;
        }

        draw_char(progText,
                  locScreenSize,
                  locTextColor,
                  screen_w,
                  screen_h,
                  *p,
                  cursor_x,
                  y,
                  scale);

        cursor_x += char_w;   // consistent spacing between glyphs
    }
}


// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------

// Toast helpers

static double get_monotonic_seconds() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static void show_toast(const char* msg) {
    snprintf(g_toast_msg, sizeof(g_toast_msg), "%s", msg);
    g_toast_until = get_monotonic_seconds() + 1.5; // visible for 1.5s
}


static void render_toast_banner(GLuint progText,
                                GLint locScreenSize,
                                GLint locTextColor,
                                int screen_w,
                                int screen_h)
{
    double now = get_monotonic_seconds();
    if (now > g_toast_until || g_toast_msg[0] == '\0')
        return;

    const char* msg = g_toast_msg;
    size_t len = strlen(msg);

    float scale = 1.0f;

    // Glyph metrics
    float pixel_w = 8.0f * scale;   // width of one pixel block
    float pixel_h = 8.0f * scale;   // height of one pixel block

    float glyph_w   = pixel_w;          // glyph is 5 columns wide, but drawn column-by-column
    float spacing_w = pixel_w * 4.0f;   // spacing between glyphs
    float char_w    = glyph_w + spacing_w;

    // Total text width
    float text_width = len * char_w;

    // Padding around text
    float pad_x = 40.0f;
    float pad_y = 20.0f;

    // Banner size
    float banner_width  = text_width + pad_x * 2.0f;
    float banner_height = 7.0f * pixel_h + pad_y * 2.0f;

    // Banner position (top center)
    float center_x = screen_w * 0.5f;
    float banner_y = screen_h * 0.08f;

    float bx = center_x - banner_width * 0.5f;
    float by = banner_y - banner_height * 0.5f;

    // Draw background quad
    glUseProgram(progText);
    glUniform2f(locScreenSize, (float)screen_w, (float)screen_h);
    glUniform4f(locTextColor, 0.0f, 0.0f, 0.0f, 0.55f);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glEnableVertexAttribArray(0);

    GLfloat quad[] = {
        bx,               by,
        bx+banner_width,  by,
        bx,               by+banner_height,
        bx+banner_width,  by+banner_height
    };

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, quad);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

    glDisableVertexAttribArray(0);

    // Draw text inside box
    float text_x = bx + pad_x;
    float text_y = by + pad_y + 7.0f;

    draw_text(progText,
              locScreenSize,
              locTextColor,
              screen_w,
              screen_h,
              msg,
              text_x,
              text_y,
              scale,
              char_w);   // IMPORTANT: pass char_w for correct spacing

    glDisable(GL_BLEND);
}



// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------

static void apply_vsync(EGLDisplay egl_dpy) {
    eglSwapInterval(egl_dpy, g_vsync_enabled ? 1 : 0);
}

int main(int argc, char** argv) {
    int screen_w, screen_h;
    int internal_w, internal_h;

    // Default Vita capture resolution
    uint32_t vita_w = 960;
    uint32_t vita_h = 544;

    // Vita presets in the order F2 should cycle: 720 -> 544 -> 504 -> 488 -> 272
    const uint32_t vita_presets_w[] = { 1280, 960, 896, 864, 480 };
    const uint32_t vita_presets_h[] = { 720, 544, 504, 488, 272 };
    const int vita_presets_count = sizeof(vita_presets_w) / sizeof(vita_presets_w[0]);
    int vita_index = 1; // default index points to 544 (960x544)

    int rgbMode  = 0;  // 0 = full-range, 1 = limited (NV12 -> RGB)
    int postMode = 0;  // 0 = full-range, 1 = limited (RGB post-processing)
    int fps      = 30; // default FPS

    // filterMode: 0 = nearest (default), 1 = bilinear
    int filterMode = 0;

    // First pass: parse arguments
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--res-720")) {
            internal_w = 1280;
            internal_h = 720;
        } else if (!strcmp(argv[i], "--res-1080")) {
            internal_w = 1920;
            internal_h = 1080;
        } else if (!strcmp(argv[i], "--res-2160")) {
            internal_w = 3840;
            internal_h = 2160;
        } else if (!strcmp(argv[i], "--rgb-full")) {
            rgbMode = 0;
        } else if (!strcmp(argv[i], "--rgb-limited")) {
            rgbMode = 1;
        } else if (!strcmp(argv[i], "--post-full")) {
            postMode = 0;
        } else if (!strcmp(argv[i], "--post-limited")) {
            postMode = 1;
        } else if (!strcmp(argv[i], "--fps-30")) {
            fps = 30;
        } else if (!strcmp(argv[i], "--fps-60")) {
            fps = 60;
        } else if (!strcmp(argv[i], "--vita-720")) {
            vita_w = 1280;
            vita_h = 720;
        } else if (!strcmp(argv[i], "--vita-544")) {
            vita_w = 960;
            vita_h = 544;
        } else if (!strcmp(argv[i], "--vita-504")) {
            vita_w = 896;
            vita_h = 504;
        } else if (!strcmp(argv[i], "--vita-488")) {
            vita_w = 864;
            vita_h = 488;
        } else if (!strcmp(argv[i], "--vita-272")) {
            vita_w = 480;
            vita_h = 272;
        } else if (!strcmp(argv[i], "--filter-nearest")) {
            filterMode = 0;
        } else if (!strcmp(argv[i], "--filter-bilinear")) {
            filterMode = 1;
        } else if (!strcmp(argv[i], "--pi3bp")) {
            g_use_dmabuf = false;
            g_pi3_mode   = true;
            fprintf(stderr, "Pi3B+ mode enabled: using MMAP + glTexSubImage2D (no DMA-BUF)\n");
        } else if (!strcmp(argv[i], "--audio")) {
            g_enable_audio = true;
            fprintf(stderr, "Audio forwarding enabled (pre-app pw-loopback)\n");
        } else if (!strcmp(argv[i], "--vsync-off")) {
            g_vsync_enabled = false;
        } else if (!strcmp(argv[i], "--vsync-on")) {
            g_vsync_enabled = true;
        }
    }

    // Determine initial vita_index based on parsed --vita-XXX (defaults to 544)
    for (int i = 0; i < vita_presets_count; ++i) {
        if (vita_w == vita_presets_w[i] && vita_h == vita_presets_h[i]) {
            vita_index = i;
            break;
        }
    }

    // If audio is enabled, start pw-loopback BEFORE touching X11/EGL
    if (g_enable_audio) {
        pid_t child = fork();
        if (child == 0) {
            // child process: run pw-loopback
            execlp("pw-loopback", "pw-loopback", (char*)NULL);
            _exit(127);
        } else if (child < 0) {
            fprintf(stderr, "Audio: fork pw-loopback failed (%d: %s)\n", errno, strerror(errno));
        } else {
            g_loopback_pid = child;
            fprintf(stderr, "Audio: started pw-loopback (pid=%d) before app\n", child);
        }
    }

    Display* dpy = XOpenDisplay(NULL);
    if (!dpy) die("XOpenDisplay");

    int screen   = DefaultScreen(dpy);
    Window root  = RootWindow(dpy, screen);

    screen_w = DisplayWidth(dpy, screen);
    screen_h = DisplayHeight(dpy, screen);

    // If internal_w/h were not set by args, default to screen size
    if (internal_w <= 0 || internal_h <= 0) {
        internal_w = screen_w;
        internal_h = screen_h;
    }

    // Precompute uniforms based on rgbMode/postMode
    float y_scale, y_offset, uv_scale;
    float post_scale, post_offset;

    if (rgbMode == 1) { // limited input
        y_scale  = 255.0f / 219.0f;
        y_offset = -16.0f / 219.0f;
        uv_scale = 1.0f;
    } else { // full input
        y_scale  = 1.0f;
        y_offset = 0.0f;
        uv_scale = 1.0f;
    }

    if (postMode == 1) { // limited output
        post_scale  = 219.0f / 255.0f;
        post_offset = 16.0f / 255.0f;
    } else { // full output
        post_scale  = 1.0f;
        post_offset = 0.0f;
    }

    XSetWindowAttributes swa;
    swa.override_redirect = True;
    swa.event_mask = ExposureMask | KeyPressMask | StructureNotifyMask;

    Window win = XCreateWindow(
        dpy, root,
        0, 0, screen_w, screen_h, 0,
        CopyFromParent, InputOutput,
        CopyFromParent,
        CWOverrideRedirect | CWEventMask, &swa
    );

    XMapRaised(dpy, win);

    Atom wm_state      = XInternAtom(dpy, "_NET_WM_STATE", False);
    Atom wm_fullscreen = XInternAtom(dpy, "_NET_WM_STATE_FULLSCREEN", False);

    XEvent xev{};
    xev.type                 = ClientMessage;
    xev.xclient.window       = win;
    xev.xclient.message_type = wm_state;
    xev.xclient.format       = 32;
    xev.xclient.data.l[0]    = 1;
    xev.xclient.data.l[1]    = wm_fullscreen;
    xev.xclient.data.l[2]    = 0;
    XSendEvent(dpy, root, False, SubstructureNotifyMask, &xev);

    EGLDisplay egl_dpy = eglGetDisplay((EGLNativeDisplayType)dpy);
    if (egl_dpy == EGL_NO_DISPLAY) die("eglGetDisplay");

    if (!eglInitialize(egl_dpy, NULL, NULL)) die("eglInitialize");

    EGLint cfg_attrs[] = {
        EGL_SURFACE_TYPE,    EGL_WINDOW_BIT,
        EGL_RED_SIZE,        8,
        EGL_GREEN_SIZE,      8,
        EGL_BLUE_SIZE,       8,
        EGL_ALPHA_SIZE,      0,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
        EGL_NONE
    };

    EGLConfig config;
    EGLint num;
    if (!eglChooseConfig(egl_dpy, cfg_attrs, &config, 1, &num) || num < 1)
        die("eglChooseConfig");

    EGLint ctx_attrs[] = {
        EGL_CONTEXT_CLIENT_VERSION, 2,
        EGL_NONE
    };

    EGLContext ctx = eglCreateContext(egl_dpy, config, EGL_NO_CONTEXT, ctx_attrs);
    if (ctx == EGL_NO_CONTEXT) die("eglCreateContext");

    EGLSurface surf = eglCreateWindowSurface(egl_dpy, config, win, NULL);
    if (surf == EGL_NO_SURFACE) die("eglCreateWindowSurface");

    if (!eglMakeCurrent(egl_dpy, surf, surf, ctx))
        die("eglMakeCurrent");

//    eglSwapInterval(egl_dpy, g_vsync_enabled ? 1 : 0); // vsync
    apply_vsync(egl_dpy);



    PFNEGLCREATEIMAGEKHRPROC eglCreateImageKHR_fn =
        (PFNEGLCREATEIMAGEKHRPROC)eglGetProcAddress("eglCreateImageKHR");
    PFNEGLDESTROYIMAGEKHRPROC eglDestroyImageKHR_fn =
        (PFNEGLDESTROYIMAGEKHRPROC)eglGetProcAddress("eglDestroyImageKHR");
    PFNGLEGLIMAGETARGETTEXTURE2DOESPROC glEGLImageTargetTexture2DOES_fn =
        (PFNGLEGLIMAGETARGETTEXTURE2DOESPROC)eglGetProcAddress("glEGLImageTargetTexture2DOES");

    if (g_use_dmabuf) {
        if (!eglCreateImageKHR_fn || !eglDestroyImageKHR_fn || !glEGLImageTargetTexture2DOES_fn) {
            fprintf(stderr, "Required DMA-BUF/EGLImage extensions not available, falling back to MMAP path\n");
            g_use_dmabuf = false;
            g_pi3_mode   = true;
        }
    }

    V4L2State cam = init_v4l2_blocking("/dev/video0", fps, vita_w, vita_h);

    GLuint progYUV = create_program(fs_yuv_single_pass_src);
    GLuint progText = create_program_text();

    GLfloat vertsYUV[] = {
        -1.f, -1.f,  0.f, 1.f,
         1.f, -1.f,  1.f, 1.f,
        -1.f,  1.f,  0.f, 0.f,
         1.f,  1.f,  1.f, 0.f
    };

    GLint locY          = glGetUniformLocation(progYUV, "texY");
    GLint locUV         = glGetUniformLocation(progYUV, "texUV");
    GLint locYScale     = glGetUniformLocation(progYUV, "y_scale");
    GLint locYOffset    = glGetUniformLocation(progYUV, "y_offset");
    GLint locUVScale    = glGetUniformLocation(progYUV, "uv_scale");
    GLint locPostScale  = glGetUniformLocation(progYUV, "post_scale");
    GLint locPostOffset = glGetUniformLocation(progYUV, "post_offset");
    GLint locPi3Mode    = glGetUniformLocation(progYUV, "pi3_mode");

    GLint locScreenSize = glGetUniformLocation(progText, "uScreenSize");
    GLint locTextColor  = glGetUniformLocation(progText, "uColor");

    GLuint texY, texUV;
    glGenTextures(1, &texY);
    glGenTextures(1, &texUV);

    int uv_w = cam.width / 2;
    int uv_h = cam.height / 2;

    // Configure texture parameters + allocate storage
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texY);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    if (filterMode == 1) {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    } else {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    }

    if (g_pi3_mode) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE,
                     cam.width, cam.height, 0,
                     GL_LUMINANCE, GL_UNSIGNED_BYTE, NULL);
    } else {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RED_EXT,
                     cam.width, cam.height, 0,
                     GL_RED_EXT, GL_UNSIGNED_BYTE, NULL);
    }

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, texUV);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    if (filterMode == 1) {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    } else {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    }

    if (g_pi3_mode) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE_ALPHA,
                     uv_w, uv_h, 0,
                     GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, NULL);
    } else {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RG_EXT,
                     uv_w, uv_h, 0,
                     GL_RG_EXT, GL_UNSIGNED_BYTE, NULL);
    }

    bool running = true;

    while (running) {
        while (XPending(dpy)) {
            XEvent ev;
            XNextEvent(dpy, &ev);

            if (ev.type == KeyPress) {
                KeySym ks = XLookupKeysym(&ev.xkey, 0);

                if (ks == XK_F1) {
                    filterMode = (filterMode == 0) ? 1 : 0;

                    glBindTexture(GL_TEXTURE_2D, texY);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                                    filterMode == 1 ? GL_LINEAR : GL_NEAREST);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
                                    filterMode == 1 ? GL_LINEAR : GL_NEAREST);

                    glBindTexture(GL_TEXTURE_2D, texUV);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                                    filterMode == 1 ? GL_LINEAR : GL_NEAREST);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
                                    filterMode == 1 ? GL_LINEAR : GL_NEAREST);

                    fprintf(stderr, "Filter switched to %s\n",
                            filterMode == 1 ? "BILINEAR" : "NEAREST");

                    show_toast(filterMode == 1 ? "Filter: Bilinear" : "Filter: Nearest");
                }

                if (ks == XK_F2) {
                    vita_index = (vita_index + 1) % vita_presets_count;
                    vita_w = vita_presets_w[vita_index];
                    vita_h = vita_presets_h[vita_index];

                    fprintf(stderr, "Switching Vita capture to %ux%u\n", vita_w, vita_h);
                    restart_v4l2(cam, fps, vita_w, vita_h, uv_w, uv_h);

                    glBindTexture(GL_TEXTURE_2D, texY);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                                    filterMode == 1 ? GL_LINEAR : GL_NEAREST);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
                                    filterMode == 1 ? GL_LINEAR : GL_NEAREST);
                    if (g_pi3_mode) {
                        glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE,
                                     cam.width, cam.height, 0,
                                     GL_LUMINANCE, GL_UNSIGNED_BYTE, NULL);
                    } else {
                        glTexImage2D(GL_TEXTURE_2D, 0, GL_RED_EXT,
                                     cam.width, cam.height, 0,
                                     GL_RED_EXT, GL_UNSIGNED_BYTE, NULL);
                    }

                    glBindTexture(GL_TEXTURE_2D, texUV);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                                    filterMode == 1 ? GL_LINEAR : GL_NEAREST);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
                                    filterMode == 1 ? GL_LINEAR : GL_NEAREST);
                    if (g_pi3_mode) {
                        glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE_ALPHA,
                                     cam.width / 2, cam.height / 2, 0,
                                     GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, NULL);
                    } else {
                        glTexImage2D(GL_TEXTURE_2D, 0, GL_RG_EXT,
                                     cam.width / 2, cam.height / 2, 0,
                                     GL_RG_EXT, GL_UNSIGNED_BYTE, NULL);
                    }

                    fprintf(stderr, "Capture restarted for %ux%u (cam reports %ux%u)\n",
                            vita_w, vita_h, cam.width, cam.height);

                    char buf[64];
                    snprintf(buf, sizeof(buf), "Vita resolution: %ux%u", vita_w, vita_h);
                    show_toast(buf);
                }

                if (ks == XK_F3) {
                    fps = (fps == 30) ? 60 : 30;
                    fprintf(stderr, "Switching FPS to %d\n", fps);
                    restart_v4l2(cam, fps, vita_w, vita_h, uv_w, uv_h);

                    char buf[64];
                    snprintf(buf, sizeof(buf), "FPS: %d", fps);
                    show_toast(buf);
                }

                if (ks == XK_F4) {
                    rgbMode = (rgbMode == 0) ? 1 : 0;
                    if (rgbMode == 1) {
                        y_scale  = 255.0f / 219.0f;
                        y_offset = -16.0f / 219.0f;
                        uv_scale = 1.0f;
                    } else {
                        y_scale  = 1.0f;
                        y_offset = 0.0f;
                        uv_scale = 1.0f;
                    }

                    fprintf(stderr, "RGB input set to %s\n",
                            rgbMode == 1 ? "LIMITED" : "FULL");

                    glUseProgram(progYUV);
                    glUniform1f(locYScale,     y_scale);
                    glUniform1f(locYOffset,    y_offset);
                    glUniform1f(locUVScale,    uv_scale);
                    glUniform1f(locPostScale,  post_scale);
                    glUniform1f(locPostOffset, post_offset);
                    glUniform1i(locPi3Mode,    g_pi3_mode ? 1 : 0);

                    show_toast(rgbMode == 1 ? "Pre RGB: Limited" : "Pre RGB: Full");
                }

                if (ks == XK_F5) {
                    postMode = (postMode == 0) ? 1 : 0;
                    if (postMode == 1) {
                        post_scale  = 219.0f / 255.0f;
                        post_offset = 16.0f / 255.0f;
                    } else {
                        post_scale  = 1.0f;
                        post_offset = 0.0f;
                    }

                    fprintf(stderr, "Post RGB output set to %s\n",
                            postMode == 1 ? "LIMITED" : "FULL");

                    glUseProgram(progYUV);
                    glUniform1f(locPostScale,  post_scale);
                    glUniform1f(locPostOffset, post_offset);
                    glUniform1f(locYScale,     y_scale);
                    glUniform1f(locYOffset,    y_offset);
                    glUniform1f(locUVScale,    uv_scale);
                    glUniform1i(locPi3Mode,    g_pi3_mode ? 1 : 0);

                    show_toast(postMode == 1 ? "Post RGB: Limited" : "Post RGB: Full");
                }
                if (ks == XK_F6) {
                    g_vsync_enabled = !g_vsync_enabled;
                    apply_vsync(egl_dpy);

                    show_toast(g_vsync_enabled ? "VSync: ON" : "VSync: OFF");
                }

                if (ks == XK_Escape) {
                    running = false;
                }
            }
        }

        struct v4l2_buffer buf{};
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        int r;
        while (true) {
            r = ioctl(cam.fd, VIDIOC_DQBUF, &buf);
            if (r < 0 && errno == EAGAIN) {
                fd_set fds;
                FD_ZERO(&fds);
                FD_SET(cam.fd, &fds);

                struct timeval tv{};
                tv.tv_sec  = 0;
                tv.tv_usec = 1000;

                int s = select(cam.fd + 1, &fds, NULL, NULL, &tv);
                if (s <= 0) {
                    continue;
                }
                continue;
            }
            break;
        }

        if (r < 0) {
            fprintf(stderr, "VIDIOC_DQBUF error (%d: %s), showing black frame + restarting capture\n",
                    errno, strerror(errno));

            glViewport(0, 0, screen_w, screen_h);
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            render_toast_banner(progText, locScreenSize, locTextColor, screen_w, screen_h);
            eglSwapBuffers(egl_dpy, surf);

            restart_v4l2(cam, fps, vita_w, vita_h, uv_w, uv_h);
            continue;
        }

        if (buf.bytesused == 0) {
            fprintf(stderr, "Empty buffer from V4L2, showing black frame + restarting capture\n");

            glViewport(0, 0, screen_w, screen_h);
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            render_toast_banner(progText, locScreenSize, locTextColor, screen_w, screen_h);
            eglSwapBuffers(egl_dpy, surf);

            restart_v4l2(cam, fps, vita_w, vita_h, uv_w, uv_h);
            continue;
        }

        if (buf.index >= cam.buf_count) {
            fprintf(stderr, "Invalid buffer index %u, restarting capture\n", buf.index);

            glViewport(0, 0, screen_w, screen_h);
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            render_toast_banner(progText, locScreenSize, locTextColor, screen_w, screen_h);
            eglSwapBuffers(egl_dpy, surf);

            restart_v4l2(cam, fps, vita_w, vita_h, uv_w, uv_h);
            continue;
        }

        if (g_use_dmabuf) {
            if (cam.dmabuf_fds[buf.index] < 0) {
                fprintf(stderr, "Invalid DMA-BUF fd for buffer %u, restarting capture\n", buf.index);

                glViewport(0, 0, screen_w, screen_h);
                glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
                glClear(GL_COLOR_BUFFER_BIT);
                render_toast_banner(progText, locScreenSize, locTextColor, screen_w, screen_h);
                eglSwapBuffers(egl_dpy, surf);

                restart_v4l2(cam, fps, vita_w, vita_h, uv_w, uv_h);
                goto requeue;
            }

            int dma_fd = cam.dmabuf_fds[buf.index];

            EGLint y_attrs[] = {
                EGL_WIDTH,                  (EGLint)cam.width,
                EGL_HEIGHT,                 (EGLint)cam.height,
                EGL_LINUX_DRM_FOURCC_EXT,   DRM_FORMAT_R8,
                EGL_DMA_BUF_PLANE0_FD_EXT,  dma_fd,
                EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0,
                EGL_DMA_BUF_PLANE0_PITCH_EXT,  (EGLint)cam.y_stride,
                EGL_NONE
            };

            EGLint uv_offset = cam.y_stride * cam.height;
            EGLint uv_attrs[] = {
                EGL_WIDTH,                  (EGLint)uv_w,
                EGL_HEIGHT,                 (EGLint)uv_h,
                EGL_LINUX_DRM_FOURCC_EXT,   DRM_FORMAT_GR88,
                EGL_DMA_BUF_PLANE0_FD_EXT,  dma_fd,
                EGL_DMA_BUF_PLANE0_OFFSET_EXT, uv_offset,
                EGL_DMA_BUF_PLANE0_PITCH_EXT,  (EGLint)cam.uv_stride,
                EGL_NONE
            };

            EGLImageKHR imgY = eglCreateImageKHR_fn(
                egl_dpy,
                EGL_NO_CONTEXT,
                EGL_LINUX_DMA_BUF_EXT,
                (EGLClientBuffer)NULL,
                y_attrs
            );
            EGLImageKHR imgUV = eglCreateImageKHR_fn(
                egl_dpy,
                EGL_NO_CONTEXT,
                EGL_LINUX_DMA_BUF_EXT,
                (EGLClientBuffer)NULL,
                uv_attrs
            );

            if (imgY == EGL_NO_IMAGE_KHR || imgUV == EGL_NO_IMAGE_KHR) {
                fprintf(stderr, "eglCreateImageKHR (DMA-BUF) failed, restarting capture\n");

                if (imgY != EGL_NO_IMAGE_KHR) eglDestroyImageKHR_fn(egl_dpy, imgY);
                if (imgUV != EGL_NO_IMAGE_KHR) eglDestroyImageKHR_fn(egl_dpy, imgUV);

                glViewport(0, 0, screen_w, screen_h);
                glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
                glClear(GL_COLOR_BUFFER_BIT);
                render_toast_banner(progText, locScreenSize, locTextColor, screen_w, screen_h);
                eglSwapBuffers(egl_dpy, surf);

                restart_v4l2(cam, fps, vita_w, vita_h, uv_w, uv_h);
                goto requeue;
            }

            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, texY);
            glEGLImageTargetTexture2DOES_fn(GL_TEXTURE_2D, imgY);

            glActiveTexture(GL_TEXTURE1);
            glBindTexture(GL_TEXTURE_2D, texUV);
            glEGLImageTargetTexture2DOES_fn(GL_TEXTURE_2D, imgUV);

            glViewport(0, 0, screen_w, screen_h);
            glClear(GL_COLOR_BUFFER_BIT);

            glUseProgram(progYUV);
            glUniform1i(locY, 0);
            glUniform1i(locUV, 1);

            glUniform1f(locYScale,     y_scale);
            glUniform1f(locYOffset,    y_offset);
            glUniform1f(locUVScale,    uv_scale);
            glUniform1f(locPostScale,  post_scale);
            glUniform1f(locPostOffset, post_offset);
            glUniform1i(locPi3Mode,    g_pi3_mode ? 1 : 0);

            glEnableVertexAttribArray(0);
            glEnableVertexAttribArray(1);
            glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), vertsYUV);
            glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), vertsYUV + 2);

            glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

            // Toast banner overlay
            render_toast_banner(progText, locScreenSize, locTextColor, screen_w, screen_h);

            eglSwapBuffers(egl_dpy, surf);

            eglDestroyImageKHR_fn(egl_dpy, imgY);
            eglDestroyImageKHR_fn(egl_dpy, imgUV);
        } else {
            uint8_t* base = cam.base[buf.index];
            if (!base) {
                fprintf(stderr, "MMAP base pointer null for buffer %u, restarting capture\n", buf.index);

                glViewport(0, 0, screen_w, screen_h);
                glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
                glClear(GL_COLOR_BUFFER_BIT);
                render_toast_banner(progText, locScreenSize, locTextColor, screen_w, screen_h);
                eglSwapBuffers(egl_dpy, surf);

                restart_v4l2(cam, fps, vita_w, vita_h, uv_w, uv_h);
                goto requeue;
            }

            uint8_t* y_plane  = base;
            uint8_t* uv_plane = base + cam.y_stride * cam.height;

            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, texY);
            if (g_pi3_mode) {
                glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
                                cam.width, cam.height,
                                GL_LUMINANCE, GL_UNSIGNED_BYTE, y_plane);
            } else {
                glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
                                cam.width, cam.height,
                                GL_RED_EXT, GL_UNSIGNED_BYTE, y_plane);
            }

            glActiveTexture(GL_TEXTURE1);
            glBindTexture(GL_TEXTURE_2D, texUV);
            if (g_pi3_mode) {
                glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
                                uv_w, uv_h,
                                GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, uv_plane);
            } else {
                glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
                                uv_w, uv_h,
                                GL_RG_EXT, GL_UNSIGNED_BYTE, uv_plane);
            }

            glViewport(0, 0, screen_w, screen_h);
            glClear(GL_COLOR_BUFFER_BIT);

            glUseProgram(progYUV);
            glUniform1i(locY, 0);
            glUniform1i(locUV, 1);

            glUniform1f(locYScale,     y_scale);
            glUniform1f(locYOffset,    y_offset);
            glUniform1f(locUVScale,    uv_scale);
            glUniform1f(locPostScale,  post_scale);
            glUniform1f(locPostOffset, post_offset);
            glUniform1i(locPi3Mode,    g_pi3_mode ? 1 : 0);

            glEnableVertexAttribArray(0);
            glEnableVertexAttribArray(1);
            glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), vertsYUV);
            glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), vertsYUV + 2);

            glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

            // Toast banner overlay
            render_toast_banner(progText, locScreenSize, locTextColor, screen_w, screen_h);

            eglSwapBuffers(egl_dpy, surf);
        }

    requeue:
        if (ioctl(cam.fd, VIDIOC_QBUF, &buf) < 0) {
            fprintf(stderr, "VIDIOC_QBUF (requeue) failed (%d: %s), restarting capture\n",
                    errno, strerror(errno));

            glViewport(0, 0, screen_w, screen_h);
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            render_toast_banner(progText, locScreenSize, locTextColor, screen_w, screen_h);
            eglSwapBuffers(egl_dpy, surf);

            restart_v4l2(cam, fps, vita_w, vita_h, uv_w, uv_h);
            continue;
        }
    }

    // Stop pw-loopback if we started it
    if (g_enable_audio && g_loopback_pid > 0) {
        kill(g_loopback_pid, SIGTERM);
        waitpid(g_loopback_pid, NULL, 0);
        fprintf(stderr, "Audio: pw-loopback stopped\n");
        g_loopback_pid = 0;
    }

    // Cleanup V4L2
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(cam.fd, VIDIOC_STREAMOFF, &type);

    for (uint32_t i = 0; i < cam.buf_count; ++i) {
        if (cam.dmabuf_fds[i] >= 0) {
            close(cam.dmabuf_fds[i]);
            cam.dmabuf_fds[i] = -1;
        }
        if (cam.base[i]) {
            munmap(cam.base[i], cam.buf_size[i]);
            cam.base[i]     = nullptr;
            cam.buf_size[i] = 0;
        }
    }

    if (cam.fd >= 0) {
        close(cam.fd);
        cam.fd = -1;
    }

    // Cleanup GL/EGL/X11
    glDeleteTextures(1, &texY);
    glDeleteTextures(1, &texUV);
    glDeleteProgram(progYUV);
    glDeleteProgram(progText);

    eglMakeCurrent(egl_dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    eglDestroySurface(egl_dpy, surf);
    eglDestroyContext(egl_dpy, ctx);
    eglTerminate(egl_dpy);

    XDestroyWindow(dpy, win);
    XCloseDisplay(dpy);

    return 0;
}
