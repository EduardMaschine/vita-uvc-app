// ============================================================================
// Real‑time NV12 to RGB renderer for Raspberry Pi (Pi3B+, Pi4, Pi5)
// Uses V4L2 capture + EGL + GLES2 + X11 fullscreen window
//
// Two capture paths:
//   • Pi4/Pi5: DMA‑BUF zero‑copy import via EGLImage + glEGLImageTargetTexture2DOES
//   • Pi3B+:   MMAP + CPU upload via glTexSubImage2D
//
// Command‑line switches:
//   --res-720 / --res-1080 / --res-2160   -> internal render resolution
//   --rgb-full / --rgb-limited            -> input YUV range
//   --post-full / --post-limited          -> output RGB range
//   --fps-30 / --fps-60                   -> capture FPS
//   --vita-272 / 488 / 504 / 544 / 720    -> capture resolution presets
//   --filter-nearest / --filter-bilinear  -> filter options
//   --pi3bp                               -> force Pi3B+ MMAP path (fixes compatibility with pi3b+)
//   --audio                               -> enables audio output from USB over to HDMI by using pw-loopback. (Thanks to the VitaUSBStream plugin for the Vita)
//
// Keyboard:
//   F1 	-> 	NEAREST filter
//   F2 	-> 	BILINEAR filter
//   ESC 	-> 	exit
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

#define MAX_V4L2_BUFFERS 4

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

static bool g_use_dmabuf   = true;   // default: Pi4/Pi5
static bool g_pi3_mode     = false;  // set by --pi3bp
static bool g_enable_audio = false;  // set by --audio
static pid_t g_loopback_pid = 0;

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
        req.count  = 3; // triple buffering
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

static const char* vs_src = R"(
attribute vec2 aPos;
attribute vec2 aTex;
varying vec2 vTex;
void main() {
    vTex = aTex;
    gl_Position = vec4(aPos, 0.0, 1.0);
}
)";

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

int main(int argc, char** argv) {
    int screen_w, screen_h;
    int internal_w, internal_h;

    // Default Vita capture resolution
    uint32_t vita_w = 960;
    uint32_t vita_h = 544;

    int rgbMode  = 0;  // 0 = full-range, 1 = limited (NV12 -> RGB)
    int postMode = 0;  // 0 = full-range, 1 = limited (RGB post-processing)
    int fps      = 30; // default FPS

    // filterMode: 0 = nearest (default), 1 = bilinear
    int filterMode = 0;

    // First pass: parse arguments (including --audio)
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
        }
    }

    // If audio is enabled, start pw-loopback BEFORE touching X11/EGL
    if (g_enable_audio) {
        pid_t child = fork();
        if (child == 0) {
            // child process: run pw-loopback
            execlp("pw-loopback", "pw-loopback", "-C", "73", "-P", "74", (char*)NULL);
//            execlp("pw-loopback", "pw-loopback", "-C", "73", "-P", "74", "--capture-channel-map", "FL,FR", "--playback-channel-map", "FL,FR", (char*)NULL);
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

    eglSwapInterval(egl_dpy, 0);

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
                    filterMode = 0;

                    glBindTexture(GL_TEXTURE_2D, texY);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

                    glBindTexture(GL_TEXTURE_2D, texUV);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

                    fprintf(stderr, "Filter switched to NEAREST\n");
                }

                if (ks == XK_F2) {
                    filterMode = 1;

                    glBindTexture(GL_TEXTURE_2D, texY);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

                    glBindTexture(GL_TEXTURE_2D, texUV);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

                    fprintf(stderr, "Filter switched to BILINEAR\n");
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
            eglSwapBuffers(egl_dpy, surf);

            restart_v4l2(cam, fps, vita_w, vita_h, uv_w, uv_h);
            continue;
        }

        if (buf.bytesused == 0) {
            fprintf(stderr, "Empty buffer from V4L2, showing black frame + restarting capture\n");

            glViewport(0, 0, screen_w, screen_h);
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            eglSwapBuffers(egl_dpy, surf);

            restart_v4l2(cam, fps, vita_w, vita_h, uv_w, uv_h);
            continue;
        }

        if (buf.index >= cam.buf_count) {
            fprintf(stderr, "Invalid buffer index %u, restarting capture\n", buf.index);

            glViewport(0, 0, screen_w, screen_h);
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
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

            eglSwapBuffers(egl_dpy, surf);
        }

    requeue:
        if (ioctl(cam.fd, VIDIOC_QBUF, &buf) < 0) {
            fprintf(stderr, "VIDIOC_QBUF (requeue) failed (%d: %s), restarting capture\n",
                    errno, strerror(errno));

            glViewport(0, 0, screen_w, screen_h);
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
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

    eglMakeCurrent(egl_dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    eglDestroySurface(egl_dpy, surf);
    eglDestroyContext(egl_dpy, ctx);
    eglTerminate(egl_dpy);

    XDestroyWindow(dpy, win);
    XCloseDisplay(dpy);

    return 0;
}
