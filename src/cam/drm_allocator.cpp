/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C)
 *
 * drm_allocator.cpp - DRM frameBuffer allocator
 */

#include <iostream>

#include "drm_allocator.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <drm.h>
#include <drm_mode.h>

#include <linux/videodev2.h>

#include <xf86drm.h>
#include <xf86drmMode.h>

#include <libcamera/buffer.h>
#include <libcamera/camera.h>
#include <libcamera/stream.h>

#include "libcamera/internal/log.h"

#define ERRSTR strerror(errno)

#define BYE_ON(cond, ...) \
do { \
        if (cond) { \
                int errsv = errno; \
                fprintf(stderr, "ERROR(%s:%d) : ", \
                        __FILE__, __LINE__); \
                errno = errsv; \
                fprintf(stderr,  __VA_ARGS__); \
                abort(); \
        } \
} while(0)

static inline int warn(const char *file, int line, const char *fmt, ...)
{
        int errsv = errno;
        va_list va;
        va_start(va, fmt);
        fprintf(stderr, "WARN(%s:%d): ", file, line);
        vfprintf(stderr, fmt, va);
        va_end(va);
        errno = errsv;
        return 1;
}

#define WARN_ON(cond, ...) \
        ((cond) ? warn(__FILE__, __LINE__, __VA_ARGS__) : 0)



/**
 * \file framebuffer_allocator.h
 * \brief FrameBuffer allocator
 */

using namespace libcamera;

LOG_DEFINE_CATEGORY(Allocator)

DRMManager::DRMManager(const std::string &module, unsigned int plane, unsigned int crt, const struct v4l2_pix_format fmt)
	: module_(module), crt_(crt), plane_(plane), fmt_(fmt), fd_(-1)
{
	std::cout << "DRMManager module: " << module_ << " plane@crt: " << plane_ << "@" << crt_ << std::endl;
}

int DRMManager::open(void)
{
	int ret = drmOpen(module_.c_str(), NULL);
	if (ret < 0) {
                ret = -errno;
                std::cout << "Failed to open V4L2 device: "
                                 << strerror(-ret) << std::endl;
                return ret;
        }
        fd_ = ret;
        return 0;
}

std::unique_ptr<FrameBuffer> DRMManager::createBuffer(int fd, int length)
{
        std::vector<FrameBuffer::Plane> planes;

        FileDescriptor ffdd(std::move(fd));
        FrameBuffer::Plane plane;
        plane.fd = std::move(ffdd);
        plane.length = length;

        planes.push_back(std::move(plane));

        return std::make_unique<FrameBuffer>(std::move(planes));
}

int DRMManager::pageFlip(FrameBuffer *buffer)
{
        int ret;

        if (plane_) {
		int dbuf_fd = buffer->planes()[0].fd.fd();
		drm_buffer drm_b;
		drm_b = drm_buffers_[dbuf_fd];

                ret = drmModeSetPlane(fd_, plane_, crt_,
                                      drm_b.fb_handle, 0,
                                      0, 0,
                                      fmt_.width, fmt_.height,
                                      0, 0, fmt_.width << 16, fmt_.height << 16);
                BYE_ON(ret, "drmModeSetPlane failed: %s\n", ERRSTR);

#if 0
                drmVBlank vblank;
                vblank.request.type = DRM_VBLANK_EVENT | DRM_VBLANK_RELATIVE;
                vblank.request.sequence = 1;
                vblank.request.signal = (unsigned long)buffer.index;
                ret = drmWaitVBlank(dev->fd, &vblank);
                BYE_ON(ret, "drmWaitVBlank failed: %s\n", ERRSTR);
#endif

        }
#if 0
        else {
                ret = drmModePageFlip(dev->fd, dev->crtc_id, buffer.fb_handle,
                        DRM_MODE_PAGE_FLIP_EVENT, (void*)(unsigned long)buffer.index);
                BYE_ON(ret, "drmModePageFlip failed: %s\n", ERRSTR);
        }
#endif
	return 0;
}


int DRMManager::createBuffers(unsigned int count,
                                   std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
        int ret;

        buffers->clear();
        drm_buffers_.clear();

        for (unsigned i = 0; i < count; ++i) {

                drm_buffer drm_b;
                unsigned int length;

                ret = createDRMBuffer(&drm_b.bo_handle, &drm_b.fb_handle, &drm_b.dbuf_fd, &length);
                if (ret) {
                        LOG(Allocator, Error) << "Unable to create DRM buffer";
                        buffers->clear();
                        drm_buffers_.clear();
                        return -1;
                }

                std::unique_ptr<FrameBuffer> buffer = createBuffer(drm_b.dbuf_fd, length);
                if (!buffer) {
                        LOG(Allocator, Error) << "Unable to create buffer";
                        buffers->clear();
                        drm_buffers_.clear();
                        return -EINVAL;
                }

                buffers->push_back(std::move(buffer));

                drm_b.index = i;
                drm_buffers_.insert(std::pair<int, drm_buffer>(drm_b.dbuf_fd, drm_b));

        }

        return count;
}

int DRMManager::createDRMBuffer(unsigned int *bo_handle, unsigned int *fb_handle, int *buf_fd, unsigned int *length)
{
        struct drm_mode_create_dumb gem;
        struct drm_mode_destroy_dumb gem_destroy;
        int ret;

        *length = fmt_.width * fmt_.height * 4; /* test only: hardcoded */
        memset(&gem, 0, sizeof gem);
        gem.width = fmt_.width;
        gem.height = fmt_.height;
        gem.bpp = fmt_.bytesperline / fmt_.width * 8; /* test only: hardcoded */

        gem.flags = (1 << 1); //RKCAM_BO_CONTIG | RKCAM_BO_CACHEABLE;

        ret = ioctl(fd_, DRM_IOCTL_MODE_CREATE_DUMB, &gem);
        if (WARN_ON(ret, "CREATE_DUMB failed: %s\n", ERRSTR))
                return -1;

        *bo_handle = gem.handle;

        struct drm_prime_handle prime;
        memset(&prime, 0, sizeof prime);
        prime.handle = *bo_handle;

        ret = ioctl(fd_, DRM_IOCTL_PRIME_HANDLE_TO_FD, &prime);
        if (WARN_ON(ret, "PRIME_HANDLE_TO_FD failed: %s\n", ERRSTR)) {
                memset(&gem_destroy, 0, sizeof gem_destroy);
                gem_destroy.handle = *bo_handle;
                ret = ioctl(fd_, DRM_IOCTL_MODE_DESTROY_DUMB, &gem_destroy);
                WARN_ON(ret, "DESTROY_DUMB failed: %s\n", ERRSTR);
                return -1;
        }

        *buf_fd = prime.fd;

        uint32_t offsets[4] = { 0 };
        uint32_t pitches[4] = { fmt_.bytesperline };
        uint32_t bo_handles[4] = { *bo_handle };
        unsigned int fourcc = fmt_.pixelformat;
        ret = drmModeAddFB2(fd_, fmt_.width, fmt_.height, fourcc, bo_handles,
                pitches, offsets, fb_handle, 0);
        if (WARN_ON(ret, "drmModeAddFB2 failed: %s\n", ERRSTR)) {
                close(*buf_fd);
                memset(&gem_destroy, 0, sizeof gem_destroy);
                gem_destroy.handle = *bo_handle;
                ret = ioctl(fd_, DRM_IOCTL_MODE_DESTROY_DUMB, &gem_destroy);
                WARN_ON(ret, "DESTROY_DUMB failed: %s\n", ERRSTR);
                return -1;
        }

        return 0;
}


DRMManager::~DRMManager()
{
}

DRMFrameBufferAllocator::DRMFrameBufferAllocator(std::shared_ptr<Camera> camera, DRMManager *manager)
	: camera_(camera), manager_(manager)
{
}

DRMFrameBufferAllocator::~DRMFrameBufferAllocator()
{
	buffers_.clear();
}

int DRMFrameBufferAllocator::allocate(Stream *stream)
{
	if (buffers_.count(stream)) {
		LOG(Allocator, Error) << "Buffers already allocated for stream";
		return -EBUSY;
	}

	LOG(Allocator, Info) << "Buffer count: " << stream->configuration().bufferCount;
	LOG(Allocator, Info) << "Size: " << stream->configuration().size.width << "x" << stream->configuration().size.height;
	LOG(Allocator, Info) << "Pixel format: " << stream->configuration().pixelFormat.toString();

        int ret = manager_->open();
        if (ret) {
		LOG(Allocator, Error)
			<< "DRM create buffers: failure opening drm device";
		return ret;
        }

        ret = manager_->createBuffers(stream->configuration().bufferCount,
                                   &buffers_[stream]);
        if (ret != (int)stream->configuration().bufferCount) {
		LOG(Allocator, Error)
			<< "DRM create buffers: Stream is not part of active configuration";
		return ret;

        }

#if 1
        /* note: see src/libcamera/pipeline/raspberrypi/raspberrypi.cpp
           modification implementing logic required to import buffers,
           otherwise no buffers are returned by the camera */
	ret = camera_->exportFrameBuffers(stream, &buffers_[stream]);
	if (ret == -EINVAL) {
		LOG(Allocator, Error)
			<< "Stream is not part of active configuration";
		return ret;
	}
#endif
	return 0;
}

int DRMFrameBufferAllocator::free(Stream *stream)
{
	auto iter = buffers_.find(stream);
	if (iter == buffers_.end())
		return -EINVAL;

	std::vector<std::unique_ptr<FrameBuffer>> &buffers = iter->second;
	buffers.clear();
	buffers_.erase(iter);

	return 0;
}

const std::vector<std::unique_ptr<FrameBuffer>> &
DRMFrameBufferAllocator::buffers(Stream *stream) const
{
	static const std::vector<std::unique_ptr<FrameBuffer>> empty;

	auto iter = buffers_.find(stream);
	if (iter == buffers_.end())
		return empty;

	return iter->second;
}
