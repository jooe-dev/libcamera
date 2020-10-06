/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C)
 *
 * drm_allocator.h - DRM frameBuffer allocator
 */
#ifndef __LIBCAMERA_DRMFRAMEBUFFER_ALLOCATOR_H__
#define __LIBCAMERA_DRMFRAMEBUFFER_ALLOCATOR_H__

#include <linux/videodev2.h>

#include <map>
#include <memory>
#include <vector>

#include <libcamera/framebuffer_allocator.h>


using namespace libcamera;

struct drm_buffer {
	unsigned int index;
	unsigned int bo_handle;
	unsigned int fb_handle;
	int dbuf_fd;
};


class DRMManager
{
public:
	DRMManager(const std::string &module, unsigned int crt, unsigned int plane, const struct v4l2_pix_format fmt);
	int open();
	std::unique_ptr<FrameBuffer> createBuffer(int fd, int length);
	int createBuffers(unsigned int count,
                                   std::vector<std::unique_ptr<FrameBuffer>> *buffers);
	int createDRMBuffer(unsigned int *bo_handle, unsigned int *fb_handle, int *buf_fd, unsigned int *length);
	int pageFlip(FrameBuffer *buffer);
	~DRMManager();

	const std::string &module() const { return module_; }

private:
	std::string module_;
	int crt_;
	int plane_;
	v4l2_pix_format fmt_;
	int fd_;
	std::map<int, drm_buffer> drm_buffers_;
};


class DRMFrameBufferAllocator
{
public:
	DRMFrameBufferAllocator(std::shared_ptr<Camera> camera, DRMManager *manager);
	DRMFrameBufferAllocator(const Camera &) = delete;
	DRMFrameBufferAllocator &operator=(const Camera &) = delete;

	~DRMFrameBufferAllocator();

	int allocate(Stream *stream);
	int free(Stream *stream);

	bool allocated() const { return !buffers_.empty(); }
	const std::vector<std::unique_ptr<FrameBuffer>> &buffers(Stream *stream) const;

private:
	std::shared_ptr<Camera> camera_;
	DRMManager * manager_;
	std::map<Stream *, std::vector<std::unique_ptr<FrameBuffer>>> buffers_;
};

#endif /* __LIBCAMERA_DRMFRAMEBUFFER_ALLOCATOR_H__ */
