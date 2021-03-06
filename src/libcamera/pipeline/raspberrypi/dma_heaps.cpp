/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Limited
 *
 * dma_heaps.h - Helper class for dma-heap allocations.
 */

#include "dma_heaps.h"

#include <array>
#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "libcamera/internal/log.h"

/*
 * /dev/dma-heap/linux,cma is the dma-heap allocator, which allows dmaheap-cma
 * to only have to worry about importing.
 *
 * Annoyingly, should the cma heap size be specified on the kernel command line
 * instead of DT, the heap gets named "reserved" instead.
 */
static constexpr std::array<const char *, 2> heapNames = {
	"/dev/dma_heap/linux,cma",
	"/dev/dma_heap/reserved"
};

namespace libcamera {

LOG_DECLARE_CATEGORY(RPI)

namespace RPi {

DmaHeap::DmaHeap()
	: dmaHeapHandle_(-1)
{
	for (const char *name : heapNames) {
		int ret = ::open(name, O_RDWR, 0);
		if (ret < 0) {
			ret = errno;
			LOG(RPI, Debug) << "Failed to open " << name << ": "
					<< strerror(ret);
			continue;
		}

		dmaHeapHandle_ = ret;
		break;
	}

	if (dmaHeapHandle_ < 0)
		LOG(RPI, Error) << "Could not open any dmaHeap device";
}

DmaHeap::~DmaHeap()
{
	if (dmaHeapHandle_ > -1)
		::close(dmaHeapHandle_);
}

FileDescriptor DmaHeap::alloc(const char *name, std::size_t size)
{
	int ret;

	if (!name)
		return FileDescriptor();

	struct dma_heap_allocation_data alloc = {};

	alloc.len = size;
	alloc.fd_flags = O_CLOEXEC | O_RDWR;

	ret = ::ioctl(dmaHeapHandle_, DMA_HEAP_IOCTL_ALLOC, &alloc);

	if (ret < 0) {
		LOG(RPI, Error) << "dmaHeap allocation failure for "
				<< name;
		return FileDescriptor();
	}

	ret = ::ioctl(alloc.fd, DMA_BUF_SET_NAME, name);
	if (ret < 0) {
		LOG(RPI, Error) << "dmaHeap naming failure for "
				<< name;
		::close(alloc.fd);
		return FileDescriptor();
	}

	return FileDescriptor(std::move(alloc.fd));
}

} /* namespace RPi */

} /* namespace libcamera */
