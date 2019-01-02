/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * media_device.h - Media device handler
 */
#ifndef __LIBCAMERA_MEDIA_DEVICE_H__
#define __LIBCAMERA_MEDIA_DEVICE_H__

#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <linux/media.h>

#include "media_object.h"

namespace libcamera {

class MediaDevice
{
public:
	MediaDevice(const std::string &devnode);
	~MediaDevice();

	bool acquire();
	void release() { acquired_ = false; }
	bool busy() const { return acquired_; }

	int open();
	void close();

	int populate();
	bool valid() const { return valid_; }

	const std::string driver() const { return driver_; }
	const std::string devnode() const { return devnode_; }

	const std::vector<MediaEntity *> &entities() const { return entities_; }
	MediaEntity *getEntityByName(const std::string &name) const;

private:
	std::string driver_;
	std::string devnode_;
	int fd_;
	bool valid_;
	bool acquired_;

	std::map<unsigned int, MediaObject *> objects_;
	MediaObject *object(unsigned int id);
	bool addObject(MediaObject *object);
	void clear();

	std::vector<MediaEntity *> entities_;

	struct media_v2_interface *findInterface(const struct media_v2_topology &topology,
						 unsigned int entityId);
	bool populateEntities(const struct media_v2_topology &topology);
	bool populatePads(const struct media_v2_topology &topology);
	bool populateLinks(const struct media_v2_topology &topology);
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_MEDIA_DEVICE_H__ */