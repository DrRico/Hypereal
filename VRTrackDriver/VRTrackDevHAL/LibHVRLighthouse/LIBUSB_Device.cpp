/*
 * The MIT License (MIT)
 * Copyright (c) 2017 Shanghai Chai Ming Huang Info&Tech Co£¬Ltd
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include "LIBUSB_Device.h"

//#define TEST_ASYNC

LIBUSB_Device::LIBUSB_Device(uint16_t vid, uint16_t pid, uint8_t ep_in, uint8_t ep_out,
	uint8_t config_index, uint8_t interface_index)
{
	DevHandle = NULL;
	VendorID = vid;
	ProductID = pid;
	BulkEpIn = ep_in;
	BulkEpOut = ep_out;
	ConfigIndex = config_index;
	InterfaceIndex = interface_index;

	usb_init();
}

LIBUSB_Device::~LIBUSB_Device()
{
	DevHandle = NULL;
}

int32_t LIBUSB_Device::Open()
{
	bool devFound = false;
	struct usb_bus *bus;
	struct usb_device *dev;

	usb_find_busses();
	usb_find_devices();

	for (bus = usb_get_busses(); bus; bus = bus->next){
		for (dev = bus->devices; dev; dev = dev->next){
			if (dev->descriptor.idVendor == VendorID &&
				dev->descriptor.idProduct == ProductID){
				DevHandle = usb_open(dev);
				if (DevHandle != NULL)
				{
					devFound = true;
					break;
				}
			}
		}
		if (devFound)
			break;
	}

	if (devFound){
		if (usb_set_configuration(DevHandle, ConfigIndex) < 0){
			debug_print("error setting config #%d: %s\n", ConfigIndex, usb_strerror());
			usb_close(DevHandle);
			return -1;
		}

		if (usb_claim_interface(DevHandle, InterfaceIndex) < 0){
			debug_print("error claiming interface #%d:\n%s\n", InterfaceIndex, usb_strerror());
			usb_close(DevHandle);
			return -1;
		}
		return 0;
	}

	return -1;
}


int32_t LIBUSB_Device::Open(char *productString)
{
	if (productString == NULL)
		return Open();

	bool devFound = false;
	struct usb_bus *bus;
	struct usb_device *dev;

	usb_find_busses();
	usb_find_devices();

	for (bus = usb_get_busses(); bus; bus = bus->next){
		for (dev = bus->devices; dev; dev = dev->next){
			if (dev->descriptor.idVendor == VendorID &&
				dev->descriptor.idProduct == ProductID){
				DevHandle = usb_open(dev);
				if (DevHandle == NULL)
					continue;
				char temp[256];
				usb_get_string_simple(DevHandle, dev->descriptor.iProduct, temp, 256);
				if (strcmp(productString, temp) == 0)
				{
					devFound = true;
					break;;
				}
				else
				{
					usb_close(DevHandle);
				}
			}
		}
		if (devFound)
			break;
	}

	if (devFound){
		if (usb_set_configuration(DevHandle, ConfigIndex) < 0){
			debug_print("error setting config #%d: %s\n", ConfigIndex, usb_strerror());
			usb_close(DevHandle);
			return -1;
		}

		if (usb_claim_interface(DevHandle, InterfaceIndex) < 0){
			debug_print("error claiming interface #%d:\n%s\n", InterfaceIndex, usb_strerror());
			usb_close(DevHandle);
			return -1;
		}
		return 0;
	}

	return -1;
}

void LIBUSB_Device::Close(void)
{
	if (DevHandle)
		usb_close(DevHandle);
}

int32_t LIBUSB_Device::BulkWrite(char *buf, uint32_t length)
{
	int ret;
#ifdef TEST_ASYNC
	// Each async transfer requires it's own context. A transfer
	// context can be re-used.  When no longer needed they must be
	// freed with usb_free_async().
	void* async_context = NULL;

	// Setup the async transfer.  This only needs to be done once
	// for multiple submit/reaps. (more below)
	ret = usb_bulk_setup_async(DevHandle, &async_context, BulkEpOut);
	if (ret < 0){
		debug_print("error usb_bulk_setup_async:\n%s\n", usb_strerror());
		return ret;
	}

	// Submit this transfer.  This function returns immediately and the
	// transfer is on it's way to the device.
	ret = usb_submit_async(async_context, buf, length);
	if (ret < 0){
		debug_print("error usb_submit_async:\n%s\n", usb_strerror());
		usb_free_async(&async_context);
		return ret;
	}

	// Wait for the transfer to complete.  If it doesn't complete in the
	// specified time it is cancelled.  see also usb_reap_async_nocancel().
	//
	ret = usb_reap_async(async_context, BulkXferTimeOut);

	// Free the context.
	usb_free_async(&async_context);

	return ret;
#else

	ret = usb_bulk_write(DevHandle, BulkEpOut, buf, length, BulkXferTimeOut);
	if (ret != length){
		debug_print("error writing:\n%s\n", usb_strerror());
		return ret;
	}
	else
		return 0;
#endif
}


int32_t LIBUSB_Device::BulkRead(char *buf, uint32_t length)
{
	int ret;
#ifdef TEST_ASYNC
	// Each async transfer requires it's own context. A transfer
	// context can be re-used.  When no longer needed they must be
	// freed with usb_free_async().
	void* async_context = NULL;

	// Setup the async transfer.  This only needs to be done once
	// for multiple submit/reaps. (more below)
	ret = usb_bulk_setup_async(DevHandle, &async_context, BulkEpIn);
	if (ret < 0){
		debug_print("error usb_bulk_setup_async:\n%s\n", usb_strerror());
		return ret;
	}

	// Submit this transfer.  This function returns immediately and the
	// transfer is on it's way to the device.
	ret = usb_submit_async(async_context, buf, length);
	if (ret < 0){
		debug_print("error usb_submit_async:\n%s\n", usb_strerror());
		usb_free_async(&async_context);
		return ret;
	}

	// Wait for the transfer to complete.  If it doesn't complete in the
	// specified time it is cancelled.  see also usb_reap_async_nocancel().
	ret = usb_reap_async(async_context, BulkXferTimeOut);

	// Free the context.
	usb_free_async(&async_context);

	return ret;
#else

	ret = usb_bulk_read(DevHandle, BulkEpIn, buf, length, BulkXferTimeOut);
	if (ret < 0){
		debug_print("error reading:\n%s\n", usb_strerror());
	}

	return ret;
#endif
}
