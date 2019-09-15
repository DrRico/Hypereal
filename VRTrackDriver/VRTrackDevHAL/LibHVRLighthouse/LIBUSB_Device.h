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

#ifndef LIBUSB_DEVICE_H__
#define LIBUSB_DEVICE_H__

#include <lusb0_usb.h>
#include <vector>
#include <stdint.h>

#ifdef  _DEBUG
#define debug_print(...) fprintf(stderr, __VA_ARGS__)
#else
#define debug_print(...)
#endif //  _DEBUG

/*!
	\class LIBUSB_Device
	\brief class of LIBUSB device

	Application should create a LIBUSB_Device object before communicating with USB device.
*/
class LIBUSB_Device
{
public:
	//! Constructed function
	/*!
		\param vid USB vendor ID
		\param pid USB product ID
		\param ep_in USB endpoint in
		\param ep_out USB endpoint out
		\param config_index USB configuration index
		\param interface_index USB interface index
	*/
	LIBUSB_Device(uint16_t vid, uint16_t pid, uint8_t ep_in, uint8_t ep_out,
		uint8_t config_index, uint8_t interface_index);

	//! Destructor function
	~LIBUSB_Device();

	//! Open lib usb device
	/*!
		\return
			- 0 on success
			- -1 on error
	*/
	int32_t Open(void);

	//! Open lib usb device with specified product string
	/*!
		\return
			- 0 on success
			- -1 on error
	*/
	int32_t Open(char *productString);

	//! Close HMD device
	void Close(void);

	//! Bulk write function
	/*!
		\param buf Pointer to data which will transfer to device
		\param length Length of buf
		\return
			- 0 on success
			- others on error
	*/
	int32_t BulkWrite(char *buf, uint32_t length);

	//! Bulk read function
	/*!
		\param buf Pointer to store data
		\param length Length of data will be read from device
		\return
			- 0 on success
			- others on error
	*/
	int32_t BulkRead(char *buf, uint32_t length);

private:
	//! Pointer to usb dev
	usb_dev_handle *DevHandle;

	//! USB vendor id
	uint16_t VendorID;

	//! USB product id
	uint16_t ProductID;

	//! Bulk in endpoint
	uint8_t BulkEpIn;

	//! Bulk out endpoint
	uint8_t BulkEpOut;

	//! USB configuration index
	uint8_t ConfigIndex;

	//! USB interface index
	uint8_t InterfaceIndex;

	//! Timeout of bulk transmission
	const int32_t BulkXferTimeOut = 5000;
};

#endif	// LIBUSB_DEVICE_H__