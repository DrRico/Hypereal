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

#ifndef HVR_COMMON_H__
#define HVR_COMMON_H__

#include <stdint.h>
#include <Windows.h>

#define DllExport __declspec(dllexport)

//! Max size of a command packet
const uint16_t CMD_PACKET_MAX_SIZE = 256;

//! Max size of a received packet
const uint16_t RX_PACKET_MAX_SIZE = 1024;

//! One light sensor report type
typedef struct LightSensorData_ {
	uint8_t		SensorID;		//!< sensor ID
	uint8_t		TriggerCount;	//!< light trigger count
	uint16_t	MeasureResult;	//!< ligth scan measure result
}LightSensorData;

/********************************************************
* Version releated
********************************************************/
#pragma pack(push)
#pragma pack(1)
//! MCU firmware version type
typedef struct FirmwareVersion_ {
	uint8_t Major;	//!< major version
	uint8_t Minor;	//!< minor version
}FirmwareVersion;
#pragma pack(pop)

/********************************************************
* Data Packet releated
********************************************************/
//! Data Packet:
/*!
	Header(2) | Length(2) | Type(2) | Payload Data(Length) | CRC(1) | Tail(2)
*/

//! Header's magic number
const uint16_t MAGIC_NUMBER_HEAD	= 0x2B2B;

//! Tail's magic number
const uint16_t MAGIC_NUMBER_TAIL	= 0xB2B2;

#pragma pack(push)
#pragma pack(1)
//! Data packet head type
typedef struct DataPacketHead_ {
	uint16_t MagicNumber;	//!< Magic number
	uint16_t Length;		//!< Length of payload data
	uint16_t Type;			//!< Packet type
}DataPacketHead;

//! Data packet tail type
typedef struct DataPacketTail_ {
	uint8_t Crc;			//!< Accumulation addition of payload data
	uint16_t MagicNumber;	//!< Magic number
}DataPacketTail;

//! Data packet type
typedef struct DataPacketStruct_ {
	DataPacketHead Head;	//!< Packet header
	uint8_t *Data;			//!< Packet body(payload data)
	DataPacketTail Tail;	//!< Packet tail
} DataPacketStruct;
#pragma pack(pop)

//! Get packet total length
/*!
	\param len Length of payload data
	\return Size of the whole packet
*/
static inline uint16_t GetPacketTotalLength(uint16_t len)
{
	return len + sizeof(DataPacketHead) + sizeof(DataPacketTail);
}

//! Assemble one packet
/*!
	\param buf Pointer to the buffer which will store the whole packet, and it should NOT be NULL.
	\param packetType Type of this packet
	\param cmdData Pointer to the payload data, and it should NOT be NULL
	\param cmdLength Lenght of payload data
	return Size of the whole packet
*/
static inline uint16_t AssemblePacket(char *buf, uint16_t packetType, uint8_t *cmdData, uint16_t cmdLength)
{
	uint16_t offset = 0;

	if (!buf || !cmdData)
		return 0;

	DataPacketStruct Pack;
	Pack.Head.MagicNumber = MAGIC_NUMBER_HEAD;
	Pack.Head.Length = cmdLength;
	Pack.Head.Type = packetType;
	Pack.Data = cmdData;
	Pack.Tail.Crc = 0;
	Pack.Tail.MagicNumber = MAGIC_NUMBER_TAIL;

	// Copy Head
	memcpy_s(buf, sizeof(Pack.Head), &(Pack.Head), sizeof(Pack.Head));
	offset += sizeof(Pack.Head);

	// Copy Data
	memcpy_s(buf + offset, Pack.Head.Length, Pack.Data, Pack.Head.Length);
	offset += Pack.Head.Length;

	// Caculate CRC Byte;
	for (int i = 0; i < Pack.Head.Length; i++)
		Pack.Tail.Crc += (uint8_t)Pack.Data[i];

	// Copy Tail
	memcpy_s(buf + offset, sizeof(Pack.Tail), &(Pack.Tail), sizeof(Pack.Tail));
	offset += sizeof(Pack.Tail);

	return offset;
}

#endif	// HVR_COMMON_H__