#include <gtest/gtest.h>
extern "C" {
#include "osem.h"
#include "iso22133.h"
}

class EncodeOSEM : public ::testing::Test
{
protected:
	EncodeOSEM() {
		// IDs
		settings.desiredID.transmitter = 0x1234;
		settings.desiredID.subTransmitter = 0x5678;
		setTransmitterID(0x9ABC);
		// Origin
		settings.coordinateSystemOrigin.latitude_deg = 12.3456789012;
		settings.coordinateSystemOrigin.longitude_deg = 23.4567890123;
		settings.coordinateSystemOrigin.altitude_m = 123.45;
		settings.coordinateSystemOrigin.isLatitudeValid = true;
		settings.coordinateSystemOrigin.isLongitudeValid = true;
		settings.coordinateSystemOrigin.isAltitudeValid = true;
		settings.coordinateSystemRotation_rad = 0.45678;
		settings.coordinateSystemType = COORDINATE_SYSTEM_WGS84;
		// Friday, April 29, 2022 2:22:22 AM
		settings.currentTime.tv_sec = 1651198942;
		settings.currentTime.tv_usec = 0;
		// Requirements
		settings.maxDeviation.position_m = 0.123;
		settings.maxDeviation.lateral_m = 0.456;
		settings.maxDeviation.yaw_rad = 0.789;
		settings.minRequiredPositioningAccuracy_m = 0.12;
		settings.heabTimeout.tv_sec = 1;
		settings.heabTimeout.tv_usec = 20000;
		settings.testMode = TEST_MODE_SCENARIO;
		settings.rate.monr = 4;
		settings.rate.monr2 = 5;
		settings.rate.heab = 6;

		settings.timeServer.ip = 0x12345678;
		settings.timeServer.port = 0x9ABC;
	}
	void SetUp() override
	{
		auto res = encodeOSEMMessage(&settings, encodeBuffer,
			sizeof(encodeBuffer), false);
		ASSERT_GT(res, 0);
	}
	ObjectSettingsType settings;
	char encodeBuffer[1024];
	char* id = encodeBuffer + 18; // skip header
	char* origin = id + 16;	// skip ID
	char* dateTime = origin + 23; // skip origin
	char* accReq = dateTime + 15; // skip date time
	char* timeServer = accReq + 22; // skip accuracy requirements
};

TEST_F(EncodeOSEM, IdStructPreamble)
{
	EXPECT_EQ(id[0], '\x20');
	EXPECT_EQ(id[1], '\x00');
	EXPECT_EQ(id[2], '\x0C');
	EXPECT_EQ(id[3], '\x00');
}

TEST_F(EncodeOSEM, DeviceId)
{
	EXPECT_EQ(id[4], '\x34');
	EXPECT_EQ(id[5], '\x12');
	EXPECT_EQ(id[6], '\x00');
	EXPECT_EQ(id[7], '\x00');
}

TEST_F(EncodeOSEM, SubDeviceId)
{
	EXPECT_EQ(id[8], '\x78');
	EXPECT_EQ(id[9], '\x56');
	EXPECT_EQ(id[10], '\x00');
	EXPECT_EQ(id[11], '\x00');
}

TEST_F(EncodeOSEM, ControlCentreId)
{
	EXPECT_EQ(id[12], '\xBC');
	EXPECT_EQ(id[13], '\x9A');
	EXPECT_EQ(id[14], '\x00');
	EXPECT_EQ(id[15], '\x00');
}

TEST_F(EncodeOSEM, OriginStructPreamble)
{
	EXPECT_EQ(origin[0], '\x21');
	EXPECT_EQ(origin[1], '\x00');
	EXPECT_EQ(origin[2], '\x13');
	EXPECT_EQ(origin[3], '\x00');
}

TEST_F(EncodeOSEM, Latitude)
{
	// 123456789012 nd = 0x001CBE991A14
	EXPECT_EQ(origin[4], '\x14');
	EXPECT_EQ(origin[5], '\x1A');
	EXPECT_EQ(origin[6], '\x99');
	EXPECT_EQ(origin[7], '\xBE');
	EXPECT_EQ(origin[8], '\x1C');
	EXPECT_EQ(origin[9], '\x00');
}

TEST_F(EncodeOSEM, Longitude)
{
	// 23456789012 nd = 0x00369D55F4CB
	EXPECT_EQ(origin[10], '\xCB');
	EXPECT_EQ(origin[11], '\xF4');
	EXPECT_EQ(origin[12], '\x55');
	EXPECT_EQ(origin[13], '\x9D');
	EXPECT_EQ(origin[14], '\x36');
	EXPECT_EQ(origin[15], '\x00');
}

TEST_F(EncodeOSEM, Altitude)
{
	// 12345 cm = 0x00003039
	EXPECT_EQ(origin[16], '\x39');
	EXPECT_EQ(origin[17], '\x30');
	EXPECT_EQ(origin[18], '\x00');
	EXPECT_EQ(origin[19], '\x00');
}

TEST_F(EncodeOSEM, Rotation)
{
	// 0.45678 rad = 2617 cd = 0x0A39
	EXPECT_EQ(origin[20], '\x39');
	EXPECT_EQ(origin[21], '\x0A');

	EXPECT_EQ(origin[22], '\x03');
}

TEST_F(EncodeOSEM, DateTimeStructPreamble)
{
	EXPECT_EQ(dateTime[0], '\x22');
	EXPECT_EQ(dateTime[1], '\x00');
	EXPECT_EQ(dateTime[2], '\x0B');
	EXPECT_EQ(dateTime[3], '\x00');
}

TEST_F(EncodeOSEM, Date)
{
	// 2022-04-29 02:22:22
	// 20220429 = 0x01348A0D
	EXPECT_EQ(dateTime[4], '\x0D');
	EXPECT_EQ(dateTime[5], '\x8A');
	EXPECT_EQ(dateTime[6], '\x34');
	EXPECT_EQ(dateTime[7], '\x01');
}

TEST_F(EncodeOSEM, GPSWeek)
{
	// GPS week 2207 = 0x089F
	EXPECT_EQ(dateTime[8], '\x9F');
	EXPECT_EQ(dateTime[9], '\x08');
}

TEST_F(EncodeOSEM, GPSSOW)
{
	// GPS second of week 440560
	// GPS qmsec of week 1762240000 = 0x6909A600
	EXPECT_EQ(dateTime[10], '\x00');
	EXPECT_EQ(dateTime[11], '\xA6');
	EXPECT_EQ(dateTime[12], '\x09');
	EXPECT_EQ(dateTime[13], '\x69');
}

TEST_F(EncodeOSEM, LeapSeconds)
{
	// 18 = 0x12
	EXPECT_EQ(dateTime[14], '\x12');
}

TEST_F(EncodeOSEM, AccReqStructPreamble)
{
	EXPECT_EQ(accReq[0], '\x23');
	EXPECT_EQ(accReq[1], '\x00');
	EXPECT_EQ(accReq[2], '\x12');
	EXPECT_EQ(accReq[3], '\x00');
}

TEST_F(EncodeOSEM, MaxWayDeviation)
{
	// 123 mm = 0x007B
	EXPECT_EQ(accReq[4], '\x7B');
	EXPECT_EQ(accReq[5], '\x00');
}

TEST_F(EncodeOSEM, MaxLateralDeviation)
{
	// 456 mm = 0x01C8
	EXPECT_EQ(accReq[6], '\xC8');
	EXPECT_EQ(accReq[7], '\x01');
}

TEST_F(EncodeOSEM, MaxYawDeviation)
{
	// 0.789 rad = 4520 cd rounded down = 0x11A8
	EXPECT_EQ(accReq[8], '\xA8');
	EXPECT_EQ(accReq[9], '\x11');
}

TEST_F(EncodeOSEM, MinPosAcc)
{
	// 12 cm = 0x000C
	EXPECT_EQ(accReq[10], '\x0C');
	EXPECT_EQ(accReq[11], '\x00');
}

TEST_F(EncodeOSEM, HeabTimeout)
{
	// 1.020 sec = 102 cs = 0x0066
	EXPECT_EQ(accReq[12], '\x66');
	EXPECT_EQ(accReq[13], '\x00');
}

TEST_F(EncodeOSEM, TestMode)
{
	// Test mode scenario
	EXPECT_EQ(accReq[14], '\x02');
}

TEST_F(EncodeOSEM, MessageRates)
{
	// 4 Hz = 0x04
	EXPECT_EQ(accReq[15], '\x04');
	// 5 Hz = 0x05
	EXPECT_EQ(accReq[16], '\x05');
	// 6 Hz = 0x06
	EXPECT_EQ(accReq[17], '\x06');
	// Message length not specified
}

TEST_F(EncodeOSEM, TimeServerStructPreamble)
{
	EXPECT_EQ(timeServer[0], '\x24');
	EXPECT_EQ(timeServer[1], '\x00');
	EXPECT_EQ(timeServer[2], '\x06');
	EXPECT_EQ(timeServer[3], '\x00');
}

TEST_F(EncodeOSEM, TimeServerIP)
{
	EXPECT_EQ(timeServer[4], '\x78');
	EXPECT_EQ(timeServer[5], '\x56');
	EXPECT_EQ(timeServer[6], '\x34');
	EXPECT_EQ(timeServer[7], '\x12');
}

TEST_F(EncodeOSEM, TimeServerPort)
{
	EXPECT_EQ(timeServer[8], '\xBC');
	EXPECT_EQ(timeServer[9], '\x9A');
}

TEST_F(EncodeOSEM, NoTimeServerStruct)
{
	settings.timeServer.ip = 0;
	settings.timeServer.port = 0;
	timeServer[0] = 0;
	timeServer[1] = 0;
	timeServer[2] = 0;
	timeServer[3] = 0;
	auto res = encodeOSEMMessage(&settings, encodeBuffer,
		sizeof(encodeBuffer), false);
	ASSERT_GT(res, 0);
	EXPECT_NE(timeServer[0], '\x24');
	EXPECT_NE(timeServer[1], '\x00');
	EXPECT_NE(timeServer[2], '\x06');
	EXPECT_NE(timeServer[3], '\x00');
}
