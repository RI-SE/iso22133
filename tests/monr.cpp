#include "monr.h"

#include <gtest/gtest.h>

class EncodeMONR : public ::testing::Test {
protected:
	EncodeMONR()
	{
		// Friday, April 29, 2022 2:22:22 AM
		objTime.tv_sec = 1651198942;
		objTime.tv_usec = 0;
		
		pos.xCoord_m = 1.0;
		pos.yCoord_m = -2.0;
		pos.zCoord_m = 3.0;
		pos.isXcoordValid = true;
		pos.isYcoordValid = true;
		pos.isZcoordValid = true;
		pos.isPositionValid = true;
		pos.heading_rad = 0.4;
		pos.isHeadingValid = true;

		spd.longitudinal_m_s = 1.0;
		spd.lateral_m_s = 2.0;
		spd.isLongitudinalValid = true;
		spd.isLateralValid = true;

		acc.longitudinal_m_s2 = 1.0;
		acc.lateral_m_s2 = 2.0;
		acc.isLongitudinalValid = true;
		acc.isLateralValid = true;

		driveDir = DriveDirectionType::OBJECT_DRIVE_DIRECTION_FORWARD;
		objState = ObjectStateType::OBJECT_STATE_RUNNING;
		readyToArm = ObjectArmReadinessType::OBJECT_READY_TO_ARM;

		errState.outsideGeofence = true;
		errState.badPositioningAccuracy = true;

		errCode = 0xBEEF;
	}
	void SetUp() override
	{
		memset(encodeBuffer, 0, sizeof(encodeBuffer));
		auto res = encodeMONRMessage(
			&objTime,
			pos,
			spd,
			acc,
			driveDir,
			objState,
			readyToArm,
			0b01101011,
			errCode,
			encodeBuffer,
			sizeof(encodeBuffer),
			false);
		ASSERT_GT(res, 0);
	}
	char encodeBuffer[1024];
	struct timeval objTime;
	CartesianPosition pos;
	SpeedType spd;
	AccelerationType acc;
	DriveDirectionType driveDir;
	ObjectStateType objState;
	ObjectArmReadinessType readyToArm;
	ObjectErrorType errState;
	unsigned short errCode;

	char* preamble = encodeBuffer + 18; // Skip header
	char* monrStruct = preamble + 4; // Skip preamble

};


TEST_F(EncodeMONR, Preamble)
{
	EXPECT_EQ(preamble[0], '\x80');
	EXPECT_EQ(preamble[1], '\x00');
	EXPECT_EQ(preamble[2], '\x24');
	EXPECT_EQ(preamble[3], '\x00');
}

TEST_F(EncodeMONR, GPSSecondOfWeek)
{
	// GPS second of week 440560
	// GPS qmsec of week 1762240000 = 0x6909A600
	EXPECT_EQ(monrStruct[0], '\x00');
	EXPECT_EQ(monrStruct[1], '\xA6');
	EXPECT_EQ(monrStruct[2], '\x09');
	EXPECT_EQ(monrStruct[3], '\x69');
}

TEST_F(EncodeMONR, XPosition)
{
	// 1.0 m = 0x000003E8 mm
	EXPECT_EQ(monrStruct[4], '\xE8');
	EXPECT_EQ(monrStruct[5], '\x03');
	EXPECT_EQ(monrStruct[6], '\x00');
	EXPECT_EQ(monrStruct[7], '\x00');
}

TEST_F(EncodeMONR, YPosition)
{
	// -2.0 m = 0xFFFFF830 mm
	EXPECT_EQ(monrStruct[8], '\x30');
	EXPECT_EQ(monrStruct[9], '\xF8');
	EXPECT_EQ(monrStruct[10], '\xFF');
	EXPECT_EQ(monrStruct[11], '\xFF');
}

TEST_F(EncodeMONR, ZPosition)
{
	// 3.0 m = 0x00000BB8 mm
	EXPECT_EQ(monrStruct[12], '\xB8');
	EXPECT_EQ(monrStruct[13], '\x0B');
	EXPECT_EQ(monrStruct[14], '\x00');
	EXPECT_EQ(monrStruct[15], '\x00');
}

TEST_F(EncodeMONR, Yaw)
{
	// 0.4 rad = 2291 centidegrees rounded down = 0x08f3
	EXPECT_EQ(monrStruct[16], '\xF3');
	EXPECT_EQ(monrStruct[17], '\x08');
}

TEST_F(EncodeMONR, Pitch)
{
	EXPECT_EQ(monrStruct[18], '\x00');
	EXPECT_EQ(monrStruct[19], '\x00');
}

TEST_F(EncodeMONR, Roll)
{
	EXPECT_EQ(monrStruct[20], '\x00');
	EXPECT_EQ(monrStruct[21], '\x00');
}

TEST_F(EncodeMONR, LongitudinalSpeed)
{
	// 1.0 m/s = 0x00000064 cm/s
	EXPECT_EQ(monrStruct[22], '\x64');
	EXPECT_EQ(monrStruct[23], '\x00');
}

TEST_F(EncodeMONR, LateralSpeed)
{
	// 2.0 m/s = 0x000000C8 cm/s
	EXPECT_EQ(monrStruct[24], '\xC8');
	EXPECT_EQ(monrStruct[25], '\x00');
}

TEST_F(EncodeMONR, LongitudinalAcceleration)
{
	// 1.0 m/s^2 = 0x000003E8 mm/s^2
	EXPECT_EQ(monrStruct[26], '\xE8');
	EXPECT_EQ(monrStruct[27], '\x03');
}

TEST_F(EncodeMONR, LateralAcceleration)
{
	// 2.0 m/s^2 = 0x000007D0 mm/s^2
	EXPECT_EQ(monrStruct[28], '\xD0');
	EXPECT_EQ(monrStruct[29], '\x07');
}

TEST_F(EncodeMONR, DriveDirection)
{
	EXPECT_EQ(monrStruct[30], '\x00');
}

TEST_F(EncodeMONR, ObjectState)
{
	EXPECT_EQ(monrStruct[31], '\x04');
}

TEST_F(EncodeMONR, ReadyToArm)
{
	EXPECT_EQ(monrStruct[32], '\x01');
}

TEST_F(EncodeMONR, ErrorState)
{
	EXPECT_EQ(monrStruct[33], 0b01101011);
}

TEST_F(EncodeMONR, ErrorCode)
{
	EXPECT_EQ(monrStruct[34], '\xEF');
	EXPECT_EQ(monrStruct[35], '\xBE');
}
