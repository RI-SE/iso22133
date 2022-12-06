#include "traj.h"
#include <gtest/gtest.h>

class EncodeTRAJHeader : public ::testing::Test
{
protected:
	EncodeTRAJHeader() {
	}
	void SetUp() override
	{
		char name[] = "some description";
		memset(encodeBuffer, 0, sizeof(encodeBuffer));
		auto res = encodeTRAJMessageHeader(
			0x0123,
			TRAJECTORY_INFO_RELATIVE_TO_ORIGIN,
			name,
			sizeof(name),
			21,
			encodeBuffer,
			sizeof(encodeBuffer),
			false);
		ASSERT_GT(res, 0);
	}

	ObjectSettingsType settings;
	char encodeBuffer[1024];
	char* id = encodeBuffer + 18; // skip header
	char* info = id + 6;	// skip ID
	char* name = info + 5;	// skip info
};

TEST_F(EncodeTRAJHeader, Id)
{
	EXPECT_EQ(id[0], '\x01');
	EXPECT_EQ(id[1], '\x01');
	EXPECT_EQ(id[2], '\x02');
	EXPECT_EQ(id[3], '\x00');
	EXPECT_EQ(id[4], '\x23');
	EXPECT_EQ(id[5], '\x01');
}

TEST_F(EncodeTRAJHeader, Info)
{
	EXPECT_EQ(info[0], '\x04');
	EXPECT_EQ(info[1], '\x01');
	EXPECT_EQ(info[2], '\x01');
	EXPECT_EQ(info[3], '\x00');
	EXPECT_EQ(info[4], '\x02');
}

TEST_F(EncodeTRAJHeader, Name)
{
	EXPECT_EQ(name[0], '\x02');
	EXPECT_EQ(name[1], '\x01');
	EXPECT_EQ(name[2], '\x40');
	EXPECT_EQ(name[3], '\x00');
	EXPECT_EQ(0, memcmp(name + 4, "some description", 17));
	// Check that remaining bytes are zero
	for (int i = 21; i < 68; i++) {
		EXPECT_EQ(name[i], '\0');
	}
}

class DecodeTRAJHeader : public ::testing::Test
{
protected:
	DecodeTRAJHeader() {
		decodeBuffer[0] = 0x7F;
		decodeBuffer[1] = 0x7E;	// preamble
		decodeBuffer[2] = 0x52;
		decodeBuffer[3] = 0x00;
		decodeBuffer[4] = 0x00;
		decodeBuffer[5] = 0x00;	 // TODO Message length
		decodeBuffer[6] = 0x02;	 // Acknowledge protocol version
		decodeBuffer[7] = 0x34;
		decodeBuffer[8] = 0x12;
		decodeBuffer[9] = 0x00;
		decodeBuffer[10] = 0x00;	 // Transmitter ID
		decodeBuffer[11] = 0x78;
		decodeBuffer[12] = 0x56;
		decodeBuffer[13] = 0x00;
		decodeBuffer[14] = 0x00;	 // Receiver ID
		decodeBuffer[15] = 0x00;	 // Message count
		decodeBuffer[16] = 0x01;
		decodeBuffer[17] = 0x00;	 // Message ID

		decodeBuffer[18] = 0x01;
		decodeBuffer[19] = 0x01;
		decodeBuffer[20] = 0x02;
		decodeBuffer[21] = 0x00;	 // ID preamble
		decodeBuffer[22] = 0x23;
		decodeBuffer[23] = 0x01;

		decodeBuffer[24] = 0x04;
		decodeBuffer[25] = 0x01;
		decodeBuffer[26] = 0x01;
		decodeBuffer[27] = 0x00;	 // Info preamble
		decodeBuffer[28] = 0x02;

		decodeBuffer[29] = 0x02;
		decodeBuffer[30] = 0x01;
		decodeBuffer[31] = 0x40;
		decodeBuffer[32] = 0x00;	 // Name preamble
		decodeBuffer[33] = 's';
		decodeBuffer[34] = 'o';
		decodeBuffer[35] = 'm';
		decodeBuffer[36] = 'e';
		decodeBuffer[37] = ' ';
		decodeBuffer[38] = 'd';
		decodeBuffer[39] = 'e';
		decodeBuffer[40] = 's';
		decodeBuffer[41] = 'c';
		decodeBuffer[42] = 'r';
		decodeBuffer[43] = 'i';
		decodeBuffer[44] = 'p';
		decodeBuffer[45] = 't';
		decodeBuffer[46] = 'i';
		decodeBuffer[47] = 'o';
		decodeBuffer[48] = 'n';
		decodeBuffer[49] = '\0';
		for (int i = 50; i < 29+68; i++) {
			decodeBuffer[i] = '\0';
		}
	}

	void SetUp() override
	{
		memset(&header, 0, sizeof(header));
		auto res = decodeTRAJMessageHeader(
			&header,
			decodeBuffer,
			sizeof(decodeBuffer),
			false);
		ASSERT_GT(res, 0);
	}

	TrajectoryHeaderType header;
	char decodeBuffer[1024];
};

TEST_F(DecodeTRAJHeader, Id)
{
	EXPECT_EQ(header.trajectoryID, 0x0123);
}

TEST_F(DecodeTRAJHeader, Info)
{
	EXPECT_EQ(header.trajectoryInfo, TRAJECTORY_INFO_RELATIVE_TO_ORIGIN);
}

TEST_F(DecodeTRAJHeader, Name)
{
	EXPECT_EQ(0, memcmp(header.trajectoryName, "some description", 17));
	for (int i = 17; i < 64; i++) {
		EXPECT_EQ(header.trajectoryName[i], '\0');
	}
}
