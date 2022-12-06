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

	char* origin = id + 16;	// skip ID
	char* dateTime = origin + 23; // skip origin
	char* accReq = dateTime + 15; // skip date time
	char* timeServer = accReq + 22; // skip accuracy requirements
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
	EXPECT_EQ(name[0], '\0x02');
	EXPECT_EQ(name[1], '\0x01');
	EXPECT_EQ(name[2], '\0x40');
	EXPECT_EQ(name[3], '\0x00');
	EXPECT_EQ(0, memcmp(name + 4, "some description", 17));
	// Check that remaining bytes are zero
	for (int i = 21; i < 68; i++) {
		EXPECT_EQ(name[i], '\0');
	}
}
