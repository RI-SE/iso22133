#include <gtest/gtest.h>
extern "C" {
#include "header.h"
}

class HeaderDecode : public ::testing::Test
{
protected:
	HeaderDecode() {
		*reinterpret_cast<uint16_t*>(message) = htole16(0x7E7F);			// Sync word
		*reinterpret_cast<uint32_t*>(message + 2) = htole32(0x12345678);	// Message length
		*reinterpret_cast<uint8_t*>(message + 6) = 0x02;					// Ack req protocol version
		*reinterpret_cast<uint32_t*>(message + 7) = htole32(0xBCDEF012);	// Transmitter ID
		*reinterpret_cast<uint32_t*>(message + 11) = htole32(0x3456789A);	// Receiver ID
		*reinterpret_cast<uint8_t*>(message + 15) = 0xBC;					// Message counter
		*reinterpret_cast<uint16_t*>(message + 16) = htole16(0xDEF0);		// Message ID

	}
	void SetUp() override {
		auto ret = decodeISOHeader(message, 18, &header, false);
		ASSERT_EQ(MESSAGE_OK, ret);
	}
	virtual ~HeaderDecode();
	char message[18];
	HeaderType header;
};
HeaderDecode::~HeaderDecode() {}

TEST_F(HeaderDecode, SyncWord) {
	EXPECT_EQ(0x7E7F, header.syncWord);
}

TEST_F(HeaderDecode, MessageLength) {
	EXPECT_EQ(0x12345678, header.messageLength);
}

TEST_F(HeaderDecode, AckReq) {
	EXPECT_EQ(0x02, header.ackReqProtVer);
}

TEST_F(HeaderDecode, TransmitterID) {
	EXPECT_EQ(0xBCDEF012, header.transmitterID);
}

TEST_F(HeaderDecode, ReceiverID) {
	EXPECT_EQ(0x3456789A, header.receiverID);
}

TEST_F(HeaderDecode, MessageCounter) {
	EXPECT_EQ(0xBC, header.messageCounter);
}

TEST_F(HeaderDecode, MessageID) {
	EXPECT_EQ(0xDEF0, header.messageID);
}
