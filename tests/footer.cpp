#include <gtest/gtest.h>
extern "C" {
#include "footer.h"
}

TEST(FooterDecode, Crc) {
	char message[2];
	*reinterpret_cast<uint16_t*>(message) = htole16(0x1234);
	FooterType footer;
	auto ret = decodeISOFooter(message, 2, &footer, false);
	ASSERT_EQ(MESSAGE_OK, ret);
	EXPECT_EQ(0x1234, footer.Crc);
}
