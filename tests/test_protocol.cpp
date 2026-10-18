#include "motor/protocol.hpp"

#include <cstring>
#include <gtest/gtest.h>

namespace {

motor::Frame feed_line(motor::Parser& parser, const char* bytes) {
    motor::Frame last{};
    bool finished = false;
    for (const char* p = bytes; *p != '\0'; ++p) {
        motor::Frame frame;
        if (parser.feed(*p, frame)) {
            last = frame;
            finished = true;
        }
    }
    EXPECT_TRUE(finished);
    return last;
}

}  // namespace

TEST(Protocol, ChecksumAndFrameMatchKnownVectors) {
    EXPECT_EQ(motor::checksum("PING", 4), 0x10);
    EXPECT_EQ(motor::checksum("SET_RPM 1200", 12), 0x71);
    EXPECT_EQ(motor::checksum("GET_STATUS", 10), 0x1D);

    char frame[32];
    ASSERT_EQ(motor::format_frame(frame, static_cast<int>(sizeof(frame)), "PING"), 8);
    EXPECT_STREQ(frame, "PING*10\n");
    ASSERT_EQ(motor::format_frame(frame, static_cast<int>(sizeof(frame)), "SET_RPM 1200"), 16);
    EXPECT_STREQ(frame, "SET_RPM 1200*71\n");
    EXPECT_EQ(motor::format_frame(frame, 8, "PING"), -1);
}

TEST(Protocol, ParsesSetRpmAndStatusCommands) {
    motor::Parser parser;
    const motor::Frame set = feed_line(parser, "SET_RPM 1200*71\n");
    ASSERT_TRUE(set.ok);
    EXPECT_STREQ(set.payload, "SET_RPM 1200");

    motor::Command command;
    const char* error = "unset";
    ASSERT_TRUE(motor::decode_command(set.payload, set.length, command, error));
    EXPECT_EQ(error, nullptr);
    EXPECT_EQ(command.id, motor::CommandId::SetRpm);
    EXPECT_EQ(command.rpm, 1200);

    const motor::Frame status = feed_line(parser, "GET_STATUS*1d\n");
    ASSERT_TRUE(status.ok);
    ASSERT_TRUE(motor::decode_command(status.payload, status.length, command, error));
    EXPECT_EQ(command.id, motor::CommandId::GetStatus);
}

TEST(Protocol, RejectsBadChecksumWithoutLosingTheNextFrame) {
    motor::Parser parser;
    motor::Frame frames[2];
    int count = 0;
    const char* bytes = "PING*11\nPING*10\n";
    for (const char* p = bytes; *p != '\0'; ++p) {
        motor::Frame frame;
        if (parser.feed(*p, frame)) {
            ASSERT_LT(count, 2);
            frames[count++] = frame;
        }
    }
    ASSERT_EQ(count, 2);
    EXPECT_FALSE(frames[0].ok);
    EXPECT_STREQ(frames[0].error, "bad_checksum");
    EXPECT_TRUE(frames[1].ok);
    EXPECT_STREQ(frames[1].payload, "PING");
}

TEST(Protocol, StreamsPartialBytesAndCarriageReturn) {
    motor::Parser parser;
    const char* bytes = "STA";
    for (const char* p = bytes; *p != '\0'; ++p) {
        motor::Frame frame;
        EXPECT_FALSE(parser.feed(*p, frame));
    }
    const motor::Frame frame = feed_line(parser, "RT*40\r\n");
    ASSERT_TRUE(frame.ok);
    motor::Command command;
    const char* error = nullptr;
    ASSERT_TRUE(motor::decode_command(frame.payload, frame.length, command, error));
    EXPECT_EQ(command.id, motor::CommandId::Start);
}

TEST(Protocol, DecodeRejectsUnknownAndBadArguments) {
    motor::Command command;
    const char* error = nullptr;

    EXPECT_TRUE(motor::decode_command("STOP", 4, command, error));
    EXPECT_EQ(command.id, motor::CommandId::Stop);
    EXPECT_TRUE(motor::decode_command("CLEAR_FAULT", 11, command, error));
    EXPECT_EQ(command.id, motor::CommandId::ClearFault);
    EXPECT_TRUE(motor::decode_command("PING", 4, command, error));
    EXPECT_EQ(command.id, motor::CommandId::Ping);

    EXPECT_TRUE(motor::decode_command("SET_RPM -250", 12, command, error));
    EXPECT_EQ(command.rpm, -250);
    EXPECT_TRUE(motor::decode_command("SET_RPM +15", 11, command, error));
    EXPECT_EQ(command.rpm, 15);

    EXPECT_FALSE(motor::decode_command("NOPE", 4, command, error));
    EXPECT_STREQ(error, "unknown_command");
    EXPECT_FALSE(motor::decode_command("SET_RPM", 7, command, error));
    EXPECT_STREQ(error, "unknown_command");
    EXPECT_FALSE(motor::decode_command("SET_RPM 12x", 11, command, error));
    EXPECT_STREQ(error, "bad_arg");
    EXPECT_FALSE(motor::decode_command("SET_RPM 2147483648", 18, command, error));
    EXPECT_STREQ(error, "bad_arg");
    EXPECT_TRUE(motor::decode_command("SET_RPM -2147483648", 19, command, error));
    EXPECT_EQ(command.rpm, static_cast<std::int32_t>(-2147483648LL));
}

TEST(Protocol, MalformedLineResyncs) {
    motor::Parser parser;
    const motor::Frame bad = feed_line(parser, "NOT_A_FRAME\n");
    EXPECT_FALSE(bad.ok);
    EXPECT_STREQ(bad.error, "bad_frame");

    const motor::Frame good = feed_line(parser, "PING*10\n");
    EXPECT_TRUE(good.ok);
    EXPECT_STREQ(good.payload, "PING");
}

TEST(Protocol, StatusLineFormatsFixedFields) {
    char line[80];
    const int n = motor::format_status(line, static_cast<int>(sizeof(line)), motor::Mode::Run,
                                       motor::FaultCode::None, 1200.f, -10.f, 0.5f);
    ASSERT_GT(n, 0);
    EXPECT_STREQ(line, "STATUS RUN NONE 1200.0 -10.0 0.500");
}
