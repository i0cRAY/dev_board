// 確認第三方函式庫能編譯、能用（交叉編譯時也會編到）。
#include <doctest/doctest.h>
#include <mavlink/common/mavlink.h>
#include <nlohmann/json.hpp>

#include <cstdint>
#include <vector>

namespace {

// 跟真的接收一樣一個 byte 一個 byte 解析。
bool parse(const mavlink_message_t& sent, mavlink_message_t& received) {
    std::vector<std::uint8_t> buffer(MAVLINK_MAX_PACKET_LEN);
    const auto size = mavlink_msg_to_send_buffer(buffer.data(), &sent);
    mavlink_status_t status{};
    for (std::size_t i = 0; i < size; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &received, &status)) {
            return true;
        }
    }
    return false;
}

}  // namespace

TEST_CASE("MAVLink ATTITUDE round trip") {
    mavlink_message_t sent{};
    mavlink_msg_attitude_pack(1, 1, &sent, 123456, 0.1f, -0.2f, 3.0f, 0, 0, 0);
    mavlink_message_t received{};
    REQUIRE(parse(sent, received));
    REQUIRE(received.msgid == MAVLINK_MSG_ID_ATTITUDE);
    mavlink_attitude_t attitude{};
    mavlink_msg_attitude_decode(&received, &attitude);
    CHECK(attitude.time_boot_ms == 123456);
    CHECK(attitude.roll == 0.1f);
    CHECK(attitude.pitch == -0.2f);
    CHECK(attitude.yaw == 3.0f);
}

TEST_CASE("MAVLink LOCAL_POSITION_NED round trip") {
    mavlink_message_t sent{};
    mavlink_msg_local_position_ned_pack(1, 1, &sent, 123456, 31.8f, -12.4f, -7.5f, 0.1f, 0.2f, 0);
    mavlink_message_t received{};
    REQUIRE(parse(sent, received));
    REQUIRE(received.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED);
    mavlink_local_position_ned_t position{};
    mavlink_msg_local_position_ned_decode(&received, &position);
    CHECK(position.x == 31.8f);
    CHECK(position.y == -12.4f);
    CHECK(position.z == -7.5f);
    CHECK(position.vx == 0.1f);
    CHECK(position.vy == 0.2f);
}

TEST_CASE("JSON null survives a round trip") {
    const auto message = nlohmann::json::parse(R"({"type": "height", "payload": {"agl": null}})");
    CHECK(message.at("payload").at("agl").is_null());
    CHECK(nlohmann::json::parse(message.dump()) == message);
}
