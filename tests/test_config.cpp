#include <doctest/doctest.h>

#include <nlohmann/json.hpp>

#include <fstream>
#include <string>

#include "core/config.h"

using namespace dev_board;
using nlohmann::json;

namespace {

constexpr const char* kConfigPath = "config/dev_board.json";

json valid() {
    std::ifstream file(kConfigPath);
    REQUIRE(file);
    return json::parse(file, nullptr, true, /*ignore_comments=*/true);
}

std::string error_of(const json& config) {
    try {
        parse_config(config.dump());
    } catch (const ConfigError& error) {
        return error.what();
    }
    return "";
}

bool mentions(const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
}

}  // namespace

TEST_CASE("shipped config matches spec section 11") {
    const Config c = load_config(kConfigPath);
    CHECK(c.grid.cell_size_m == 0.25);
    CHECK(c.zone.width_m == 1.0);
    CHECK(c.zone.ring_width_m == 0.5);
    CHECK(c.zone.width_cells == 4);
    CHECK(c.zone.ring_cells == 2);
    CHECK(c.zone.box_short_side_m == 0.4);
    CHECK(c.evaluate.hole_depth_m == 0.4);
    CHECK(c.evaluate.max_slope_deg == 10.0);
    CHECK(c.evaluate.max_roughness_m == 0.1);
    CHECK(c.evaluate.depth_noise_m == 0.0);
    CHECK(c.evaluate.max_zone_invalid_ratio == 0.1);
    CHECK(c.evaluate.max_cell_invalid_ratio == 0.5);
    CHECK(c.evaluate.min_points_per_cell == 3);
    CHECK(c.evaluate.low_percentile == 0.2);
    CHECK(c.evaluate.refit_rounds == 2);
    CHECK(c.limits.min_distance_m == 0.3);
    CHECK(c.limits.max_tilt_deg == 5.0);
    CHECK(c.suggest.neighbor_rings == 1);
    CHECK(c.suggest.max_distance_m == 8.0);
    CHECK(c.suggest.exclude_radius_m == 2.0);
    CHECK(c.suggest.exclude_expire_ms == 300000);
    CHECK(c.drop_check.drop_max_height_m == 5.0);
    CHECK(c.drop_check.max_speed_mps == 0.3);
    CHECK(c.drop_check.frames == 10);
    CHECK(c.drop_check.min_pass == 8);
    CHECK(c.drop_check.max_wait_ms == 1500);
    CHECK(c.timing.data_max_age_ms == 500);
    CHECK(c.timing.px4_buffer_ms == 1000);
    CHECK(c.timing.interpolation_max_ms == 100);
    CHECK(c.timing.tcp_recv_timeout_ms == 3000);
    CHECK(c.timing.tcp_connect_timeout_ms == 1000);
    CHECK(c.camera.near_m == 0.1);
    CHECK(c.camera.far_m == 20.0);
    CHECK(c.camera.t_cam == std::array<double, 3>{0, 0, 0});
    CHECK(c.network.mavlink_port == 14540);
}

TEST_CASE("depth intrinsics are the RGB intrinsics scaled by 1/4") {
    const Config c = load_config(kConfigPath);
    CHECK(c.camera.depth.width == 160);
    CHECK(c.camera.depth.height == 120);
    CHECK(c.camera.depth.fx == doctest::Approx(c.camera.rgb.fx / 4));
    CHECK(c.camera.depth.fy == doctest::Approx(c.camera.rgb.fy / 4));
    CHECK(c.camera.depth.cx == doctest::Approx(c.camera.rgb.cx / 4));
    CHECK(c.camera.depth.cy == doctest::Approx(c.camera.rgb.cy / 4));
}

TEST_CASE("r and Zw are rounded up to whole cells") {
    json j = valid();
    j["zone"]["ring_width_m"] = 0.35;
    j["zone"]["width_m"] = 1.2;
    const Config c = parse_config(j.dump());
    CHECK(c.zone.ring_width_m == 0.5);
    CHECK(c.zone.width_m == 1.5);
    CHECK(c.zone.ring_cells == 2);
    CHECK(c.zone.width_cells == 6);
}

TEST_CASE("rounding ignores floating point error but not real excess") {
    CHECK(ceil_cells(1.1, 0.1) == 11);
    CHECK(ceil_cells(0.3, 0.1) == 3);
    CHECK(ceil_cells(0.5, 0.25) == 2);
    CHECK(ceil_cells(0.501, 0.25) == 3);
    CHECK(ceil_cells(0.01, 0.25) == 1);
}

TEST_CASE("bad fields are rejected and named") {
    json j = valid();
    SUBCASE("missing field") {
        j["zone"].erase("width_m");
        CHECK(mentions(error_of(j), "zone.width_m"));
    }
    SUBCASE("missing section") {
        j.erase("zone");
        CHECK(mentions(error_of(j), "zone"));
    }
    SUBCASE("section is not an object") {
        j["zone"] = 3;
        CHECK(mentions(error_of(j), "zone"));
    }
    SUBCASE("misspelled field") {
        j["zone"]["widht_m"] = 1.0;
        CHECK(mentions(error_of(j), "zone.widht_m"));
    }
    SUBCASE("unknown section") {
        j["zoen"] = json::object();
        CHECK(mentions(error_of(j), "zoen"));
    }
    SUBCASE("unknown nested field") {
        j["camera"]["depth"]["hfov"] = 87;
        CHECK(mentions(error_of(j), "camera.depth.hfov"));
    }
    SUBCASE("string for a number") {
        j["zone"]["width_m"] = "1.0";
        CHECK(mentions(error_of(j), "zone.width_m"));
    }
    SUBCASE("bool for a number") {
        j["zone"]["width_m"] = true;
        CHECK(mentions(error_of(j), "zone.width_m"));
    }
    SUBCASE("fraction for an integer") {
        j["drop_check"]["frames"] = 3.7;
        CHECK(mentions(error_of(j), "drop_check.frames"));
    }
    SUBCASE("integer too large for int") {
        j["drop_check"]["frames"] = 18446744073709551615u;
        CHECK(mentions(error_of(j), "drop_check.frames"));
    }
    SUBCASE("zero cell size") {
        j["grid"]["cell_size_m"] = 0;
        CHECK(mentions(error_of(j), "grid.cell_size_m"));
    }
    SUBCASE("negative ring width") {
        j["zone"]["ring_width_m"] = -0.5;
        CHECK(mentions(error_of(j), "zone.ring_width_m"));
    }
    SUBCASE("ratio above one") {
        j["evaluate"]["max_zone_invalid_ratio"] = 10;
        CHECK(mentions(error_of(j), "evaluate.max_zone_invalid_ratio"));
    }
    SUBCASE("port zero") {
        j["network"]["depth_port"] = 0;
        CHECK(mentions(error_of(j), "network.depth_port"));
    }
    SUBCASE("port too large") {
        j["network"]["depth_port"] = 65536;
        CHECK(mentions(error_of(j), "network.depth_port"));
    }
    SUBCASE("empty host") {
        j["network"]["server_host"] = "";
        CHECK(mentions(error_of(j), "network.server_host"));
    }
    SUBCASE("t_cam with two values") {
        j["camera"]["t_cam"] = {0, 0};
        CHECK(mentions(error_of(j), "camera.t_cam"));
    }
}

TEST_CASE("contradicting fields are rejected") {
    json j = valid();
    SUBCASE("near not below far") {
        j["camera"]["near_m"] = 20;
        CHECK(mentions(error_of(j), "camera.near_m"));
    }
    SUBCASE("more passes than frames") {
        j["drop_check"]["min_pass"] = 11;
        CHECK(mentions(error_of(j), "drop_check.min_pass"));
    }
    SUBCASE("both UDP listeners on one port") {
        j["network"]["command_port"] = 14540;
        CHECK(mentions(error_of(j), "network.command_port"));
    }
}

TEST_CASE("text that is not a config object is rejected") {
    CHECK_THROWS_AS(parse_config(""), ConfigError);
    CHECK_THROWS_AS(parse_config("{"), ConfigError);
    CHECK_THROWS_AS(parse_config("[]"), ConfigError);
    CHECK_THROWS_AS(parse_config(R"({"grid": {"cell_size_m": 1e999}})"), ConfigError);
}

TEST_CASE("missing file is reported by name") {
    try {
        load_config("no/such/file.json");
        FAIL("expected ConfigError");
    } catch (const ConfigError& error) {
        CHECK(mentions(error.what(), "no/such/file.json"));
    }
}
