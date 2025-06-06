#pragma once

#include <ouster/types.h>
#include <ouster/client.h>
#include <fstream>
#include <nlohmann/json.hpp>
#include <vtk-9.1/vtkdiy2/include/vtkdiy2/fmt/format.h>

using namespace ouster;
using namespace ouster::sensor;


// Manual Mapping of UDPProfileLidar and UDPProfileIMU
inline UDPProfileLidar parse_udp_profile_lidar(const std::string& profile) {
    if (profile == "LEGACY") return PROFILE_LIDAR_LEGACY;
    if (profile == "RNG19_RFL8_SIG16_NIR16") return PROFILE_RNG19_RFL8_SIG16_NIR16;
    if (profile == "RNG19_RFL8_SIG16_NIR16_DUAL") return PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL;
    if (profile == "RNG15_RFL8_NIR8") return PROFILE_RNG15_RFL8_NIR8;
    if (profile == "FIVE_WORD_PIXEL") return PROFILE_FIVE_WORD_PIXEL;
    if (profile == "FUSA_RNG15_RFL8_NIR8_DUAL") return PROFILE_FUSA_RNG15_RFL8_NIR8_DUAL;
    throw std::runtime_error("Unknown udp_profile_lidar: " + profile);
}

inline UDPProfileIMU parse_udp_profile_imu(const std::string& profile) {
    if (profile == "LEGACY") return PROFILE_IMU_LEGACY;
    throw std::runtime_error("Unknown udp_profile_imu: " + profile);
}

inline sensor_info load_sensor_info_from_json(const std::string& json_path) {
    std::ifstream ifs(json_path);
    if (!ifs.is_open()) throw std::runtime_error("Failed to open JSON file");

    nlohmann::json j;
    ifs >> j;

    sensor_info info{};

    // 공식 ouster-ros/SDK JSON 구조 파싱
    const auto& format = j.at("format");
    info.format.columns_per_frame = format.at("columns_per_frame").get<uint32_t>();
    info.format.pixels_per_column = format.at("pixels_per_column").get<uint32_t>();
    info.format.columns_per_packet = format.at("columns_per_packet").get<uint32_t>();
    info.format.udp_profile_lidar = parse_udp_profile_lidar(format.at("udp_profile_lidar").get<std::string>());
    info.format.udp_profile_imu = parse_udp_profile_imu(format.at("udp_profile_imu").get<std::string>());
    // column_window는 공식 JSON에 없을 수 있으므로 기본값 사용
    if (format.contains("column_window")) {
        info.format.column_window = {
            format.at("column_window")[0].get<uint32_t>(),
            format.at("column_window")[1].get<uint32_t>()
        };
    } else {
        info.format.column_window = {0, info.format.columns_per_frame - 1};
    }
    info.format.pixel_shift_by_row = format.at("pixel_shift_by_row").get<std::vector<int>>();

    info.prod_line = j.at("prod_line").get<std::string>();
    info.fw_rev = j.at("fw_rev").get<std::string>();
    if (j.contains("build_date")) info.build_date = j.at("build_date").get<std::string>();
    if (j.contains("image_rev")) info.image_rev = j.at("image_rev").get<std::string>();
    if (j.contains("status")) info.status = j.at("status").get<std::string>();
    if (j.contains("sn")) info.sn = std::stoull(j.at("sn").get<std::string>());
    info.init_id = j.value("initialization_id", 0);

    info.beam_azimuth_angles = j.at("beam_azimuth_angles").get<std::vector<double>>();
    info.beam_altitude_angles = j.at("beam_altitude_angles").get<std::vector<double>>();
    info.lidar_origin_to_beam_origin_mm = j.at("lidar_origin_to_beam_origin_mm").get<double>();

    auto parse_mat4d = [](const nlohmann::json& arr) -> mat4d {
        if (arr.size() != 16) throw std::runtime_error("Matrix must have 16 elements");
        mat4d m;
        for (int i = 0; i < 16; ++i)
            m(i / 4, i % 4) = arr[i];
        return m;
    };
    if (j.contains("beam_to_lidar_transform"))
        info.beam_to_lidar_transform = parse_mat4d(j.at("beam_to_lidar_transform"));
    if (j.contains("imu_to_sensor_transform"))
        info.imu_to_sensor_transform = parse_mat4d(j.at("imu_to_sensor_transform"));
    if (j.contains("lidar_to_sensor_transform"))
        info.lidar_to_sensor_transform = parse_mat4d(j.at("lidar_to_sensor_transform"));

    std::cout << "columns_per_frame: " << info.format.columns_per_frame << std::endl;
    std::cout << "pixels_per_column: " << info.format.pixels_per_column << std::endl;
    std::cout << "pixel_shift_by_row size: " << info.format.pixel_shift_by_row.size() << std::endl;
    std::cout << "beam_azimuth_angles size: " << info.beam_azimuth_angles.size() << std::endl;
    std::cout << "beam_altitude_angles size: " << info.beam_altitude_angles.size() << std::endl;

    return info;
}
