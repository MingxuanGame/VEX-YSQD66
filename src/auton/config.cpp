#include "auton/kit.hpp"

#include <cstdio>
#include <cstring>

namespace auton {
namespace {
constexpr std::uint32_t kMagic = 0x41555856;
constexpr std::uint32_t kVersion = 1;
constexpr int kWords = 5 + kRouteCount * kStepCount * kParamCount;
using Record = std::array<unsigned char, kWords * 4>;

void put(Record &record, int word, std::uint32_t value) {
    for (int i = 0; i < 4; ++i) {
        record[word * 4 + i] = static_cast<unsigned char>(value >> (i * 8));
    }
}

std::uint32_t get(const Record &record, int word) {
    std::uint32_t value = 0;
    for (int i = 0; i < 4; ++i) {
        value |= static_cast<std::uint32_t>(record[word * 4 + i]) << (i * 8);
    }
    return value;
}

std::uint32_t checksum(const Record &record) {
    std::uint32_t hash = 2166136261U;
    for (int i = 0; i < (kWords - 1) * 4; ++i) {
        hash = (hash ^ record[i]) * 16777619U;
    }
    return hash;
}

Record encode(const Config &config, std::uint32_t generation) {
    Record record{};
    put(record, 0, kMagic);
    put(record, 1, kVersion);
    put(record, 2, generation);
    put(record, 3, config.selected_route);
    int word = 4;
    for (const auto &route : config.routes) {
        for (const auto &step : route.steps) {
            for (int value : step.values) {
                put(record, word++, value);
            }
        }
    }
    put(record, kWords - 1, checksum(record));
    return record;
}

bool decode(const Record &record, Config &config) {
    if (get(record, 0) != kMagic || get(record, 1) != kVersion ||
        get(record, kWords - 1) != checksum(record)) {
        return false;
    }
    Config candidate{};
    if (get(record, 3) >= kRouteCount) {
        return false;
    }
    candidate.selected_route = static_cast<int>(get(record, 3));
    int word = 4;
    for (auto &route : candidate.routes) {
        for (auto &step : route.steps) {
            for (int p = 0; p < kParamCount; ++p) {
                const auto value = get(record, word++);
                if (value > static_cast<unsigned>(param_max(static_cast<Param>(p)))) {
                    return false;
                }
                step.values[p] = static_cast<int>(value);
            }
        }
    }
    config = candidate;
    return true;
}

bool read_slot(const char *base, int slot, Record &record, Config &config) {
    char path[180];
    std::snprintf(path, sizeof(path), "%s.%d", base, slot);
    auto *file = std::fopen(path, "rb");
    if (!file) {
        return false;
    }
    const bool complete = std::fread(record.data(), 1, record.size(), file) == record.size();
    const bool end = std::fgetc(file) == EOF && !std::ferror(file);
    const bool closed = std::fclose(file) == 0;
    return complete && end && closed && decode(record, config);
}

int newest(const char *base, Record &record, Config &config) {
    Record other{};
    Config other_config{};
    const bool first = read_slot(base, 0, record, config);
    const bool second = read_slot(base, 1, other, other_config);
    // 无符号差处理版本号回绕；两个有效槽正常只相差一次保存。
    const auto difference = get(other, 2) - get(record, 2);
    if (second && (!first || (difference != 0 && difference < 0x80000000U))) {
        record = other;
        config = other_config;
        return 1;
    }
    return first ? 0 : -1;
}
} // namespace

Config defaults() {
    Config config{};
    for (auto &route : config.routes) {
        for (auto &step : route.steps) {
            // TODO：所有角度、动作时长均需实机标定；这里仅提供超时上限初值。
            step.values[static_cast<int>(Param::ContactTimeout)] = 2000;
            step.values[static_cast<int>(Param::TurnTimeout)] = 2000;
        }
    }
    // 路线二独立配置；在此修改其默认参数，改动作顺序则修改 Runner 的步序分支。
    return config;
}

const char *param_name(Param param) {
    constexpr std::array<const char *, kParamCount> kNames{
        "D_deg",   "back_ms",   "forward_ms",       "lift_ms",
        "down_ms", "action_ms", "contact_limit_ms", "turn_limit_ms"};
    const int index = static_cast<int>(param);
    return index >= 0 && index < kParamCount ? kNames[index] : "INVALID";
}

int param_max(Param param) {
    return param == Param::Angle ? 360 : 10000;
}

bool uses_param(int step, Param param) {
    if (step < 0 || step >= kStepCount) {
        return false;
    }
    switch (param) {
    case Param::Angle:
    case Param::TurnTimeout:
        return true;
    case Param::Back:
        return step == 1 || step == 3;
    case Param::Forward:
        return step == 1 || step == 3 || step == 4 || step == 5;
    case Param::Lift:
        return step == 2 || step == 4;
    case Param::Lower:
        return step == 0 || step == 2 || step == 4 || step == 5;
    case Param::Action:
        return step != 5;
    case Param::ContactTimeout:
        return step == 0 || step == 2 || step == 4;
    default:
        return false;
    }
}

bool valid_config(const Config &config) {
    if (config.selected_route < 0 || config.selected_route >= kRouteCount) {
        return false;
    }
    for (const auto &route : config.routes) {
        for (const auto &step : route.steps) {
            for (int p = 0; p < kParamCount; ++p) {
                if (step.values[p] < 0 || step.values[p] > param_max(static_cast<Param>(p))) {
                    return false;
                }
            }
        }
    }
    return true;
}

Store::Store(const char *base_path) {
    if (base_path && std::strlen(base_path) < m_base.size()) {
        std::snprintf(m_base.data(), m_base.size(), "%s", base_path);
    }
}

bool Store::load(Config &config) {
    Record record{};
    Config candidate{};
    if (m_base[0] == '\0' || newest(m_base.data(), record, candidate) < 0) {
        return false;
    }
    config = candidate;
    return true;
}

bool Store::save(const Config &config) {
    if (m_base[0] == '\0' || !valid_config(config)) {
        return false;
    }
    Record previous{};
    Config old{};
    const int active = newest(m_base.data(), previous, old);
    const int slot = active == 0 ? 1 : 0;
    const Record record = encode(config, active < 0 ? 1 : get(previous, 2) + 1);
    char path[180];
    std::snprintf(path, sizeof(path), "%s.%d", m_base.data(), slot);
    auto *file = std::fopen(path, "wb");
    if (!file) {
        return false;
    }
    bool ok = std::fwrite(record.data(), 1, record.size(), file) == record.size();
    if (std::fflush(file) != 0) {
        ok = false;
    }
    if (std::fclose(file) != 0) {
        ok = false;
    }
    Record verified{};
    return ok && read_slot(m_base.data(), slot, verified, old) && record == verified;
}
} // namespace auton
