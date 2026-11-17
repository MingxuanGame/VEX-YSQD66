#include "macro/storage.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace macrorec {
namespace {
constexpr std::array<const char *, 6> kHeader{
    "VEX_MACRO,2",
    "period_ms,20",
    "record_ms,14500",
    "reset_ms,500",
    "mapping,chassis_arcade_v1",
    "t_ms,left_x,left_y,right_x,right_y,buttons,chassis_left_power,chassis_right_power,checkpoint"};
constexpr const char *kLegacyColumns =
    "t_ms,left_x,left_y,right_x,right_y,buttons,chassis_left_power,chassis_right_power";

bool read_line(std::FILE *file, char *line, std::size_t capacity) {
    if (!std::fgets(line, static_cast<int>(capacity), file)) {
        return false;
    }
    const auto length = std::strlen(line);
    if (length == 0 || line[length - 1] != '\n') {
        return false;
    }
    line[length - 1] = '\0';
    if (length > 1 && line[length - 2] == '\r') {
        line[length - 2] = '\0';
    }
    return true;
}

bool parse_row(char *line, std::size_t index, Frame &frame, bool legacy) {
    std::array<int, 9> values{};
    const std::size_t count = legacy ? 8 : 9;
    char *cursor = line;
    for (std::size_t i = 0; i < count; ++i) {
        if (!((*cursor >= '0' && *cursor <= '9') || *cursor == '-')) {
            return false;
        }
        char *end = nullptr;
        errno = 0;
        const long value = std::strtol(cursor, &end, 10);
        if (errno != 0 || end == cursor || value < INT_MIN || value > INT_MAX ||
            *end != (i + 1 == count ? '\0' : ',')) {
            return false;
        }
        values[i] = static_cast<int>(value);
        cursor = end + (i + 1 == count ? 0 : 1);
    }
    if (values[0] != static_cast<int>(index * kPeriodMs) || values[5] < 0 ||
        values[5] > kButtonMask || (values[8] != 0 && values[8] != 1)) {
        return false;
    }
    frame = {
        {values[1], values[2], values[3], values[4], static_cast<std::uint16_t>(values[5]), true},
        {values[6], values[7]},
        values[8] == 1};
    return valid_frame(frame);
}

// expected 非空时只使用一帧栈空间回读验证，避免在 PROS 回调栈上构造整个 Clip。
bool read_file(std::FILE *file, Clip *output, const Clip *expected) {
    std::array<char, 192> line{};
    if (!read_line(file, line.data(), line.size())) {
        return false;
    }
    const bool legacy = std::strcmp(line.data(), "VEX_MACRO,1") == 0;
    if (!legacy && std::strcmp(line.data(), kHeader[0]) != 0) {
        return false;
    }
    for (std::size_t i = 1; i < kHeader.size(); ++i) {
        const char *header = legacy && i == kHeader.size() - 1 ? kLegacyColumns : kHeader[i];
        if (!read_line(file, line.data(), line.size()) || std::strcmp(line.data(), header) != 0) {
            return false;
        }
    }
    for (std::size_t i = 0; i < kFrameCount; ++i) {
        Frame frame{};
        if (!read_line(file, line.data(), line.size()) ||
            !parse_row(line.data(), i, frame, legacy)) {
            return false;
        }
        if (output) {
            (*output)[i] = frame;
        }
        if (expected) {
            const auto &original = (*expected)[i];
            if (frame.input.left_x != original.input.left_x ||
                frame.input.left_y != original.input.left_y ||
                frame.input.right_x != original.input.right_x ||
                frame.input.right_y != original.input.right_y ||
                frame.input.buttons != original.input.buttons ||
                frame.command.chassis_left_power != original.command.chassis_left_power ||
                frame.command.chassis_right_power != original.command.chassis_right_power ||
                frame.checkpoint != original.checkpoint) {
                return false;
            }
        }
    }
    return read_line(file, line.data(), line.size()) && std::strcmp(line.data(), "END,725") == 0 &&
           std::fgetc(file) == EOF && !std::ferror(file);
}
} // namespace

std::vector<int> catalog(const char *listing) {
    std::vector<int> ids;
    if (!listing) {
        return ids;
    }
    const char *cursor = listing;
    while (*cursor) {
        const char *end = std::strchr(cursor, '\n');
        if (!end) {
            end = cursor + std::strlen(cursor);
        }
        auto length = end - cursor;
        if (length > 0 && cursor[length - 1] == '\r') {
            --length;
        }
        if (length == 16 && std::strncmp(cursor, "macro_", 6) == 0 &&
            std::strncmp(cursor + 12, ".csv", 4) == 0) {
            int id = 0;
            bool digits = true;
            for (int i = 6; i < 12; ++i) {
                digits = digits && cursor[i] >= '0' && cursor[i] <= '9';
                id = id * 10 + (digits ? cursor[i] - '0' : 0);
            }
            if (digits && id > 0) {
                ids.push_back(id);
            }
        }
        cursor = *end ? end + 1 : end;
    }
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    return ids;
}

bool macro_path(const char *directory, int id, char *path, std::size_t capacity) {
    if (!directory || !path || capacity == 0 || id < 1 || id > kMaxMacroId) {
        return false;
    }
    const int length = std::snprintf(path, capacity, "%s/macro_%06d.csv", directory, id);
    return length > 0 && static_cast<std::size_t>(length) < capacity;
}

bool load_clip(const char *path, Clip &clip) {
    auto *file = std::fopen(path, "rb");
    if (!file) {
        return false;
    }
    const bool valid = read_file(file, &clip, nullptr);
    const bool closed = std::fclose(file) == 0;
    return valid && closed;
}

bool save_new_clip(const char *directory, int id, const Clip &clip) {
    for (const auto &frame : clip) {
        if (!valid_frame(frame)) {
            return false;
        }
    }
    std::array<char, 192> path{};
    if (!macro_path(directory, id, path.data(), path.size())) {
        return false;
    }
    errno = 0;
    auto *existing = std::fopen(path.data(), "rb");
    if (existing) {
        std::fclose(existing);
        return false;
    }
    if (errno != ENOENT) {
        return false;
    }
    // 单任务为 SD 的唯一写入者；不使用 PROS 未承诺支持的 rename/O_EXCL。
    auto *file = std::fopen(path.data(), "wb");
    if (!file) {
        return false;
    }
    bool written = true;
    for (const char *header : kHeader) {
        written = (std::fprintf(file, "%s\n", header) > 0) && written;
    }
    for (std::size_t i = 0; i < clip.size() && written; ++i) {
        const auto &frame = clip[i];
        written = std::fprintf(file, "%lu,%d,%d,%d,%d,%u,%d,%d,%d\n",
                               static_cast<unsigned long>(i * kPeriodMs), frame.input.left_x,
                               frame.input.left_y, frame.input.right_x, frame.input.right_y,
                               static_cast<unsigned>(frame.input.buttons),
                               frame.command.chassis_left_power, frame.command.chassis_right_power,
                               frame.checkpoint ? 1 : 0) > 0;
    }
    written = (std::fprintf(file, "END,725\n") > 0) && written;
    written = (std::fflush(file) == 0) && written;
    written = (std::fclose(file) == 0) && written;
    if (!written) {
        return false;
    }
    file = std::fopen(path.data(), "rb");
    if (!file) {
        return false;
    }
    const bool verified = read_file(file, nullptr, &clip);
    const bool closed = std::fclose(file) == 0;
    return verified && closed;
}

} // namespace macrorec
