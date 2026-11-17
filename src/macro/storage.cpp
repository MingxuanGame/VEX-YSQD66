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
    "VEX_MACRO,3",
    "period_ms,20",
    "record_ms,14500",
    "reset_ms,500",
    "mapping,chassis_arcade_v1",
    "t_ms,left_x,left_y,right_x,right_y,buttons,chassis_left_power,chassis_right_power,checkpoint"};

bool save_failure(SaveError *error, const char *stage, int error_number) {
    if (error) {
        *error = {stage, error_number};
    }
    return false;
}

bool filename_equal(const char *name, std::size_t length, const char *expected) {
    if (length != std::strlen(expected)) {
        return false;
    }
    for (std::size_t i = 0; i < length; ++i) {
        const auto lower = [](char value) {
            return value >= 'A' && value <= 'Z' ? value + ('a' - 'A') : value;
        };
        if (lower(name[i]) != lower(expected[i])) {
            return false;
        }
    }
    return true;
}

bool listing_contains(const char *listing, const char *name) {
    for (const char *cursor = listing; *cursor;) {
        const char *end = std::strchr(cursor, '\n');
        if (!end) {
            end = cursor + std::strlen(cursor);
        }
        const auto length = end - cursor;
        if (filename_equal(cursor, length > 0 && end[-1] == '\r' ? length - 1 : length, name)) {
            return true;
        }
        cursor = *end ? end + 1 : end;
    }
    return false;
}

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

bool parse_row(char *line, std::size_t index, Frame &frame) {
    std::array<int, 9> values{};
    constexpr std::size_t count = 9;
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

bool read_header(std::FILE *file) {
    std::array<char, 192> line{};
    for (const char *header : kHeader) {
        if (!read_line(file, line.data(), line.size()) || std::strcmp(line.data(), header) != 0) {
            return false;
        }
    }
    return true;
}

// expected 非空时只使用一帧栈空间回读验证，避免在 PROS 回调栈上构造整个 Clip。
bool read_file(std::FILE *file, Clip *output, const Clip *expected) {
    if (!read_header(file)) {
        return false;
    }
    std::array<char, 192> line{};
    for (std::size_t i = 0; i < kFrameCount; ++i) {
        Frame frame{};
        if (!read_line(file, line.data(), line.size()) || !parse_row(line.data(), i, frame)) {
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

bool compatible_clip(const char *path) {
    auto *file = std::fopen(path, "rb");
    if (!file) {
        return false;
    }
    const bool valid = read_header(file);
    const bool closed = std::fclose(file) == 0;
    return valid && closed;
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

bool save_new_clip(const char *directory, int id, const Clip &clip, SaveError *error,
                   const char *directory_listing) {
    if (error) {
        *error = {};
    }
    for (const auto &frame : clip) {
        if (!valid_frame(frame)) {
            return save_failure(error, "VALIDATE", EINVAL);
        }
    }
    std::array<char, 192> path{};
    if (!macro_path(directory, id, path.data(), path.size())) {
        return save_failure(error, "PATH", EINVAL);
    }
    if (directory_listing) {
        const char *name = std::strrchr(path.data(), '/');
        if (listing_contains(directory_listing, name ? name + 1 : path.data())) {
            return save_failure(error, "EXISTS", EEXIST);
        }
    } else {
        errno = 0;
        auto *existing = std::fopen(path.data(), "rb");
        if (existing) {
            std::fclose(existing);
            return save_failure(error, "EXISTS", EEXIST);
        }
        if (errno != ENOENT) {
            return save_failure(error, "CHECK", errno);
        }
    }
    // 单任务为 SD 的唯一写入者；不使用 PROS 未承诺支持的 rename/O_EXCL。
    errno = 0;
    auto *file = std::fopen(path.data(), "wb");
    if (!file) {
        return save_failure(error, "CREATE", errno);
    }
    const auto write_failure = [&](const char *stage) {
        // fclose 可能另报错误，先保留使本次保存失败的原始 errno。
        const int write_error = errno;
        std::fclose(file);
        return save_failure(error, stage, write_error);
    };
    for (const char *header : kHeader) {
        errno = 0;
        if (std::fprintf(file, "%s\n", header) <= 0) {
            return write_failure("HEADER");
        }
    }
    for (std::size_t i = 0; i < clip.size(); ++i) {
        const auto &frame = clip[i];
        errno = 0;
        if (std::fprintf(
                file, "%lu,%d,%d,%d,%d,%u,%d,%d,%d\n", static_cast<unsigned long>(i * kPeriodMs),
                frame.input.left_x, frame.input.left_y, frame.input.right_x, frame.input.right_y,
                static_cast<unsigned>(frame.input.buttons), frame.command.chassis_left_power,
                frame.command.chassis_right_power, frame.checkpoint ? 1 : 0) <= 0) {
            return write_failure("FRAMES");
        }
    }
    errno = 0;
    if (std::fprintf(file, "END,725\n") <= 0) {
        return write_failure("TRAILER");
    }
    errno = 0;
    if (std::fflush(file) != 0) {
        return write_failure("FLUSH");
    }
    errno = 0;
    if (std::fclose(file) != 0) {
        return save_failure(error, "CLOSE", errno);
    }
    errno = 0;
    file = std::fopen(path.data(), "rb");
    if (!file) {
        return save_failure(error, "REOPEN", errno);
    }
    errno = 0;
    const bool verified = read_file(file, nullptr, &clip);
    const int verify_error = std::ferror(file) ? errno : 0;
    errno = 0;
    const bool closed = std::fclose(file) == 0;
    if (!verified) {
        return save_failure(error, "VERIFY", verify_error);
    }
    if (!closed) {
        return save_failure(error, "VERIFY_CLOSE", errno);
    }
    return true;
}

} // namespace macrorec
