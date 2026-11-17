#include "macro/storage.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdio>
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
        const auto lower = [](char c) { return (c >= 'A' && c <= 'Z') ? c + ('a' - 'A') : c; };
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
        const auto length = static_cast<std::size_t>(end - cursor);
        const auto trimmed = (length > 0 && end[-1] == '\r') ? length - 1 : length;
        if (filename_equal(cursor, trimmed, name)) {
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
    auto length = std::strlen(line);
    if (length == 0 || line[length - 1] != '\n') {
        return false;
    }
    line[--length] = '\0';
    if (length > 0 && line[length - 1] == '\r') {
        line[--length] = '\0';
    }
    return true;
}

bool parse_row(const char *line, std::size_t index, Frame &frame) {
    int t_ms = 0;
    int lx = 0;
    int ly = 0;
    int rx = 0;
    int ry = 0;
    int buttons = 0;
    int left_power = 0;
    int right_power = 0;
    int checkpoint = 0;
    int consumed = 0;
    if (std::sscanf(line, "%d,%d,%d,%d,%d,%d,%d,%d,%d%n", &t_ms, &lx, &ly, &rx, &ry, &buttons,
                    &left_power, &right_power, &checkpoint, &consumed) != 9 ||
        line[consumed] != '\0') {
        return false;
    }
    if (t_ms != static_cast<int>(index * kPeriodMs) || buttons < 0 || buttons > kButtonMask ||
        (checkpoint != 0 && checkpoint != 1)) {
        return false;
    }
    frame = {{lx, ly, rx, ry, static_cast<std::uint16_t>(buttons), true},
             {left_power, right_power},
             checkpoint == 1};
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

// expected 非空时逐帧比对，避免在回调栈上分配第二个 Clip。
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
            const auto &orig = (*expected)[i];
            if (frame.input.left_x != orig.input.left_x ||
                frame.input.left_y != orig.input.left_y ||
                frame.input.right_x != orig.input.right_x ||
                frame.input.right_y != orig.input.right_y ||
                frame.input.buttons != orig.input.buttons ||
                frame.command.chassis_left_power != orig.command.chassis_left_power ||
                frame.command.chassis_right_power != orig.command.chassis_right_power ||
                frame.checkpoint != orig.checkpoint) {
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
    for (const char *cursor = listing; *cursor;) {
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
    return std::fclose(file) == 0 && valid;
}

bool load_clip(const char *path, Clip &clip) {
    auto *file = std::fopen(path, "rb");
    if (!file) {
        return false;
    }
    const bool valid = read_file(file, &clip, nullptr);
    return std::fclose(file) == 0 && valid;
}

bool save_new_clip(const char *directory, int id, const Clip &clip, SaveError *error,
                   const char *directory_listing) {
    if (error) {
        *error = {};
    }
    std::array<char, 192> path{};
    if (!macro_path(directory, id, path.data(), path.size())) {
        return save_failure(error, "PATH", EINVAL);
    }
    // PROS 4.2.3 下 fopen 不存在文件也会返回 ENFILE(23)，优先使用目录列表判重。
    if (directory_listing) {
        const char *name = std::strrchr(path.data(), '/');
        if (listing_contains(directory_listing, name ? name + 1 : path.data())) {
            return save_failure(error, "EXISTS", EEXIST);
        }
    } else if (auto *existing = std::fopen(path.data(), "rb")) {
        std::fclose(existing);
        return save_failure(error, "EXISTS", EEXIST);
    }

    errno = 0;
    auto *file = std::fopen(path.data(), "wb");
    if (!file) {
        return save_failure(error, "OPEN", errno);
    }
    for (const char *header : kHeader) {
        std::fprintf(file, "%s\n", header);
    }
    for (std::size_t i = 0; i < clip.size(); ++i) {
        const auto &frame = clip[i];
        std::fprintf(file, "%lu,%d,%d,%d,%d,%u,%d,%d,%d\n",
                     static_cast<unsigned long>(i * kPeriodMs), frame.input.left_x,
                     frame.input.left_y, frame.input.right_x, frame.input.right_y,
                     static_cast<unsigned>(frame.input.buttons), frame.command.chassis_left_power,
                     frame.command.chassis_right_power, frame.checkpoint ? 1 : 0);
    }
    std::fprintf(file, "END,725\n");
    const bool write_ok = !std::ferror(file) && std::fflush(file) == 0;
    const int write_err = errno;
    if (std::fclose(file) != 0 || !write_ok) {
        return save_failure(error, "WRITE", write_err != 0 ? write_err : errno);
    }

    errno = 0;
    file = std::fopen(path.data(), "rb");
    if (!file) {
        return save_failure(error, "VERIFY", errno);
    }
    const bool verified = read_file(file, nullptr, &clip);
    const int verify_err = std::ferror(file) ? errno : 0;
    if (std::fclose(file) != 0 || !verified) {
        return save_failure(error, "VERIFY", verify_err != 0 ? verify_err : errno);
    }
    return true;
}

} // namespace macrorec
