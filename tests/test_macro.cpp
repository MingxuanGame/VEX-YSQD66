#include "macro/engine.hpp"
#include "macro/storage.hpp"
#include "test_common.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>

namespace {
class MacroHardware final : public macrorec::Hardware {
public:
    bool apply(const macrorec::Command &command) override;
    void stop() override;
    macrorec::Command output{};
    int writes = 0;
    bool healthy = true;
};

bool MacroHardware::apply(const macrorec::Command &command) {
    output = command;
    ++writes;
    return healthy;
}

void MacroHardware::stop() {
    output = {};
}

macrorec::Input input_for(std::size_t index) {
    return {static_cast<int>(index % 255) - 127,      static_cast<int>(index % 151) - 75,
            static_cast<int>(index % 201) - 100,      127,
            static_cast<std::uint16_t>(index % 4096), true};
}

void test_mapping() {
    macrorec::Input input{100, -127, 0, -127, 0x0fff, true};
    auto command = macrorec::map_input(input);
    CHECK(command.chassis_left_power == 127 && command.chassis_right_power == 127);
    input.right_x = -127;
    command = macrorec::map_input(input);
    CHECK(command.chassis_left_power == 127 && command.chassis_right_power == 0);
    input.left_y = 0;
    command = macrorec::map_input(input);
    CHECK(command.chassis_left_power == 127 && command.chassis_right_power == -127);
    input.connected = false;
    command = macrorec::map_input(input);
    CHECK(command.chassis_left_power == 0 && command.chassis_right_power == 0);
    input = {};
    input.connected = true;
    command = macrorec::map_input(input);
    CHECK(command.chassis_left_power == 0 && command.chassis_right_power == 0);
    CHECK(!macrorec::valid_frame({{128, 0, 0, 0, 0, true}, {}}));
    CHECK(!macrorec::valid_frame({{0, 0, 0, 0, 0x1000, true}, {}}));
    CHECK(!macrorec::valid_frame({{}, {-128, 0}}));
}

void test_record_replay() {
    MacroHardware hardware;
    macrorec::Engine engine(hardware);
    constexpr std::uint32_t kStartMs = 113;
    CHECK(engine.start_recording(kStartMs));
    CHECK(!engine.start_recording(kStartMs));
    for (std::uint32_t t = 0; t < 3000; t += 20) {
        engine.update(kStartMs + t, true, input_for(10));
        CHECK(hardware.writes == 0);
        CHECK(engine.state() == macrorec::State::Countdown);
        CHECK(hardware.output.chassis_left_power == 0);
    }
    for (std::size_t i = 0; i < macrorec::kFrameCount; ++i) {
        engine.update(kStartMs + 3000 + static_cast<std::uint32_t>(i) * 20, true, input_for(i));
        const auto &frame = engine.clip()[i];
        CHECK(frame.input.left_x == input_for(i).left_x);
        CHECK(frame.input.left_y == input_for(i).left_y);
        CHECK(frame.input.right_x == input_for(i).right_x);
        CHECK(frame.input.right_y == 127 && frame.input.buttons == input_for(i).buttons);
        CHECK(frame.command.chassis_left_power == hardware.output.chassis_left_power);
        CHECK(engine.frame_count() == i + 1);
        CHECK(engine.state() == macrorec::State::Recording);
        // 同一时间槽重复服务不能产生第二帧。
        engine.update(kStartMs + 3001 + static_cast<std::uint32_t>(i) * 20, true, input_for(0));
        CHECK(engine.frame_count() == i + 1);
    }
    CHECK(hardware.writes == 725);
    for (std::uint32_t t = 17500; t < 18000; t += 20) {
        engine.update(kStartMs + t, true, input_for(0));
        CHECK(engine.state() == macrorec::State::Resetting);
        CHECK(hardware.output.chassis_left_power == 0);
        CHECK(hardware.output.chassis_right_power == 0);
    }
    engine.update(kStartMs + 18000, true);
    CHECK(engine.state() == macrorec::State::Complete && !engine.busy());
    auto clip = engine.clip();
    clip[0].command = {-42, 73}; // 外部微调应覆盖原始轴映射。
    hardware.writes = 0;
    CHECK(engine.start_replay(clip, 50000));
    clip[0].command = {127, 127}; // 起跑后修改源缓冲区不改变执行快照。
    for (std::uint32_t t = 0; t < 18000; t += 20) {
        engine.update(50000 + t, true, {}); // 回放无需手柄连接。
        if (t < 3000) {
            CHECK(hardware.writes == 0);
        } else if (t == 3000) {
            CHECK(hardware.output.chassis_left_power == -42);
            CHECK(hardware.output.chassis_right_power == 73);
        } else if (t >= 17500) {
            CHECK(hardware.output.chassis_left_power == 0);
        }
    }
    engine.update(68000, true);
    CHECK(engine.state() == macrorec::State::Complete);
    CHECK(hardware.writes == 725);
}

void test_cancellation_and_faults() {
    MacroHardware hardware;
    macrorec::Engine engine(hardware);
    macrorec::Clip clip{};
    clip.fill({input_for(0), {100, -80}});
    const auto reach_frame_zero = [&](bool recording) {
        CHECK(recording ? engine.start_recording(0) : engine.start_replay(clip, 0));
        for (std::uint32_t t = 0; t <= 3000; t += 20) {
            engine.update(t, true, input_for(0));
        }
        CHECK(engine.busy());
    };
    reach_frame_zero(true);
    engine.update(3020, true, {});
    CHECK(engine.state() == macrorec::State::Fault);
    CHECK(hardware.output.chassis_left_power == 0);
    CHECK(engine.frame_count() == 1); // 断联不补零帧、不保存部分录制。

    reach_frame_zero(false);
    engine.update(3020, false);
    CHECK(engine.state() == macrorec::State::Fault);
    CHECK(hardware.output.chassis_left_power == 0);

    reach_frame_zero(false);
    engine.update(3040, true);
    CHECK(engine.state() == macrorec::State::Fault); // 跳过 3020ms 的一帧。
    CHECK(hardware.output.chassis_right_power == 0);

    reach_frame_zero(false);
    engine.cancel();
    CHECK(engine.state() == macrorec::State::Cancelled && engine.frame_count() == 0);
    engine.update(3020, true);
    CHECK(hardware.output.chassis_left_power == 0);

    reach_frame_zero(false);
    hardware.healthy = false;
    engine.update(3020, true);
    CHECK(engine.state() == macrorec::State::Fault);
    CHECK(hardware.output.chassis_left_power == 0);
    hardware.healthy = true;

    CHECK(engine.start_recording(0));
    engine.update(0, true);
    engine.update(41, true);
    CHECK(engine.state() == macrorec::State::Fault); // 包括倒计时在内的看门狗。
    clip[724].command.chassis_left_power = 128;
    CHECK(!engine.start_replay(clip, 0));
    CHECK(!engine.busy() && hardware.output.chassis_left_power == 0);

    // 毫秒计数器回绕不应延长倒计时或改变第一帧。
    constexpr auto kWrapStart = std::numeric_limits<std::uint32_t>::max() - 1000;
    CHECK(engine.start_recording(kWrapStart));
    for (std::uint32_t t = 0; t <= 3000; t += 20) {
        engine.update(kWrapStart + t, true, input_for(0));
    }
    CHECK(engine.state() == macrorec::State::Recording && engine.frame_count() == 1);
}

void test_scheduler_jitter() {
    MacroHardware hardware;
    macrorec::Engine engine(hardware);
    CHECK(engine.start_recording(0));
    for (std::uint32_t t = 0; t < 3000; t += 20) {
        engine.update(t, true);
    }
    // 点击发生在周期中间：第一帧晚 13ms，然后唤醒时间交替提前/滞后 1ms。
    for (std::size_t i = 0; i < macrorec::kFrameCount; ++i) {
        const auto jitter_ms = i == 0 ? 0 : (i % 2 == 0 ? 1 : -1);
        const auto t = static_cast<std::uint32_t>(3013 + i * 20 + jitter_ms);
        engine.update(t, true, input_for(i));
        CHECK(engine.state() == macrorec::State::Recording);
        CHECK(engine.frame_count() == i + 1);
    }
    for (std::uint32_t t = 17513; t <= 18013; t += 20) {
        engine.update(t, true);
        CHECK(hardware.output.chassis_left_power == 0);
    }
    CHECK(engine.state() == macrorec::State::Complete);
    CHECK(hardware.writes == 725);
}

void test_storage() {
    const auto ids = macrorec::catalog("macro_000003.csv\nother.txt\nmacro_000001.csv\r\n"
                                       "macro_000003.csv\nmacro_000000.csv\nmacro_000002.csv\n"
                                       "macro_999999.csv\nmacro_99999x.csv\nmacro_000004.csv.tmp");
    CHECK(ids.size() == 4 && ids[0] == 1 && ids[1] == 2 && ids[2] == 3 && ids[3] == 999999);
    CHECK(macrorec::catalog(nullptr).empty());
    std::array<char, 100> path{};
    CHECK(!macrorec::macro_path("/usd", 0, path.data(), path.size()));
    CHECK(!macrorec::macro_path("/usd", 1000000, path.data(), path.size()));
    CHECK(!macrorec::macro_path("/usd", 1, path.data(), 8));

    const auto directory = std::filesystem::temp_directory_path() / "vex-macro-protocol-test";
    std::filesystem::remove_all(directory);
    std::filesystem::create_directories(directory);
    const auto root = directory.string();
    macrorec::Clip clip{};
    for (std::size_t i = 0; i < clip.size(); ++i) {
        clip[i] = {input_for(i), macrorec::map_input(input_for(i))};
    }
    CHECK(macrorec::save_new_clip(root.c_str(), 1, clip));
    CHECK(!macrorec::save_new_clip(root.c_str(), 1, clip)); // 拒绝覆盖。
    CHECK(!macrorec::save_new_clip("/missing-vex-macro-dir", 2, clip));
    CHECK(macrorec::macro_path(root.c_str(), 1, path.data(), path.size()));
    macrorec::Clip loaded{};
    CHECK(macrorec::load_clip(path.data(), loaded));
    CHECK(loaded[724].input.buttons == clip[724].input.buttons);
    CHECK(loaded[724].command.chassis_left_power == clip[724].command.chassis_left_power);
    const auto source = directory / "macro_000001.csv";
    std::ifstream stream(source, std::ios::binary);
    const std::string text((std::istreambuf_iterator<char>(stream)), {});
    stream.close(); // Windows 上必须先释放文件句柄，否则末尾 remove_all 会失败。
    const auto corrupt = directory / "corrupt.csv";
    const auto load_text = [&](const std::string &modified) {
        std::ofstream output(corrupt, std::ios::binary);
        output << modified;
        output.close();
        return macrorec::load_clip(corrupt.string().c_str(), loaded);
    };
    CHECK(!load_text(text.substr(0, text.size() / 2)));
    CHECK(!load_text(text + "0,0,0,0,0,0,0,0\n"));
    CHECK(!load_text(text.substr(0, text.find("END,725"))));
    auto modified = text;
    modified.replace(modified.find("period_ms,20"), 12, "period_ms,10");
    CHECK(!load_text(modified));
    modified = text;
    modified.replace(modified.find("VEX_MACRO,1"), 11, "VEX_MACRO,2");
    CHECK(!load_text(modified));
    modified = text;
    modified.replace(modified.find("chassis_arcade_v1"), 17, "chassis_arcade_v2");
    CHECK(!load_text(modified));
    const auto first = text.find('\n', text.find("\nt_ms,") + 1) + 1;
    const auto first_end = text.find('\n', first);
    const auto with_first_row = [&](const char *row) {
        auto replacement = text;
        replacement.replace(first, first_end - first, row);
        return load_text(replacement);
    };
    CHECK(!with_first_row("20,0,0,0,0,0,0,0"));
    CHECK(!with_first_row("0,128,0,0,0,0,0,0"));
    CHECK(!with_first_row("0,0,0,0,0,4096,0,0"));
    CHECK(!with_first_row("0,0,0,0,0,0,-128,0"));
    CHECK(!with_first_row("0,0,0,0,0,0,2147483648,0"));
    CHECK(!with_first_row("0,0,0,0,0,0,0,0,1"));
    CHECK(!with_first_row("0,0,0,0,0,0,,0"));
    CHECK(with_first_row("0,0,0,0,0,0,-42,73"));
    CHECK(loaded[0].command.chassis_left_power == -42);
    std::string crlf;
    for (char ch : text) {
        if (ch == '\n') {
            crlf += '\r';
        }
        crlf += ch;
    }
    CHECK(load_text(crlf));
    clip[724].command.chassis_right_power = 128;
    CHECK(!macrorec::save_new_clip(root.c_str(), 2, clip));
    std::filesystem::remove_all(directory);
}
} // namespace

void test_macro() {
    test_mapping();
    test_record_replay();
    test_cancellation_and_faults();
    test_scheduler_jitter();
    test_storage();
}
