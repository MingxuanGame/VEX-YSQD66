#include "macro/engine.hpp"
#include "macro/storage.hpp"
#include "test_common.hpp"

#include <array>
#include <cerrno>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {
class MacroHardware final : public macrorec::Hardware {
public:
    bool apply(const macrorec::Command &command) override;
    void stop() override;
    bool reset() override;
    macrorec::Command output{};
    int writes = 0;
    bool healthy = true;
    bool reset_healthy = true;
    int resets = 0;
};

bool MacroHardware::apply(const macrorec::Command &command) {
    output = command;
    ++writes;
    return healthy;
}

void MacroHardware::stop() {
    output = {};
}

bool MacroHardware::reset() {
    stop();
    ++resets;
    return reset_healthy;
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

void test_reset_and_exact_replay() {
    MacroHardware hardware;
    macrorec::Engine engine(hardware);
    hardware.output = {100, -100};
    engine.cancel(); // 对应启动/加载时的统一重置入口。
    CHECK(hardware.resets == 1 && hardware.output.chassis_left_power == 0);
    CHECK(engine.start_recording(0));
    CHECK(hardware.resets == 2);
    for (std::uint32_t t = 0; t < 3000; t += 20) {
        engine.update(t, true);
    }
    for (std::size_t i = 0; i < macrorec::kFrameCount; ++i) {
        engine.update(3000 + i * 20, true, input_for(i));
        CHECK(hardware.resets == 3); // 首帧前重置，其后不打断采样。
    }
    const auto clip = engine.clip();
    for (std::uint32_t t = 17500; t <= 18000; t += 20) {
        engine.update(t, true);
        CHECK(hardware.resets == 4); // 末尾 500ms 只停止，不反复清零。
    }
    CHECK(engine.start_replay(clip, 20000));
    for (std::uint32_t t = 20000; t < 23000; t += 20) {
        engine.update(t, true, input_for(99));
    }
    for (std::size_t i = 0; i < macrorec::kFrameCount; ++i) {
        engine.update(23000 + i * 20, true, input_for(99));
        CHECK(hardware.output.chassis_left_power == clip[i].command.chassis_left_power);
        CHECK(hardware.output.chassis_right_power == clip[i].command.chassis_right_power);
    }
    engine.cancel();
    hardware.reset_healthy = false;
    const int writes = hardware.writes;
    CHECK(!engine.start_recording(40000));
    CHECK(engine.state() == macrorec::State::Fault && hardware.writes == writes);
    CHECK(std::string(engine.status()) == "COMPONENT RESET FAILED");
    hardware.reset_healthy = true;
    CHECK(engine.start_recording(40000));
    for (std::uint32_t t = 40000; t < 43000; t += 20) {
        engine.update(t, true);
    }
    hardware.reset_healthy = false;
    engine.update(43000, true, input_for(0));
    CHECK(engine.state() == macrorec::State::Fault && hardware.writes == writes);
}

void test_scheduler_jitter() {
    class TimedHardware final : public macrorec::Hardware {
    public:
        std::uint32_t now = 0;
        std::uint32_t reset_cost_ms = 0;
        std::uint32_t write_cost_ms = 0;
        std::uint32_t stop_cost_ms = 0;
        bool holding = false;
        std::vector<std::uint32_t> writes;
        std::vector<std::uint32_t> stops;

        bool apply(const macrorec::Command &) override {
            now += write_cost_ms;
            writes.push_back(now);
            holding = true;
            return true;
        }

        void stop() override {
            if (holding) {
                now += stop_cost_ms;
                stops.push_back(now);
            }
            holding = false;
        }

        bool reset() override {
            stop();
            now += reset_cost_ms;
            return true;
        }

        std::uint32_t time_ms(std::uint32_t) const override {
            return now;
        }
    };

    // 1ms 服务，包含首帧重置耗时和倒计时相位偏差；每个成功输出必须保持整整 20ms。
    for (const bool recording : {true, false}) {
        TimedHardware hardware;
        hardware.reset_cost_ms = 7;
        macrorec::Engine engine(hardware);
        macrorec::Clip clip{};
        CHECK(recording ? engine.start_recording(0) : engine.start_replay(clip, 0));
        while (engine.busy() && hardware.now < 19000) {
            engine.update(hardware.now, true, input_for(7));
            ++hardware.now;
        }
        CHECK(engine.state() == macrorec::State::Complete);
        CHECK(hardware.writes.size() == macrorec::kFrameCount);
        CHECK(hardware.writes.front() == 3014); // 两次重置各耗时 7ms，实际写入后才建立时基。
        for (std::size_t i = 1; i < hardware.writes.size(); ++i) {
            CHECK(hardware.writes[i] - hardware.writes[i - 1] == 20);
        }
        CHECK(hardware.stops.size() == 1);
        CHECK(hardware.stops.back() - hardware.writes.back() == 20);
    }
    const auto prepare = [](TimedHardware &hardware, macrorec::Engine &engine) {
        CHECK(engine.start_recording(0));
        for (hardware.now = 0; hardware.now <= 3000; hardware.now += 20) {
            engine.update(hardware.now, true, input_for(7));
        }
        CHECK(hardware.writes.size() == 1);
    };
    // 原来会成功执行 39ms/1ms 的序列，现在延迟帧直接故障，不得继续追赶。
    TimedHardware delayed;
    macrorec::Engine delayed_engine(delayed);
    prepare(delayed, delayed_engine);
    delayed.now = 3039;
    delayed_engine.update(3039, true, input_for(7));
    delayed.now = 3040;
    delayed_engine.update(3040, true, input_for(7));
    CHECK(delayed_engine.state() == macrorec::State::Fault);
    CHECK(std::string(delayed_engine.status()) == "FRAME DEADLINE MISSED");
    CHECK(delayed.writes.size() == 1 && !delayed.holding);

    TimedHardware early;
    macrorec::Engine early_engine(early);
    prepare(early, early_engine);
    early.now = 3019;
    early_engine.update(3019, true, input_for(7));
    CHECK(early.writes.size() == 1);
    early.now = 3020;
    early_engine.update(3020, true, input_for(7));
    CHECK(early.writes.size() == 2 && early.writes[1] - early.writes[0] == 20);
    // SDK 写入延迟也纳入检查，不能只看调用前的时间戳。
    early.now = 3040;
    early.write_cost_ms = 1;
    early_engine.update(3040, true, input_for(7));
    CHECK(early_engine.state() == macrorec::State::Fault && !early.holding);

    TimedHardware tail;
    macrorec::Engine tail_engine(tail);
    prepare(tail, tail_engine);
    for (tail.now = 3020; tail.now < 17500; tail.now += 20) {
        tail_engine.update(tail.now, true, input_for(7));
    }
    CHECK(tail.writes.size() == 725);
    tail.now = 17501;
    tail_engine.update(17500, true); // 旧的调用参数不能掩盖真实时钟已经迟到。
    CHECK(tail_engine.state() == macrorec::State::Fault && !tail.holding);

    TimedHardware checkpoint;
    macrorec::Engine checkpoint_engine(checkpoint);
    macrorec::Clip prefix{};
    prefix[0].checkpoint = true;
    CHECK(checkpoint_engine.start_from_checkpoint(prefix, 1, 0));
    for (checkpoint.now = 0; checkpoint.now <= 3000; checkpoint.now += 20) {
        checkpoint_engine.update(checkpoint.now, true);
    }
    checkpoint.stop_cost_ms = 1;
    checkpoint.now = 3020;
    checkpoint_engine.update(3020, true);
    CHECK(checkpoint_engine.state() == macrorec::State::Fault && !checkpoint.holding);
    CHECK(checkpoint.stops.back() - checkpoint.writes.back() == 21);
}

void test_checkpoints() {
    MacroHardware hardware;
    macrorec::Engine engine(hardware);
    CHECK(engine.start_recording(0));
    for (std::uint32_t t = 0; t < 3000; t += 20) {
        CHECK(!engine.needs_input(t));
        engine.update(t, true);
    }
    // L1 长按只产生一个检查点，释放后再次按下才能增加。
    for (std::size_t i = 0; i < macrorec::kFrameCount; ++i) {
        auto input = input_for(i);
        input.buttons = i == 0 || i == 1 || i == 3 ? macrorec::kCheckpointButton : 0;
        engine.update(3000 + i * 20, true, input);
        CHECK(engine.clip()[i].checkpoint == (i == 0 || i == 3));
    }
    engine.cancel();

    macrorec::Clip source{};
    for (std::size_t i = 0; i < source.size(); ++i) {
        source[i] = {input_for(i), {static_cast<int>(i % 127), -80}, true};
    }
    CHECK(!engine.start_from_checkpoint(source, 0, 0));
    CHECK(!engine.start_from_checkpoint(source, macrorec::kFrameCount + 1, 0));
    source[1].checkpoint = false;
    CHECK(!engine.start_from_checkpoint(source, 2, 0));

    // 首帧、中间及倒数第二帧的边界都应保留前缀并重新录满剩余部分。
    for (const std::size_t boundary : {1U, 101U, 724U}) {
        hardware.writes = 0;
        CHECK(engine.start_from_checkpoint(source, boundary, 0));
        CHECK(engine.recording());
        CHECK(!engine.start_from_checkpoint(source, boundary, 0));
        for (std::uint32_t t = 0; t < 3000 + boundary * 20; t += 20) {
            CHECK(!engine.needs_input(t));
            engine.update(t, true); // 前缀无需手柄连接。
        }
        CHECK(hardware.writes == static_cast<int>(boundary));
        const auto pause_ms = static_cast<std::uint32_t>(3000 + boundary * 20);
        // 周期提前 1ms 时，不得执行已清空的后缀或提前开始暂停。
        engine.update(pause_ms - 1, true);
        CHECK(hardware.writes == static_cast<int>(boundary));
        engine.update(pause_ms, true);
        CHECK(engine.state() == macrorec::State::CheckpointPause);
        CHECK(engine.elapsed_ms(pause_ms) == 0);
        for (std::uint32_t t = 20; t < 3000; t += 20) {
            CHECK(!engine.needs_input(pause_ms + t));
            engine.update(pause_ms + t, true, input_for(99));
            CHECK(hardware.output.chassis_left_power == 0);
            CHECK(hardware.writes == static_cast<int>(boundary));
        }
        const auto resume_ms = pause_ms + 3000;
        CHECK(engine.needs_input(resume_ms));
        for (std::size_t i = boundary; i < macrorec::kFrameCount; ++i) {
            auto input = input_for(55);
            input.buttons = i == boundary || i == boundary + 2 ? macrorec::kCheckpointButton : 0;
            engine.update(resume_ms + (i - boundary) * 20, true, input);
            CHECK(engine.state() == macrorec::State::Recording);
            CHECK(engine.clip()[i].input.left_x == input.left_x);
            CHECK(engine.clip()[i].checkpoint == (i == boundary + 2));
        }
        for (std::size_t i = 0; i < boundary; ++i) {
            CHECK(engine.clip()[i].command.chassis_left_power ==
                  source[i].command.chassis_left_power);
            CHECK(engine.clip()[i].checkpoint == source[i].checkpoint);
        }
        CHECK(source[boundary].checkpoint == (boundary != 1)); // 不修改加载源文件快照。
        const auto reset_ms = resume_ms + (macrorec::kFrameCount - boundary) * 20;
        for (std::uint32_t t = 0; t <= 500; t += 20) {
            engine.update(reset_ms + t, true);
            CHECK(hardware.output.chassis_right_power == 0);
        }
        CHECK(engine.state() == macrorec::State::Complete);
        CHECK(engine.frame_count() == macrorec::kFrameCount);
        CHECK(hardware.writes == 725);
    }

    // 结尾仅回放，不停车三秒续录，也不生成待保存录制。
    CHECK(engine.start_from_checkpoint(source, macrorec::kFrameCount, 0));
    CHECK(!engine.recording());
    for (std::uint32_t t = 0; t <= 18000; t += 20) {
        CHECK(!engine.needs_input(t));
        engine.update(t, true);
        CHECK(engine.state() != macrorec::State::CheckpointPause);
    }
    CHECK(engine.state() == macrorec::State::Complete);

    const auto reach_pause = [&]() {
        CHECK(engine.start_from_checkpoint(source, 1, 0));
        for (std::uint32_t t = 0; t <= 3020; t += 20) {
            engine.update(t, true);
        }
        CHECK(engine.state() == macrorec::State::CheckpointPause);
    };
    reach_pause();
    engine.update(3040, false);
    CHECK(engine.state() == macrorec::State::Fault);
    CHECK(hardware.output.chassis_left_power == 0);
    reach_pause();
    engine.cancel();
    engine.update(3040, true, input_for(0));
    CHECK(engine.state() == macrorec::State::Cancelled);
    reach_pause();
    engine.update(3061, true);
    CHECK(engine.state() == macrorec::State::Fault);
    reach_pause();
    for (std::uint32_t t = 3040; t < 6020; t += 20) {
        engine.update(t, true);
    }
    engine.update(6020, true, {}); // 交还驾驶的首帧断联，必须故障停车。
    CHECK(engine.state() == macrorec::State::Fault);
    CHECK(engine.frame_count() == 1);
    CHECK(hardware.output.chassis_left_power == 0);
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
        clip[i].checkpoint = i == 0 || i == 100 || i == 724;
    }
    macrorec::SaveError error{"OLD ERROR", EIO};
    CHECK(macrorec::save_new_clip(root.c_str(), 1, clip, &error));
    CHECK(std::string(error.stage) == "NONE" && error.error_number == 0);
    CHECK(!macrorec::save_new_clip(root.c_str(), 1, clip, &error)); // 拒绝覆盖。
    CHECK(std::string(error.stage) == "EXISTS" && error.error_number == EEXIST);
    CHECK(!macrorec::save_new_clip("/missing-vex-macro-dir", 2, clip, &error));
    CHECK(std::string(error.stage) == "CREATE" && error.error_number == ENOENT);
    CHECK(!macrorec::save_new_clip(root.c_str(), 0, clip, &error));
    CHECK(std::string(error.stage) == "PATH" && error.error_number == EINVAL);
    // PROS 把不存在文件的 fopen 失败映射为 ENFILE；真机使用成功枚举，避免该探测。
    CHECK(macrorec::save_new_clip(root.c_str(), 2, clip, &error, "macro_000001.csv\n"));
    CHECK(
        !macrorec::save_new_clip(root.c_str(), 2, clip, &error, "other.txt\nMACRO_000002.CSV\r\n"));
    CHECK(std::string(error.stage) == "EXISTS" && error.error_number == EEXIST);
    CHECK(macrorec::save_new_clip(root.c_str(), 3, clip, &error,
                                  "macro_000003.csv.tmp\nmacro_000030.csv\n"));
    CHECK(!macrorec::save_new_clip(root.c_str(), 3, clip, &error, "macro_000003.csv"));
    CHECK(std::string(error.stage) == "EXISTS" && error.error_number == EEXIST);
    CHECK(macrorec::macro_path(root.c_str(), 1, path.data(), path.size()));
    macrorec::Clip loaded{};
    CHECK(macrorec::load_clip(path.data(), loaded));
    CHECK(loaded[724].input.buttons == clip[724].input.buttons);
    CHECK(loaded[724].command.chassis_left_power == clip[724].command.chassis_left_power);
    for (std::size_t i = 0; i < clip.size(); ++i) {
        CHECK(loaded[i].checkpoint == clip[i].checkpoint);
    }
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
    modified.replace(modified.find("VEX_MACRO,3"), 11, "VEX_MACRO,4");
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
    CHECK(!with_first_row("20,0,0,0,0,0,0,0,0"));
    CHECK(!with_first_row("0,128,0,0,0,0,0,0,0"));
    CHECK(!with_first_row("0,0,0,0,0,4096,0,0,0"));
    CHECK(!with_first_row("0,0,0,0,0,0,-128,0,0"));
    CHECK(!with_first_row("0,0,0,0,0,0,2147483648,0,0"));
    CHECK(!with_first_row("0,0,0,0,0,0,0,0"));
    CHECK(!with_first_row("0,0,0,0,0,0,,0,0"));
    CHECK(!with_first_row("0,0,0,0,0,0,0,0,2"));
    CHECK(!with_first_row("0,0,0,0,0,0,0,0,-1"));
    CHECK(!with_first_row("0,0,0,0,0,0,0,0,0,1"));
    CHECK(with_first_row("0,0,0,0,0,0,-42,73,1"));
    CHECK(loaded[0].command.chassis_left_power == -42);
    CHECK(loaded[0].checkpoint);
    CHECK(macrorec::compatible_clip(source.string().c_str()));
    CHECK(!load_text("VEX_MACRO,1" + text.substr(text.find('\n'))));
    CHECK(!macrorec::compatible_clip(corrupt.string().c_str()));
    CHECK(!load_text("VEX_MACRO,2" + text.substr(text.find('\n'))));
    CHECK(!macrorec::compatible_clip(corrupt.string().c_str()));
    std::string crlf;
    for (char ch : text) {
        if (ch == '\n') {
            crlf += '\r';
        }
        crlf += ch;
    }
    CHECK(load_text(crlf));
    clip[724].command.chassis_right_power = 128;
    CHECK(!macrorec::save_new_clip(root.c_str(), 2, clip, &error));
    CHECK(std::string(error.stage) == "VALIDATE" && error.error_number == EINVAL);
    std::filesystem::remove_all(directory);
}
} // namespace

void test_macro() {
    test_mapping();
    test_record_replay();
    test_reset_and_exact_replay();
    test_cancellation_and_faults();
    test_scheduler_jitter();
    test_checkpoints();
    test_storage();
}
