#pragma once
#include <cstdint>

namespace io {
struct InputState;
} // namespace io

namespace control {

enum class Mode { Drive, Tune, Auto, SafeWait };

class ModeManager {
public:
    ModeManager();

    // 每 20ms 调一次；只处理按键和状态流转，不直接操作硬件。
    Mode update(const io::InputState &input, uint32_t now_ms);

    Mode getMode() const;
    bool isTuneExitPending() const;
    bool consumeTuneExit();

    void startAuto();
    void endAuto();
    void reset();
    void requestSafeWait();
    void clearSafeWait();

private:
    Mode m_mode;
    bool m_tune_exit_pending;
};

} // namespace control
