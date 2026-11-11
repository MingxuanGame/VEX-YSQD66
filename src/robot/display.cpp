#include "robot/pros_display.hpp"
#include "robot/display.hpp"
#include "pros/llemu.hpp"

namespace robot {

namespace {

class ProsDisplay : public Display {
public:
    void clearLine(int line) override {
        pros::lcd::clear_line(line);
    }

    void printLine(int line, const char *text) override {
        if (text)
            pros::lcd::print(line, "%s", text);
    }
};

} // namespace

Display &prosDisplay() {
    static ProsDisplay display;
    return display;
}

} // namespace robot
