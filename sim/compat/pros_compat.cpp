#include "sim/compat/pros_compat.hpp"

#ifdef ENABLE_SIMULATOR

namespace pros {

void delay(uint32_t ms) {
    sim::delay(ms);
}

uint32_t millis() {
    return sim::millis();
}

} // namespace pros

#endif
