#include "motion/motion_tuning.hpp"
#include "config/motion_config.hpp"
#include "motion/motion_engine.hpp"
#include "tuning/param_apply.hpp"
#include "tuning/params_manager.hpp"

namespace motion {

MotionTuning::MotionTuning(MotionEngine &engine) : m_engine(&engine) {}

int MotionTuning::registerAll(tuning::ParamsManager &pm) {
    int count = 0;

    if (pm.addParam("vel_kp", cfg::kVelocityKp, 0.0, 10.0, tuning::ParamType::Double,
                    tuning::ParamGroup::PIDGroup, -3, 2) >= 0) {
        ++count;
    }
    if (pm.addParam("vel_ki", cfg::kVelocityKi, 0.0, 10.0, tuning::ParamType::Double,
                    tuning::ParamGroup::PIDGroup, -3, 2) >= 0) {
        ++count;
    }
    if (pm.addParam("vel_kd", cfg::kVelocityKd, 0.0, 10.0, tuning::ParamType::Double,
                    tuning::ParamGroup::PIDGroup, -3, 2) >= 0) {
        ++count;
    }

    return count;
}

int MotionTuning::bindAll(tuning::ParamsManager &pm) {
    (void)pm;
    if (!m_engine)
        return 0;

    int count = 0;
    if (tuning::ParamApply::bind("vel_kp", applyKp, this))
        ++count;
    if (tuning::ParamApply::bind("vel_ki", applyKi, this))
        ++count;
    if (tuning::ParamApply::bind("vel_kd", applyKd, this))
        ++count;
    return count;
}

void MotionTuning::applyKp(const tuning::TunableParam &p, void *context) {
    auto *self = static_cast<MotionTuning *>(context);
    if (self && self->m_engine) {
        self->m_engine->setVelocityKp(p.getDoubleValue());
    }
}

void MotionTuning::applyKi(const tuning::TunableParam &p, void *context) {
    auto *self = static_cast<MotionTuning *>(context);
    if (self && self->m_engine) {
        self->m_engine->setVelocityKi(p.getDoubleValue());
    }
}

void MotionTuning::applyKd(const tuning::TunableParam &p, void *context) {
    auto *self = static_cast<MotionTuning *>(context);
    if (self && self->m_engine) {
        self->m_engine->setVelocityKd(p.getDoubleValue());
    }
}

} // namespace motion
