#include "test_common.hpp"
#include "tuning/tuning.hpp"

namespace {

int g_apply_calls = 0;

void onParam(const tuning::TunableParam &) {
    ++g_apply_calls;
}

} // namespace

void test_tuning() {
    using namespace tuning;

    ParamsManager pm;
    CHECK(pm.addParam("unit_kp", 1.0, 0.0, 10.0, ParamType::Double, ParamGroup::PIDGroup, -2, 2) >=
          0);

    TunableParam *p = pm.getParam("unit_kp");
    CHECK(p != nullptr);
    if (!p)
        return;

    // 调整编辑位后改变范围，默认值不能被“当前值”顶掉。
    p->moveDigitPos(MoveDirection::Right);
    CHECK(p->getParamDigitPos() == -1);
    p->setValueRange(0.0, 5.0);
    p->resetToDefault();
    CHECK(p->getDoubleValue() == 1.0);

    // 序列化必须把编辑位也带回来。
    p->incParam();
    p->saveParam();
    char line[256] = {};
    CHECK(p->serialize(line, sizeof(line)));
    TunableParam q;
    CHECK(q.deserialize(line));
    CHECK(q.getParamDigitPos() == -1);
    CHECK(q.getDoubleValue() == p->getDoubleValue());
    CHECK(q.isDirty());

    // 已注册参数的默认值不能被读盘值覆盖，否则 resetToDefault() 会回到保存值。
    TunableParam registered("unit_kp", 1.0, 0.0, 10.0, ParamType::Double, ParamGroup::PIDGroup, -2,
                            2);
    CHECK(registered.deserialize(line));
    CHECK(registered.getDoubleValue() == p->getDoubleValue());
    registered.resetToDefault();
    CHECK(registered.getDoubleValue() == 1.0);

    // 非法 ParamType 必须被拒绝，避免后续 std::get 抛 bad_variant_access。
    TunableParam invalid("unit_kp", 1.0, 0.0, 10.0, ParamType::Double, ParamGroup::PIDGroup, -2, 2);
    CHECK(!invalid.deserialize("unit_kp|2|1.0|0|1|1|0.0|10.0"));

    // ParamApply 绑定后应调用 handler 并清除 dirty。
    CHECK(ParamApply::bind("unit_kp", onParam));
    p->incParam();
    CHECK(p->isDirty());
    ParamApply::applyAllDirty(pm);
    CHECK(g_apply_calls == 1);
    CHECK(!p->isDirty());

    // 重名参数应被拒绝，而不是静默覆盖。
    CHECK(pm.addParam("unit_kp", 1.0, 0.0, 1.0, ParamType::Double, ParamGroup::PIDGroup, 0, 0) ==
          -1);
}
