#pragma once

#include "macro/engine.hpp"

#include <vector>

namespace macrorec {

constexpr int kMaxMacroId = 999999;

// 从 SDK 提供的换行分隔文件名中提取 macro_NNNNNN.csv，按编号排序去重。
// 非阻塞，不验证内容；损坏文件同样占用编号，防止覆盖；其他名称被忽略。
std::vector<int> catalog(const char *listing);
// 路径 directory/macro_NNNNNN.csv；非法编号/超长路径返回 false，不修改文件。
bool macro_path(const char *directory, int id, char *path, std::size_t capacity);
// 同步阻塞 IO，只能停车后调用。严格验证 v1/v2 协议、725 帧、尾标记和 EOF。
// v1 没有检查点，加载时全部置 false；v2 检查点列仅允许 0/1。
// 失败不返回可用宏，调用方不得使用部分载入的数据。
bool load_clip(const char *path, Clip &clip);
// 同步以 v2 写入不存在的新文件，flush/close 后逐帧（含检查点）回读校验，成功才返回 true。
// 不覆盖任何既有文件；失败可能留下不完整文件并占用编号，下次应递增。
bool save_new_clip(const char *directory, int id, const Clip &clip);

} // namespace macrorec
