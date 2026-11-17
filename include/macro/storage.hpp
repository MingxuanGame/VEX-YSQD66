#pragma once

#include "macro/engine.hpp"

#include <vector>

namespace macrorec {

constexpr int kMaxMacroId = 999999;

// 保存诊断；stage 指向静态阶段名称，无需释放，error_number 为失败时的 errno。
// SDK 未提供错误码或回读内容不匹配时可能为 0；参数/已有文件错误使用 EINVAL/EEXIST。
struct SaveError {
    const char *stage = "NONE";
    int error_number = 0;
};

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
// error 可为空；非空时成功清空诊断，失败保留首个错误，关闭文件不覆盖原始错误码。
// directory_listing 非空时，使用同一目录的完整、新鲜枚举替代 fopen 存在性探测。
// 调用方须保证枚举成功且未截断、从枚举到保存期间只有本任务写 SD；按 FAT32 忽略大小写。
// PROS 的不存在文件打开失败也会报告 ENFILE，不能据此断言文件不存在。
bool save_new_clip(const char *directory, int id, const Clip &clip, SaveError *error = nullptr,
                   const char *directory_listing = nullptr);

} // namespace macrorec
