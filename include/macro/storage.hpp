#pragma once

#include "macro/engine.hpp"

#include <cstddef>
#include <vector>

namespace macrorec {

constexpr int kMaxMacroId = 999999;

struct SaveError {
    const char *stage = "NONE";
    int error_number = 0;
};

// 从换行分隔的目录列表中提取 macro_NNNNNN.csv 编号（升序去重）。
std::vector<int> catalog(const char *listing);

// 生成 directory/macro_NNNNNN.csv 路径；编号越界或缓冲区不足返回 false。
bool macro_path(const char *directory, int id, char *path, std::size_t capacity);

// 读取并校验 v4 六行头部，用于列表过滤；旧版缺少反馈，不支持闭环回放。
bool compatible_clip(const char *path);

// 读取并完整校验 v4 宏文件（6 行头部 + 725 帧 + END,725 + EOF）。
bool load_clip(const char *path, Clip &clip);

// 写入新宏文件并回读校验，不覆盖已有文件；失败时记录阶段与 errno。
bool save_new_clip(const char *directory, int id, const Clip &clip, SaveError *error = nullptr,
                   const char *directory_listing = nullptr);

} // namespace macrorec
