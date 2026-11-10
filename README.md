# VEX-yuanshen

基于 PROS 的 VEX V5 机器人 C/C++ 工程。

## 开发规范与编辑器配置

- `.clang-format`：C/C++ 四空格缩进、大括号不换行、函数间空行；`.clang-format-ignore` 为独立 clang-format 排除 SDK、固件和生成物，编辑器中也仅格式化自有代码。
- `.editorconfig`：UTF-8、LF、文件末尾换行；Makefile 保留 Tab，JSON 使用两空格。
- `.gitattributes`：统一 Git 文本换行为 LF，固件、归档和图片按二进制处理。
- `.clang-tidy`：检查部分命名规则、命名空间导入及常见误用；端口和比赛行为仍需代码检视及实机验证。
- `.vscode/settings.example.json`：通过 clangd 按仓库规则保存时格式化 C/C++，关闭自动推断缩进。运行下方配置脚本后同步到本机设置。

按 VS Code 扩展建议安装 clangd 和 EditorConfig 扩展。使用独立格式化命令时，需要 `clang-format`；VS Code 的 clangd 可直接提供格式化。命名诊断由 clangd 的 clang-tidy 集成提供，也可使用独立 `clang-tidy`。这些配置已用 LLVM 22 验证，旧版本出现未知选项时先核对工具版本。

仅对本次修改的自有 C/C++ 文件执行格式化，例如：

```powershell
clang-format -i src/chassis.cpp include/chassis.h
clang-format --dry-run --Werror src/chassis.cpp include/chassis.h
```

## 自动配置环境

在项目根目录执行：

```powershell
python tools/configure_env.py
```

脚本从当前电脑查找 clangd、ARM GCC/G++ 和可选的 PROS CLI，打印找到的路径并生成本地配置。查找顺序为：命令行指定的路径、当前 PATH、常见安装目录、已有本地配置中的有效路径。

常见目录包括 Windows 的 MSYS2、LLVM、PROS、部分 WinGet 安装目录和 VS Code 扩展工具目录，以及 Linux/macOS 的系统 bin、LLVM 和 Homebrew 目录。脚本只搜索指定目录，不扫描整块磁盘。自定义位置未被发现时，将工具的 bin 目录加入 PATH，或直接指定：

```powershell
python tools/configure_env.py --clangd "D:/LLVM/bin/clangd.exe" --cc "D:/toolchain/bin/arm-none-eabi-gcc.exe" --cxx "D:/toolchain/bin/arm-none-eabi-g++.exe"
```

可用 `--pros PATH` 指定 PROS CLI。未找到 PROS 时仍可配置 clangd，构建和上传需要另外安装 PROS。

配置并检查所有源文件：

```powershell
python tools/configure_env.py --check
```

脚本验证编译器目标为 ARM 裸机平台，读取当前 Makefile 的目标参数、预处理宏和语言标准，再用 GCC 查询对应 CPU/FPU 的标准库头文件目录。`--check` 调用 clangd 检查每个 C/C++ 源文件；全部出现 `All checks completed, 0 errors` 且退出码为 0 即表示通过。

在 VS Code 中打开项目文件夹，然后执行 `clangd: Restart language server`。

## 编译和上传

使用 VS Code 的 `PROS: Build`、`PROS: Rebuild`、`PROS: Upload`、`PROS: Terminal` 和 `PROS: Build & Upload` 任务；任务直接调用检测到的 PROS CLI，并携带本地环境设置。`PROS: Update compile_commands` 会重新运行配置脚本。

`PROS: Check sources` 会运行 `tools/configure_env.py --check`，用于配置并检查所有源文件。

若 PROS 已在终端 PATH 中，也可以直接使用：

```powershell
pros build
pros build all
pros upload
pros terminal
```

## 常见问题

- **找不到工具**：安装对应工具、添加 bin 目录到 PATH，或用 `--clangd` / `--cc` / `--cxx` / `--pros` 指定路径。
- **找不到头文件**：重新生成配置，确认 ARM 工具链带有标准库头文件。不要从其他机器复制编译数据库。
- **clangd 无法启动或仍使用旧路径**：查看 VS Code 的 clangd 输出面板，重新配置并重启语言服务器，检查用户或工作区设置是否覆盖项目配置。
- **Make 或 shell 缺失**：补全 PROS 构建环境；clangd 检查通过不代表完整固件构建环境已就绪。
