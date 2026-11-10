# VEX-yuanshen

基于 PROS 的 VEX V5 机器人 C/C++ 工程。

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
