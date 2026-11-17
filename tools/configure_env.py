"""Detect this computer's ARM toolchain and configure the project environment."""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


TOOL_NAMES = {"clangd": "clangd", "cc": "arm-none-eabi-gcc",
              "cxx": "arm-none-eabi-g++", "pros": "pros",
              "make": "make"}


def search_directories():
    """Search bounded installation locations; never scan the whole drive."""
    home = Path.home()
    patterns = [str(home / ".local/bin"), str(home / ".local/share/uv/tools/pros-cli/bin"),
                str(home / ".vscode/extensions/llvm-vs-code-extensions.vscode-clangd-*/install/*/bin"),
                str(home / ".config/Code/User/globalStorage/sigbots.pros/install/*/bin")]
    if os.name == "nt":
        for name in ("ProgramFiles", "ProgramFiles(x86)", "LOCALAPPDATA", "APPDATA"):
            if os.environ.get(name):
                base = Path(os.environ[name])
                patterns += [str(base / "LLVM/bin"), str(base / "Arm GNU Toolchain*/bin"),
                             str(base / "Arm GNU Toolchain*/*/bin"),
                             str(base / "Programs/LLVM/bin"),
                             str(base / "Programs/pros-cli/Scripts"),
                             str(base / "Code/User/globalStorage/sigbots.pros/install/*/bin"),
                             str(base / "Code/User/globalStorage/sigbots.pros/install/*/usr/bin"),
                             str(base / "Code/User/globalStorage/sigbots.pros/install/pros-toolchain-windows/usr/bin"),
                             str(base / "Code/User/globalStorage/sigbots.pros/install/pros-cli-windows"),
                             str(base / "Microsoft Visual Studio/*/*/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin"),
                             str(base / "Microsoft/WinGet/Packages/*LLVM*/*/bin"),
                             str(base / "Microsoft/WinGet/Packages/*MSYS2*/ucrt64/bin")]
        import string
        for drive in string.ascii_uppercase:
            if Path(f"{drive}:/").exists():
                patterns += [f"{drive}:/msys64/ucrt64/bin", f"{drive}:/msys64/mingw64/bin",
                             f"{drive}:/PROS/usr/bin", f"{drive}:/WinGetPackages/*MSYS2*/ucrt64/bin",
                             f"{drive}:/WinGetPackages/*LLVM*/*/bin"]
    else:
        patterns += ["/usr/bin", "/usr/local/bin", "/usr/lib/llvm-*/bin",
                     "/opt/homebrew/bin", "/opt/homebrew/opt/llvm/bin", "/usr/local/opt/llvm/bin"]
    import glob
    return list(dict.fromkeys(path for pattern in patterns for path in sorted(glob.glob(pattern), reverse=True)
                              if Path(path).is_dir()))


def detect_tool(name, explicit, previous, env, directories, required=True):
    command = TOOL_NAMES[name]
    if explicit:
        found = shutil.which(explicit, path=env["PATH"])
        if not found:
            raise ValueError(f"Cannot find --{name}: {explicit}")
    else:
        found = shutil.which(command, path=env["PATH"])
        if not found:
            found = shutil.which(command, path=os.pathsep.join(directories))
        if not found and previous:
            found = shutil.which(previous, path=env["PATH"])
    if not found and required:
        raise ValueError(f"Cannot find {command}. Install it, add its bin directory to PATH, or pass --{name} PATH.")
    return Path(found).resolve().as_posix() if found else None


def flags(language):
    # Read the target, macros, and language standards from the build rules.
    common = (ROOT / "common.mk").read_text(encoding="utf-8")
    makefile = (ROOT / "Makefile").read_text(encoding="utf-8")
    def assignment(text, key):
        match = re.search(r"^" + re.escape(key) + r"\s*[:?]?=\s*(.*)$", text, re.M)
        if not match:
            raise ValueError(f"Missing {key} in build configuration")
        value = match.group(1).split("#", 1)[0].strip()
        if "$" in value:
            raise ValueError(f"{key} uses Make expansion; update tools/configure_env.py for these flags")
        return value.split()
    result = assignment(common, "MFLAGS") + assignment(common, "CPPFLAGS")
    standard = "CXX_STANDARD" if language == "c++" else "C_STANDARD"
    result += ["-std=" + assignment(common, standard)[0]]
    result += assignment(makefile, "EXTRA_CXXFLAGS" if language == "c++" else "EXTRA_CFLAGS")
    for suffix in ("h", "hpp"):
        if (ROOT / f"include/liblvgl/llemu.{suffix}").exists():
            result.append(f"-D_PROS_INCLUDE_LIBLVGL_LLEMU_{suffix.upper()}")
    return result


def system_includes(compiler, options, language, env):
    # Query with the real CPU/FPU flags so multilib headers match VEX V5.
    query = subprocess.run([compiler, *options, "-E", "-v", "-x", language, "-"],
                           input="", text=True, capture_output=True, env=env, cwd=ROOT, check=True)
    paths = []
    collecting = False
    for line in query.stderr.splitlines():
        if "#include <...> search starts here:" in line:
            collecting = True
        elif "End of search list." in line:
            break
        elif collecting and line.strip():
            paths.extend(["-isystem", Path(line.strip()).resolve().as_posix()])
    if not paths:
        raise ValueError(f"Could not read system include paths from {compiler}")
    return paths


def check_project_templates():
    """project.pros 的 template location 可能来自另一台机器；只警告，不阻塞构建。"""
    project = ROOT / "project.pros"
    if not project.exists():
        return
    try:
        data = json.loads(project.read_text(encoding="utf-8-sig"))
        templates = data.get("py/state", {}).get("templates", {})
    except (OSError, ValueError):
        return
    for name, template in templates.items():
        location = template.get("location")
        if location and not Path(location).exists():
            print(f"warning: PROS template '{name}' not found at {location} "
                  f"(pros build still works; pros c fetch/upgrade may fail)", flush=True)


def configure(config, env, check):
    clangd = config["clangd"]
    subprocess.run([clangd, "--version"], env=env, cwd=ROOT, check=True, capture_output=True)
    entries = []
    for language, suffixes, key in (("c++", (".cpp", ".cc", ".cxx", ".c++"), "cxx"), ("c", (".c",), "cc")):
        sources = sorted(p for p in (ROOT / "src").rglob("*") if p.suffix in suffixes)
        if not sources:
            continue
        compiler = config[key]
        target = subprocess.run([compiler, "-dumpmachine"], env=env, cwd=ROOT,
                                text=True, capture_output=True, check=True).stdout.strip()
        if not target.startswith("arm-none-eabi"):
            raise ValueError(f"Expected an ARM bare-metal compiler, got {target}: {compiler}")
        options = flags(language)
        includes = system_includes(compiler, options, language, env)
        for source in sources:
            relative = source.relative_to(ROOT / "src")
            arguments = [compiler, *options, *includes, "-iquote", (ROOT / "include").as_posix(),
                         "-iquote", (ROOT / "include" / relative.parent).as_posix(), "-c", source.as_posix()]
            entries.append({"directory": ROOT.as_posix(), "file": source.as_posix(), "arguments": arguments})
    if not entries:
        raise ValueError("No C/C++ sources found in src")
    settings = json.loads((ROOT / ".vscode/settings.example.json").read_text(encoding="utf-8"))
    settings["clangd.path"] = clangd
    settings["clangd.arguments"] += ["--compile-commands-dir=" + ROOT.as_posix()]
    local_settings = ROOT / ".vscode/settings.json"
    if local_settings.exists():
        current = json.loads(local_settings.read_text(encoding="utf-8-sig"))
        # Preserve unrelated personal editor settings, remove obsolete tool paths.
        current.pop("terminal.integrated.env.linux", None)
        current.update(settings)
        settings = current
    write_json(ROOT / "compile_commands.json", entries)
    write_json(local_settings, settings)
    cpp = json.loads((ROOT / ".vscode/c_cpp_properties.example.json").read_text(encoding="utf-8"))
    cpp["configurations"][0]["compilerPath"] = config["cxx"]
    write_json(ROOT / ".vscode/c_cpp_properties.json", cpp)
    tasks = json.loads((ROOT / ".vscode/tasks.example.json").read_text(encoding="utf-8"))
    for task in tasks["tasks"]:
        command = task.get("command")
        if command in ("pros", "make"):
            task["command"] = config.get(command) or command
            task.setdefault("options", {})["env"] = {
                **config.get("environment", {}), "PATH": env["PATH"]}
        elif command == "python":
            task["command"] = sys.executable
            task.setdefault("options", {})["env"] = {
                **config.get("environment", {}), "PATH": env["PATH"]}
    write_json(ROOT / ".vscode/tasks.json", tasks)
    print(f"Configured clangd: {clangd}\nGenerated {len(entries)} compile commands.", flush=True)
    if check:
        failed = 0
        for entry in entries:
            result = subprocess.run(
                [clangd, "--check=" + entry["file"],
                 "--compile-commands-dir=" + ROOT.as_posix()],
                cwd=ROOT, env=env, text=True, capture_output=True)
            text = (result.stdout or "") + (result.stderr or "")
            if result.returncode == 0:
                continue
            # clangd 22 的 --check 会执行 "tweak"（如 ExtractFunction），
            # 某些语法结构会让 tweak 自身失败并返回非零，但下面仍是
            # "All checks completed, 1 errors"。这类内部 tweak 失败不代表源码有错。
            completed = re.findall(r"All checks completed, (\d+) errors?", text)
            tweak_failures = len(re.findall(r"tweak: .*FAIL", text))
            if completed and 0 < int(completed[-1]) <= tweak_failures:
                print(f"warning: clangd internal tweak failed (ignored): {entry['file']}", flush=True)
                continue
            failed += 1
            print(f"clangd check failed: {entry['file']}", flush=True)
            print(text, file=sys.stderr)
        if failed:
            raise ValueError(f"{failed} source file(s) failed clangd check")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in TOOL_NAMES:
        parser.add_argument("--" + name, help=f"Override the detected {TOOL_NAMES[name]} executable")
    parser.add_argument("--check", action="store_true", help="Run clangd --check on all sources after configuration")
    args = parser.parse_args()
    local = ROOT / "toolchain.local.json"
    config = json.loads(local.read_text(encoding="utf-8-sig")) if local.exists() else {}
    env = dict(os.environ)
    env.update(config.get("environment", {}))
    env["PATH"] = os.pathsep.join([*config.get("extraPath", []), env.get("PATH", "")])
    directories = search_directories()
    for name in TOOL_NAMES:
        config[name] = detect_tool(name, getattr(args, name), config.get(name), env, directories,
                                   required=name in ("clangd", "cc", "cxx"))
        print(f"{name}: {config[name] or 'not found (optional)'}", flush=True)
    # Make compiler and PROS dependencies available to spawned tools and VS Code tasks.
    paths = [str(Path(config[name]).parent) for name in TOOL_NAMES if config[name]]
    for path in list(paths):
        parent = Path(path)
        if parent.parent.name in ("ucrt64", "mingw64"):
            shell_bin = parent.parent.parent / "usr/bin"
            if shell_bin.is_dir():
                paths.append(str(shell_bin))
    config["extraPath"] = list(dict.fromkeys([*paths, *config.get("extraPath", [])]))
    if config.get("cxx"):
        # PROS CLI 通过 PROS_TOOLCHAIN 查找 usr/bin 下的 arm-none-eabi-* 和 make。
        config.setdefault("environment", {})["PROS_TOOLCHAIN"] = str(Path(config["cxx"]).parent.parent)
        env["PROS_TOOLCHAIN"] = config["environment"]["PROS_TOOLCHAIN"]
    env["PATH"] = os.pathsep.join([*config["extraPath"], env["PATH"]])
    env["LC_ALL"] = "C"
    check_project_templates()
    write_json(local, config)
    configure(config, env, args.check)


if __name__ == "__main__":
    try:
        main()
    except (ValueError, OSError, subprocess.CalledProcessError) as error:
        print(f"Error: {error}", file=sys.stderr)
        sys.exit(1)
