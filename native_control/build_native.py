"""Build the team's C attitude controller as a shared library.

Windows examples:
  python native_control/build_native.py
The script tries gcc, clang and cl in that order. No Python package is required.
"""
from __future__ import annotations
import os, platform, shutil, subprocess, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
BIN = ROOT / "bin"
BIN.mkdir(exist_ok=True)

system = platform.system()
if system == "Windows":
    out = BIN / "controlo_atitude.dll"
    candidates = [
        (["gcc", "-shared", "-O2", "-std=c11", "-o", str(out), "controlo_atitude.c", "export_wrapper.c"], "gcc"),
        (["clang", "-shared", "-O2", "-std=c11", "-o", str(out), "controlo_atitude.c", "export_wrapper.c"], "clang"),
        (["cl", "/nologo", "/O2", "/LD", "controlo_atitude.c", "export_wrapper.c", f"/Fe:{out}"], "cl"),
    ]
elif system == "Darwin":
    out = BIN / "libcontrolo_atitude.dylib"
    candidates = [(["clang", "-dynamiclib", "-O2", "-std=c11", "-o", str(out), "controlo_atitude.c", "export_wrapper.c"], "clang")]
else:
    out = BIN / "libcontrolo_atitude.so"
    candidates = [(["gcc", "-shared", "-fPIC", "-O2", "-std=c11", "-o", str(out), "controlo_atitude.c", "export_wrapper.c", "-lm"], "gcc"),
                  (["clang", "-shared", "-fPIC", "-O2", "-std=c11", "-o", str(out), "controlo_atitude.c", "export_wrapper.c", "-lm"], "clang")]

for cmd, compiler in candidates:
    if shutil.which(cmd[0]):
        print(f"Building with {compiler}: {' '.join(cmd)}")
        subprocess.run(cmd, cwd=ROOT, check=True)
        print(f"Built {out}")
        raise SystemExit(0)

print("No supported C compiler found. Install GCC/MinGW-w64, clang, or MSVC Build Tools.", file=sys.stderr)
raise SystemExit(2)
