"""ctypes bridge to the team's C yaw controller.

Only the functions that exist in the team's feature branch are exposed here:
yaw outer loop, yaw rate/torque loop, and roll/pitch damping. 3-axis attitude
control remains a separate simulator extension and is not attributed to this C code.
"""
from __future__ import annotations
import ctypes
import platform
import subprocess
import sys
from pathlib import Path


class NativeControllerError(RuntimeError):
    pass


class NativeYawController:
    def __init__(self, auto_build: bool = True) -> None:
        root = Path(__file__).resolve().parents[1] / "native_control"
        if platform.system() == "Windows":
            lib = root / "bin" / "controlo_atitude.dll"
        elif platform.system() == "Darwin":
            lib = root / "bin" / "libcontrolo_atitude.dylib"
        else:
            lib = root / "bin" / "libcontrolo_atitude.so"
        if not lib.exists() and auto_build:
            proc = subprocess.run([sys.executable, str(root / "build_native.py")], cwd=root,
                                  capture_output=True, text=True)
            if proc.returncode != 0:
                raise NativeControllerError(
                    "C controller is not built. Run `python native_control/build_native.py` "
                    "with GCC/MinGW-w64, clang, or MSVC available.\n" + (proc.stderr or proc.stdout))
        if not lib.exists():
            raise NativeControllerError(f"Native controller library not found: {lib}")
        self.lib = ctypes.CDLL(str(lib))
        self.lib.gs_init_controladores.argtypes = []
        self.lib.gs_init_controladores.restype = None
        self.lib.gs_reset_controladores.argtypes = []
        self.lib.gs_reset_controladores.restype = None
        self.lib.gs_malha_externa_yaw.argtypes = [ctypes.c_float, ctypes.c_float]
        self.lib.gs_malha_externa_yaw.restype = ctypes.c_float
        self.lib.gs_malha_intermedia_yaw.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.gs_malha_intermedia_yaw.restype = ctypes.c_float
        self.lib.gs_damping_roll_pitch.argtypes = [ctypes.c_float, ctypes.c_float]
        self.lib.gs_damping_roll_pitch.restype = ctypes.c_float
        self.lib.gs_init_controladores()

    def reset(self) -> None:
        self.lib.gs_reset_controladores()

    def yaw_outer(self, position_error_deg: float, dt: float) -> float:
        return float(self.lib.gs_malha_externa_yaw(position_error_deg, dt))

    def yaw_inner(self, desired_rate_rad_s: float, actual_rate_rad_s: float, dt: float) -> float:
        return float(self.lib.gs_malha_intermedia_yaw(desired_rate_rad_s, actual_rate_rad_s, dt))

    def damping_body_torque(self, actual_rate_rad_s: float, gain: float) -> float:
        return float(self.lib.gs_damping_roll_pitch(actual_rate_rad_s, gain))
