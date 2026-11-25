#!/usr/bin/env python3
"""Read-only Linux inventory for the Phase 09 Pi baseline; emits JSON to stdout.

No package installation, service control, ROS launch or robot commands. Run with
the system Python, without sourcing a workspace, to record the host environment.
"""
import datetime
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess


def read(path):
    try:
        return Path(path).read_text().strip().strip("\x00")
    except (OSError, UnicodeError):
        return None


def probe(command):
    if not shutil.which(command[0]):
        return {"available": False}
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=15)
        return {"available": True, "exit": result.returncode,
                "stdout": result.stdout, "stderr": result.stderr}
    except subprocess.TimeoutExpired:
        return {"available": True, "timeout_s": 15}


def collect():
    return {
        "scope": "Read-only hardware/software inventory; not a navigation benchmark",
        "captured_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "collector_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "hostname": platform.node(), "architecture": platform.machine(),
        "kernel": platform.release(), "python": platform.python_version(),
        "model": read("/proc/device-tree/model"),
        "os_release": read("/etc/os-release"), "memory": read("/proc/meminfo"),
        "cpu": read("/proc/cpuinfo"),
        "temperature_millidegrees_c": read("/sys/class/thermal/thermal_zone0/temp"),
        "cpu_scaling": {p.name: read(p) for p in
                        Path("/sys/devices/system/cpu/cpu0/cpufreq").glob("scaling_*")
                        if p.is_file()},
        "python_modules": {name: importlib.util.find_spec(name) is not None
                           for name in ["numpy", "scipy", "shapely", "yaml", "pyproj", "cv2", "PIL"]},
        "commands": {
            "ros_packages": probe(["dpkg-query", "-W", "-f=${db:Status-Abbrev} ${Package} ${Version}\n", "ros-humble-*"]),
            "python_packages": probe(["dpkg-query", "-W", "-f=${db:Status-Abbrev} ${Package} ${Version}\n", "python3-scipy", "python3-shapely", "python3-pyproj"]),
            "network": probe(["ip", "-j", "address"]),
            "routes": probe(["ip", "-j", "route"]),
            "storage": probe(["df", "-B1", "/"]),
            "throttled": probe(["vcgencmd", "get_throttled"]),
            "temperature_firmware": probe(["vcgencmd", "measure_temp"]),
        },
        "physical_details_unverified": ["power supply", "cooling/fan", "ambient temperature"],
        "uid": os.getuid(),
    }


if __name__ == "__main__":
    print(json.dumps(collect(), indent=2))
