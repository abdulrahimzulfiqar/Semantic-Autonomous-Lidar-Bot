"""Periodic Pi process/host samples; CPU seconds and PSS, no timing inference."""
import datetime
import os
from pathlib import Path
import subprocess
import threading
import time


def read(path):
    try:
        return Path(path).read_text()
    except OSError:
        return ''


def firmware_throttled():
    """Sample the Pi's current and sticky throttling flags, when available."""
    try:
        return subprocess.check_output(
            ['vcgencmd', 'get_throttled'], text=True, stderr=subprocess.DEVNULL,
            timeout=.5).strip()
    except (OSError, subprocess.SubprocessError):
        return None


def sample():
    processes=[]
    for path in Path('/proc').glob('[0-9]*'):
        try:
            if b'ROS_DOMAIN_ID=218' not in (path/'environ').read_bytes().split(b'\0'):
                continue
            stat=(path/'stat').read_text().split(') ',1)[1].split()
            memory={line.split(':',1)[0]:int(line.split()[1]) for line in
                (path/'smaps_rollup').read_text().splitlines() if line.startswith(('Rss:','Pss:','Private_Dirty:'))}
            args=(path/'cmdline').read_bytes().decode(errors='replace').split('\0')
            processes.append(dict(pid=int(path.name),argv=args[:4],
                cpu_s=(int(stat[11])+int(stat[12]))/os.sysconf('SC_CLK_TCK'),memory_kib=memory))
        except (OSError,ValueError):
            pass
    memory={line.split(':',1)[0]:int(line.split()[1]) for line in read('/proc/meminfo').splitlines()
            if line.startswith(('MemTotal:','MemAvailable:','SwapTotal:','SwapFree:'))}
    return dict(wall_monotonic=time.monotonic(),utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        processes=processes,host_memory_kib=memory,
        firmware_throttled=firmware_throttled(),
        temperature_millidegrees_c=read('/sys/class/thermal/thermal_zone0/temp').strip(),
        cpu_frequency_khz=read('/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq').strip())


class Sampler:
    def __init__(self):
        self.rows=[]
        self.stopped=threading.Event()
        self.thread=threading.Thread(target=self.run,daemon=True)

    def run(self):
        while not self.stopped.is_set():
            self.rows.append(sample())
            self.stopped.wait(1.)

    def start(self):
        self.thread.start()

    def stop(self):
        self.stopped.set()
        self.thread.join(timeout=5)
