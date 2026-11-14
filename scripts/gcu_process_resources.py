"""Read-only per-process Linux RSS/CPU samples; RSS sums include shared pages."""
import os
from pathlib import Path
import time


def sample(domain='195'):
    roles={'planner_server','controller_server','bt_navigator','amcl','map_server','ekf_node'}
    values=[]
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            if ('ROS_DOMAIN_ID='+domain).encode() not in (proc/'environ').read_bytes().split(b'\0'):continue
            args=(proc/'cmdline').read_bytes().split(b'\0');role=Path(args[0].decode()).name
            if role not in roles:continue
            stat=(proc/'stat').read_text().split(') ',1)[1].split()
            rss=next(int(line.split()[1]) for line in (proc/'status').read_text().splitlines() if line.startswith('VmRSS:'))
            values.append(dict(pid=int(proc.name),role=role,rss_kib=rss,
                cpu_s=(int(stat[11])+int(stat[12]))/os.sysconf('SC_CLK_TCK')))
        except (OSError,ValueError,StopIteration):continue
    return dict(wall=time.monotonic(),processes=values,scope='Per-process VmRSS, not PSS or whole-stack RAM; shared pages may be counted repeatedly')
