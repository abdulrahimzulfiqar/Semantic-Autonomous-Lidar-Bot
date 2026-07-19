#!/usr/bin/env python3
"""
monitor_performance.py
----------------------
What is it?
A lightweight diagnostic monitor that logs CPU temperature and RAM usage on a 
Raspberry Pi over time. It supports dynamic run names, saves logs to a CSV file 
inside a dedicated directory, and generates an interactive HTML graph using Chart.js on exit.

Why do we need it?
To visualize resource consumption during different experimental states (e.g. idle baseline 
vs running different LLM configurations), without loading down the Pi.

How does it work?
On launch, you specify a session name (e.g. `python3 monitor_performance.py --name qwen_0.5b`).
It records stats to `benchmark_results/perf_<name>.csv` and outputs a self-contained 
graphing page to `benchmark_results/chart_<name>.html`.
"""

import time
import os
import sys
import csv
import argparse
from datetime import datetime

def get_temp():
    """Reads the CPU temperature in Celsius."""
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            temp_raw = int(f.read().strip())
            return temp_raw / 1000.0
    except Exception:
        return 0.0

def get_ram_usage():
    """Reads RAM usage in MB by parsing /proc/meminfo."""
    try:
        with open("/proc/meminfo", "r") as f:
            lines = f.readlines()
        mem_info = {}
        for line in lines:
            parts = line.split()
            if len(parts) >= 2:
                mem_info[parts[0].replace(":", "")] = int(parts[1])
        
        total = mem_info.get("MemTotal", 0) / 1024.0  # Convert KB to MB
        free = mem_info.get("MemFree", 0) / 1024.0
        buffers = mem_info.get("Buffers", 0) / 1024.0
        cached = mem_info.get("Cached", 0) / 1024.0
        
        # Calculate active used memory
        used = total - (free + buffers + cached)
        return used, total
    except Exception:
        return 0.0, 0.0

def draw_ascii_chart(data, width=50, height=12):
    """Draws a fallback ASCII line-art chart of temperature data in the terminal."""
    if not data:
        return
    
    temps = [d[1] for d in data]
    min_temp = min(temps)
    max_temp = max(temps)
    temp_range = max_temp - min_temp if max_temp != min_temp else 1.0
    
    step = max(1, len(temps) // width)
    chart_temps = temps[::step][:width]
    
    print("\n" + "=" * 65)
    print(f" TEMPERATURE TIMELINE GRAPH ({min_temp:.1f}°C - {max_temp:.1f}°C)")
    print("=" * 65)
    
    for y in range(height, -1, -1):
        val = min_temp + (temp_range * (y / height))
        row_str = f"{val:5.1f}°C | "
        
        for t in chart_temps:
            normalized_t = int(((t - min_temp) / temp_range) * height)
            if normalized_t == y:
                row_str += "*"
            elif normalized_t > y:
                row_str += " "
            else:
                row_str += " "
        print(row_str)
        
    print("      |" + "-" * len(chart_temps))
    print(f"      | Started: {data[0][0]}  ===>  Ended: {data[-1][0]}")
    print("=" * 65)

def write_interactive_html(html_path, session_name, data):
    """Generates a beautiful, interactive HTML dashboard using Chart.js."""
    if not data:
        return
    
    timestamps = [f"'{d[0]}'" for d in data]
    temps = [str(round(d[1], 2)) for d in data]
    rams = [str(round(d[2], 2)) for d in data]
    
    html_content = f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>PI Performance Chart - {session_name}</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        body {{
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background-color: #f0f2f5;
            color: #333;
            margin: 0;
            padding: 30px;
        }}
        .container {{
            max-width: 1000px;
            margin: 0 auto;
            background: #ffffff;
            padding: 25px;
            border-radius: 12px;
            box-shadow: 0 4px 15px rgba(0,0,0,0.08);
        }}
        h1 {{
            font-size: 24px;
            margin-bottom: 5px;
            color: #1e293b;
        }}
        .subtitle {{
            color: #64748b;
            font-size: 14px;
            margin-bottom: 30px;
        }}
        .card-row {{
            display: flex;
            gap: 20px;
            margin-bottom: 30px;
        }}
        .card {{
            flex: 1;
            padding: 15px 20px;
            background: #f8fafc;
            border-radius: 8px;
            border: 1px solid #e2e8f0;
        }}
        .card-title {{
            font-size: 12px;
            text-transform: uppercase;
            letter-spacing: 1px;
            color: #64748b;
            margin-bottom: 5px;
        }}
        .card-value {{
            font-size: 20px;
            font-weight: bold;
            color: #0f172a;
        }}
        .chart-box {{
            position: relative;
            height: 450px;
            width: 100%;
        }}
    </style>
</head>
<body>
    <div class="container">
        <h1>PI Telemetry Report</h1>
        <div class="subtitle">Session Name: <strong>{session_name}</strong> | Generated: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}</div>
        
        <div class="card-row">
            <div class="card">
                <div class="card-title">Max Temp</div>
                <div class="card-value">{max(float(t) for t in temps):.1f} °C</div>
            </div>
            <div class="card">
                <div class="card-title">Min Temp</div>
                <div class="card-value">{min(float(t) for t in temps):.1f} °C</div>
            </div>
            <div class="card">
                <div class="card-title">Max RAM Used</div>
                <div class="card-value">{max(float(r) for r in rams):.1f} MB</div>
            </div>
        </div>

        <div class="chart-box">
            <canvas id="performanceChart"></canvas>
        </div>
    </div>

    <script>
        const ctx = document.getElementById('performanceChart').getContext('2d');
        const performanceChart = new Chart(ctx, {{
            type: 'line',
            data: {{
                labels: [{', '.join(timestamps)}],
                datasets: [
                    {{
                        label: 'CPU Temperature (°C)',
                        data: [{', '.join(temps)}],
                        borderColor: '#ef4444',
                        backgroundColor: 'rgba(239, 68, 68, 0.1)',
                        yAxisID: 'yTemp',
                        borderWidth: 2,
                        tension: 0.3,
                        pointRadius: 2
                    }},
                    {{
                        label: 'RAM Usage (MB)',
                        data: [{', '.join(rams)}],
                        borderColor: '#3b82f6',
                        backgroundColor: 'rgba(59, 130, 246, 0.1)',
                        yAxisID: 'yRam',
                        borderWidth: 2,
                        tension: 0.3,
                        pointRadius: 2
                    }}
                ]
            }},
            options: {{
                responsive: true,
                maintainAspectRatio: false,
                interaction: {{
                    mode: 'index',
                    intersect: false,
                }},
                scales: {{
                    x: {{
                        grid: {{ display: false }}
                    }},
                    yTemp: {{
                        type: 'linear',
                        display: true,
                        position: 'left',
                        suggestedMin: 30,
                        suggestedMax: 85,
                        title: {{
                            display: true,
                            text: 'Temperature (°C)',
                            color: '#ef4444'
                        }},
                        grid: {{ drawOnChartArea: true }}
                    }},
                    yRam: {{
                        type: 'linear',
                        display: true,
                        position: 'right',
                        suggestedMin: 0,
                        suggestedMax: 4000,
                        title: {{
                            display: true,
                            text: 'RAM Used (MB)',
                            color: '#3b82f6'
                        }},
                        grid: {{ drawOnChartArea: false }}
                    }}
                }}
            }}
        }});
    </script>
</body>
</html>
"""
    try:
        with open(html_path, "w") as f:
            f.write(html_content)
        print(f"[✓] Interactive chart saved successfully to: {os.path.abspath(html_path)}")
    except Exception as e:
        print(f"[✗] Failed to save HTML chart: {e}")

def main():
    parser = argparse.ArgumentParser(description="Performance Logger & Charting Tool")
    parser.add_argument("--name", default="session", help="Session suffix name (e.g. 'baseline', 'qwen05b')")
    parser.add_argument("--outdir", default="benchmark_results", help="Directory where logs are saved")
    args = parser.parse_args()
    
    # Resolve directory path
    outdir_path = os.path.abspath(args.outdir)
    os.makedirs(outdir_path, exist_ok=True)
    
    # Construct filenames
    safe_name = "".join(c for c in args.name if c.isalnum() or c in ("_", "-")).rstrip()
    csv_path = os.path.join(outdir_path, f"perf_{safe_name}.csv")
    html_path = os.path.join(outdir_path, f"chart_{safe_name}.html")
    
    print("============================================================")
    print(" Raspberry Pi Performance Monitor & Logger")
    print("============================================================")
    print(f"[*] Target Directory: {outdir_path}")
    print(f"[*] Session Name:     {safe_name}")
    print(f"[*] CSV Output:       perf_{safe_name}.csv")
    print(f"[*] HTML Chart:       chart_{safe_name}.html")
    print("[*] Press Ctrl+C to stop logging and generate reports.")
    print("-" * 60)
    print(f"{'Timestamp':<10} | {'Temp (°C)':<9} | {'RAM Used (MB)':<13} | {'RAM Total (MB)':<14}")
    print("-" * 60)
    
    log_data = []
    
    # Initialize CSV header
    try:
        with open(csv_path, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Timestamp", "Temperature (C)", "RAM Used (MB)", "RAM Total (MB)"])
    except Exception as e:
        print(f"[✗] Error initializing CSV file: {e}")
        sys.exit(1)
        
    try:
        while True:
            now_str = datetime.now().strftime("%H:%M:%S")
            temp = get_temp()
            used_ram, total_ram = get_ram_usage()
            
            # Print status to stdout
            print(f"{now_str:<10} | {temp:9.1f} | {used_ram:13.1f} | {total_ram:14.1f}")
            
            log_data.append((now_str, temp, used_ram))
            
            # Append data row to CSV
            with open(csv_path, "a", newline="") as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([now_str, temp, used_ram, total_ram])
                
            time.sleep(2.0)
    except KeyboardInterrupt:
        print("\n\n[-] Monitoring stopped.")
        if log_data:
            draw_ascii_chart(log_data)
            write_interactive_html(html_path, safe_name, log_data)
            print(f"[✓] CSV data saved to: {os.path.abspath(csv_path)}")
        else:
            print("[!] No data logged.")

if __name__ == "__main__":
    main()
