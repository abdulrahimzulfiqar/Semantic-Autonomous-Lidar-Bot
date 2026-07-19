#!/usr/bin/env python3
"""
benchmark_llms.py
-----------------
What is it?
A lightweight benchmarking script to evaluate local LLM performance (Qwen, TinyLlama) 
running via Ollama on a Raspberry Pi 4.

Why do we need it?
To assess token generation speed (tokens/sec), latency (Time to First Token), and model 
intelligence on resource-constrained hardware before writing ROS 2 integration code.

How does it work?
It communicates with the local Ollama REST API (http://localhost:11434) using Python's
built-in urllib library, runs a series of test prompts, and outputs timing statistics.
"""

import urllib.request
import urllib.error
import json
import time
import sys
import argparse

# Default connection settings
DEFAULT_OLLAMA_URL = "http://localhost:11434"
DEFAULT_MODELS = ["qwen2.5:0.5b", "tinyllama"]

# Test prompts designed for a wheelchair semantic navigation system
BENCHMARK_PROMPTS = [
    {
        "name": "Direct Command",
        "prompt": "Extract the destination name from the command: 'Go to the kitchen'. Respond with a JSON object in this format: {\"destination\": \"name\"}."
    },
    {
        "name": "Indirect Command",
        "prompt": "Extract the destination name from the command: 'I am really hungry, take me to where the food is'. Respond with a JSON object in this format: {\"destination\": \"name\"}."
    },
    {
        "name": "Safety/Greeting",
        "prompt": "Respond with a polite greeting to the user who just sat down on the autonomous wheelchair. Keep it under 15 words."
    }
]

def check_ollama(url):
    """Checks if the Ollama service is running."""
    try:
        req = urllib.request.Request(url, method="GET")
        with urllib.request.urlopen(req, timeout=3.0) as response:
            if response.status == 200:
                print(f"[✓] Connected to Ollama service at {url}")
                return True
    except Exception as e:
        print(f"[✗] Failed to connect to Ollama at {url}: {e}")
        print("\nHow to install and run Ollama on the Raspberry Pi:")
        print("  1. Run command to install: curl -fsSL https://ollama.com/install.sh | sh")
        print("  2. Start the service (if not auto-started): systemctl start ollama")
        print("  3. Pull your target models: ollama pull qwen2.5:0.5b")
        return False

def pull_model(url, model_name):
    """Pulls the model from Ollama library if not present."""
    print(f"\nChecking availability of model: {model_name}...")
    
    # Try generating a short response to check if model exists
    data = json.dumps({"model": model_name, "prompt": "test", "stream": False}).encode("utf-8")
    req = urllib.request.Request(
        f"{url}/api/generate",
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST"
    )
    
    try:
        with urllib.request.urlopen(req, timeout=5.0) as response:
            json.loads(response.read().decode("utf-8"))
            print(f"[✓] Model '{model_name}' is loaded and ready.")
            return True
    except urllib.error.HTTPError as e:
        if e.code == 404:
            print(f"[!] Model '{model_name}' not found locally. Pulling it now (this may take several minutes)...")
            pull_data = json.dumps({"name": model_name, "stream": False}).encode("utf-8")
            pull_req = urllib.request.Request(
                f"{url}/api/pull",
                data=pull_data,
                headers={"Content-Type": "application/json"},
                method="POST"
            )
            try:
                with urllib.request.urlopen(pull_req, timeout=300.0) as pull_resp:
                    pull_result = json.loads(pull_resp.read().decode("utf-8"))
                    if pull_result.get("status") == "success":
                        print(f"[✓] Successfully pulled model '{model_name}'")
                        return True
            except Exception as pull_err:
                print(f"[✗] Error pulling model: {pull_err}")
        else:
            print(f"[✗] HTTP Error checking model: {e}")
    except Exception as e:
        print(f"[✗] Error checking model availability: {e}")
    return False

def run_benchmark(url, model_name):
    """Runs the prompt list through the model and measures timings."""
    print("=" * 60)
    print(f" BENCHMARKING MODEL: {model_name}")
    print("=" * 60)
    
    results = []
    
    for i, p in enumerate(BENCHMARK_PROMPTS):
        print(f"\n[Test {i+1}/{len(BENCHMARK_PROMPTS)}] {p['name']}")
        print(f"  Prompt: \"{p['prompt']}\"")
        
        payload = {
            "model": model_name,
            "prompt": p['prompt'],
            "stream": False,
            "options": {
                "num_predict": 128,  # Restrict response length to prevent runs from dragging
                "temperature": 0.0,   # Keep response deterministic
                "num_ctx": 512       # Reduce context window to save RAM and speed up execution
            }
        }
        
        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            f"{url}/api/generate",
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST"
        )
        
        start_time = time.time()
        try:
            with urllib.request.urlopen(req, timeout=60.0) as response:
                raw_res = response.read().decode("utf-8")
                res = json.loads(raw_res)
                total_time = time.time() - start_time
                
                # Ollama returns timings in nanoseconds (divide by 1e9 to get seconds)
                eval_duration = res.get("eval_duration", 0) / 1e9
                eval_count = res.get("eval_count", 0)
                prompt_eval_duration = res.get("prompt_eval_duration", 0) / 1e9
                prompt_eval_count = res.get("prompt_eval_count", 0)
                
                tokens_per_second = eval_count / eval_duration if eval_duration > 0 else 0
                prompt_tokens_per_second = prompt_eval_count / prompt_eval_duration if prompt_eval_duration > 0 else 0
                
                print("  Response:")
                print(f"  ---")
                # Indent response lines for clean printing
                print("\n".join(f"    {line}" for line in res.get('response', '').strip().split('\n')))
                print(f"  ---")
                print(f"  Timings:")
                print(f"    - Total Request Duration: {total_time:.2f} s")
                print(f"    - Model Load Duration:   {res.get('load_duration', 0) / 1e9:.2f} s")
                print(f"    - Prompt Eval Speed:     {prompt_tokens_per_second:.2f} t/s ({prompt_eval_count} tokens in {prompt_eval_duration:.2f}s)")
                print(f"    - Token Generation Speed: {tokens_per_second:.2f} t/s ({eval_count} tokens in {eval_duration:.2f}s)")
                
                results.append({
                    "prompt_name": p['name'],
                    "total_time": total_time,
                    "tokens_per_second": tokens_per_second,
                    "token_count": eval_count,
                    "response": res.get('response', '').strip()
                })
        except Exception as e:
            print(f"  [✗] Prompt execution failed: {e}")
            
    # Calculate averages
    if results:
        avg_speed = sum(r['tokens_per_second'] for r in results) / len(results)
        avg_time = sum(r['total_time'] for r in results) / len(results)
        print("\n" + "-" * 40)
        print(f"Summary for {model_name}:")
        print(f"  Average Token Generation Speed: {avg_speed:.2f} tokens/sec")
        print(f"  Average Total Response Time:    {avg_time:.2f} seconds")
        print("-" * 40)
        return {"model": model_name, "avg_speed": avg_speed, "avg_time": avg_time}
    return None

def main():
    parser = argparse.ArgumentParser(description="Ollama Benchmark Tool for Raspberry Pi 4")
    parser.add_argument("--url", default=DEFAULT_OLLAMA_URL, help=f"Ollama server URL (default: {DEFAULT_OLLAMA_URL})")
    parser.add_argument("--models", nargs="+", default=DEFAULT_MODELS, help=f"Models to benchmark (default: {' '.join(DEFAULT_MODELS)})")
    args = parser.parse_args()
    
    print("============================================================")
    print(" Raspberry Pi 4 Local LLM Benchmarking Utility")
    print("============================================================")
    
    if not check_ollama(args.url):
        sys.exit(1)
        
    runs = []
    for model in args.models:
        if pull_model(args.url, model):
            summary = run_benchmark(args.url, model)
            if summary:
                runs.append(summary)
                
    if runs:
        print("\n" + "=" * 60)
        print(" FINAL BENCHMARK COMPARISON")
        print("=" * 60)
        for r in runs:
            print(f"Model: {r['model']:<18} | Speed: {r['avg_speed']:5.2f} tokens/s | Avg Time: {r['avg_time']:5.2f} s")
        print("=" * 60)

if __name__ == "__main__":
    main()
