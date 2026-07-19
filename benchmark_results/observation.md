# LLM Benchmarking Observations & Analysis
**Date:** July 10, 2026  
**Hardware:** Raspberry Pi 4 Model B (4GB RAM)  
**Task:** Comparative evaluation of local LLM performance under CPU execution.

---

## 📊 Side-by-Side Performance Matrix

This table summarizes all three benchmark runs executed on the Pi, comparing the **Reasoning** (Chain-of-Thought) model, the **Transformer-based** Qwen model, and the **State Space Model (SSM)** Liquid LFM model.

| Metric | 🧠 Qwen 0.8B Reasoning (`sorc/qwen3.5-instruct:0.8b`) | 🇨🇳 Qwen 0.8B Non-Reasoning (`sorc-qwen3.5-0.8B-instr-text-Nonreason`) | 🧪 Liquid LFM 1.2B (`LFM2.5-1.2B-inst-GGUf-Q4_K_M`) |
| :--- | :--- | :--- | :--- |
| **Architecture Type** | Transformer (Chain-of-Thought) | Transformer | Linear State Space Model (SSM) |
| **Parameters** | 0.8 Billion (800M) | 0.8 Billion (800M) | 1.2 Billion (1.2B) |
| **Tokens Generated** | **191 tokens** (High due to CoT) | **24 tokens** (Concise) | **27 tokens** (Concise) |
| **Total Duration** | 1 min 2.02 seconds | 9.00 seconds | **7.68 seconds** |
| **Model Load Time** | 2.13 seconds | 1.51 seconds | **0.65 seconds** |
| **Prompt Eval Rate** | 17.79 tokens/s | **57.08 tokens/s** | 12.11 tokens/s |
| **Text Generation Rate** | 3.25 tokens/s | 3.43 tokens/s | **4.73 tokens/s** |

---

## 🔍 Key Engineering Takeaways

### 1. The Cost of Reasoning (Chain-of-Thought)
* **Observation:** The reasoning model took **62.02 seconds** to reply compared to **9.00 seconds** for the non-reasoning version of the same size.
* **Why:** The speed of text generation on the Pi CPU is constrained to ~3.3 tokens/s. Because the reasoning model is forced to think "out loud", it generated **191 tokens** instead of **24 tokens**.
* **Wheelchair Navigation Constraint:** For robot navigation commands, we only need a concise, structured instruction (e.g., `{"goal": "exit"}`). CoT reasoning models are **unsuitable** for real-time local execution on the Pi.

### 2. State Space Models (LFM) vs. Transformers (Qwen)
Despite being 50% larger (1.2B vs 0.8B), the **Liquid LFM** model beat the Qwen Transformer model in total latency and writing speed:

* **Text Generation Speed (SSM Advantage):** LFM writes at **4.73 tokens/s** compared to Qwen's **3.43 tokens/s**. Because SSMs compress historical text context into a fixed-size mathematical state, the Pi's CPU does not have to perform heavy self-attention calculations for every new word.
* **Prompt Processing Speed (Transformer Advantage):** Qwen ingested the prompt at **57.08 tokens/s** compared to LFM's **12.11 tokens/s**. Traditional Transformers can process all prompt words in parallel across the Pi's quad cores, whereas SSMs must process them sequentially.

---

## 🧭 Recommendation for Wheelchair Navigation Planning

If we pass a **large topological map** (long input prompt) to target a **short goal coordinate** (short output response):

1. **Use Qwen 0.8B / 1.5B (Non-Reasoning):** Because mapping prompts are long, Qwen's superior reading speed (57 tokens/s) will parse the map much faster. The writing speed difference is negligible because the output is only a few tokens (e.g. coordinates or room name).
2. **Avoid LFM for Large Inputs:** LFM's sequential prompt evaluation (12 tokens/s) will cause a long lag time before the wheelchair starts moving if the input map is large.
