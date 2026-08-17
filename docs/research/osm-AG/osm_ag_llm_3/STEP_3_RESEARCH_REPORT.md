# Step 3 Research Report: LLM spatial comprehension on osmAG

- **Paper Title**: *Can Large Language Models Understand Spatial Topological Maps? An Empirical Study on osmAG* (arXiv:2403.08228)
- **Authors**: Fujing Xie, Sören Schwertfeger, et al. (ShanghaiTech University)
- **GitHub Repository**: [https://github.com/xiefujing/LLM-osmAG-Comprehension](https://github.com/xiefujing/LLM-osmAG-Comprehension)
- **Pipeline Order**: **Step 3 (Immediate post-osmAG LLM Benchmark)**

---

## 1. What is the Author Doing?

### **What is it?**
This paper conducts an empirical benchmark evaluating how well state-of-the-art Large Language Models (GPT-4, Claude 3, LLaMA-3) understand spatial environments when formatted as textual **osmAG XML graphs**.

### **Why do we need it?**
Robots operating in complex indoor environments need high-level spatial reasoning. Before using an LLM to navigate a real robot, we must evaluate whether an LLM can parse textual XML topometric representations, reason about room adjacencies, and compute valid topological room paths.

### **How does it work?**
1. **Input Encoding**: Converts the topological map (`.osm` XML) into textual prompt context describing areas, rooms, boundaries, and passage doors.
2. **Task Evaluation**: Evaluates LLMs across 3 core spatial tasks:
   - **Task A: Room Connectivity & Adjacency**: "Is Room 1 connected to Room 3 through Door 2?"
   - **Task B: Topological Shortest Path**: "What is the sequence of rooms and doors to travel from Entrance to Office 2?"
   - **Task C: Spatial Inclusion**: "Which room area contains Object X given boundary coordinates?"
3. **Prompt Engineering**: Benchmarks Zero-Shot, Few-Shot, and Chain-of-Thought (CoT) prompting techniques.

---

## 2. GitHub Repository Analysis & Inclusions

- **Code Provided**: Python evaluation scripts that format `.osm` XML strings into OpenAI/Claude API prompts.
- **Missing for Real Robot**: Contains **zero ROS 2 nodes**, no Gazebo simulation files, no hardware drivers, and no Nav2 local controller bridges.
