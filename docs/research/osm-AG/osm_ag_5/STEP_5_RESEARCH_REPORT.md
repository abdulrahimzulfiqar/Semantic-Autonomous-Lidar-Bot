# Step 5 Research Report: osmAG-LLM Open-Vocabulary Object Navigation

- **Paper Title**: *osmAG-LLM: Zero-Shot Open-Vocabulary Object Navigation via Semantic Maps and Large Language Models Reasoning* (IEEE RA-L Accepted Dec 2025 / arXiv:2507.12753)
- **Authors**: Fujing Xie, Sören Schwertfeger, Hermann Blum (ShanghaiTech & ETH Zurich)
- **GitHub Repository**: [https://github.com/xiexiexiaoxiexie/osmAG-LLM](https://github.com/xiexiexiaoxiexie/osmAG-LLM)
- **Pipeline Order**: **Step 5 (Zero-Shot Object-Goal Navigation & Active Search)**

---

## 1. What is the Author Doing?

### **What is it?**
This paper addresses **Object-Goal Navigation** ("Find the coffee mug" or "Find the printer") using zero-shot LLM reasoning combined with `osmAG` semantic maps.

### **Why do we need it?**
High-detail visual-language 3D maps (like 3D Gaussian Splatting or CLIP-voxels) are computationally heavy and quickly become outdated when objects are moved. `osmAG-LLM` uses lightweight topological room maps (`osmAG`) and leverages LLM commonsense knowledge to reason about where an unmapped object is likely located (e.g., "coffee mug $\rightarrow$ kitchen or lounge"), guiding the robot to navigate to the target room for active visual detection.

### **How does it work?**
1. **Open-Vocabulary Prompt Query**: User inputs a text target: *"Find my red laptop bag"*.
2. **LLM Room Probability Ranking**: The LLM evaluates the `osmAG` room graph and ranks candidate rooms based on commonsense likelihood.
3. **Topological Room Navigation**: The robot uses `osmAG` path planning to drive to the highest-ranked room.
4. **Active Visual Search**: Once inside the room, an online visual detector (YOLO / CLIP / OW-DETR) scans the room to locate the object.

---

## 2. GitHub Repository Analysis & Inclusions

- **Code Provided**: Python evaluation codebase built for simulated benchmark environments (HM3D-SEM / Habitat simulator).
- **Missing for Real Robot**: Built primarily for Habitat simulator benchmark evaluation. It **does not** provide a real-world ROS 2 Humble hardware driver stack, LiDAR bringup, or wheelchair motor control nodes.
