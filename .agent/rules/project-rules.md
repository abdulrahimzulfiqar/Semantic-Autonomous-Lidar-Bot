---
trigger: always_on
---

# Project Agent Rules — Semantic Autonomous Indoor Navigation

These rules govern ALL work on this repository. Follow them without exception.

---

## 1. Teaching-First Approach

- Before writing any code or making any change, **explain what we are about to do and why**, as if the reader has never seen this technology before.
- Use plain language first, then introduce technical terms with inline definitions.
- Every explanation must answer three questions: **What is it? Why do we need it? How does it work?**
- When introducing a ROS 2 concept (node, topic, service, action, launch file, parameter), explain it in one sentence before using it.
- Never assume prior knowledge of SLAM, YOLO, Nav2, sensor fusion, NLP, or LLM integration.

---

## 2. Modular Folder Architecture

Every distinct capability in the pipeline MUST live in its own self-contained directory. Never mix concerns across folders.

### Required Top-Level Structure

```
├── docs/                        # All project-wide documentation
│   ├── architecture/            # System diagrams, data-flow, design decisions
│   ├── tutorials/               # Step-by-step learning guides
│   ├── guides/                  # Goal-oriented how-to guides
│   ├── reference/               # API docs, message definitions, parameter tables
│   └── research/                # Paper drafts, literature review, citations
├── lidarbot_ws/                 # ROS 2 Workspace (contains packages in src/)
│   └── src/
│       ├── lidarbot/            # Core sub-packages (bringup, slam, navigation, etc.)
│       └── rplidar_ros/         # External LiDAR driver package
├── web_app/                     # React dashboard
├── firmware/                    # Arduino .ino files and embedded code
├── simulation/                  # Gazebo worlds, simulated sensor configs
├── tests/                       # Integration tests that span multiple modules
├── scripts/                     # Helper scripts (setup, calibration, benchmarking)
├── assets/                      # Images, videos, diagrams for README/docs
├── docker/                      # Dockerfiles for reproducible environments
├── .github/                     # CI/CD workflows, issue templates, PR templates
├── README.md                    # Project entry point (concise, links to docs/)
├── CONTRIBUTING.md              # How to contribute
├── CHANGELOG.md                 # Version history
├── LICENSE                      # Open-source license
└── .repos                       # vcstool file for external ROS 2 dependencies
```

### Per-Module Rules

- Every module directory under `src/` MUST contain its own `README.md` explaining:
  - What this module does (one paragraph)
  - What inputs it takes (topics subscribed, services called)
  - What outputs it produces (topics published, services offered)
  - How to run it standalone for testing
  - Configuration parameters (table format)
  - Dependencies
- Every module MUST have a `config/` folder for YAML parameter files. Never hardcode parameters.
- Every module MUST have a `launch/` folder with at least one launch file.
- Every module MUST have a `test/` folder (even if initially empty with a placeholder).

---

## 3. Documentation Standards

### 3.1 README.md (Root)

The root README is the front door. Keep it **concise and scannable**. It must contain:

- Project title and one-line description
- Status badges (build, docs, license, ROS 2 distro)
- A hero image or GIF showing the system in action
- A system architecture diagram (Mermaid preferred)
- A "Quick Start" section (user should be able to run the project in under 10 minutes)
- A table of contents linking to `docs/` for deeper reading
- Hardware requirements summary (link to detailed hardware docs)
- Citation block (BibTeX) for the research paper
- Links to video demonstrations

The root README must NOT contain:
- Full wiring tables (move to `docs/reference/hardware_wiring.md`)
- Long installation guides (move to `docs/tutorials/`)
- Detailed software architecture explanations (move to `docs/architecture/`)

### 3.2 Documentation Folder (`docs/`)

Follow the **Diátaxis Framework** to organize documentation:

| Category       | Purpose                                  | Location                 |
| -------------- | ---------------------------------------- | ------------------------ |
| **Tutorials**  | Learning-oriented, step-by-step lessons  | `docs/tutorials/`        |
| **How-To Guides** | Goal-oriented, solve specific problems | `docs/guides/`           |
| **Explanation** | Understanding-oriented, design decisions | `docs/architecture/`     |
| **Reference**  | Information-oriented, specs and APIs     | `docs/reference/`        |

### 3.3 Writing Rules for All Documentation

- Use **headers** (`##`, `###`) liberally. No wall of text.
- Every document must start with a one-sentence summary of what it covers.
- Use **tables** for structured data (parameters, wiring, message fields).
- Use **Mermaid diagrams** for architecture, data flow, and state machines.
- Use **code blocks** with correct language tags for all commands and code.
- Use **admonitions** (GitHub alerts: NOTE, TIP, IMPORTANT, WARNING, CAUTION) for critical information.
- Include a "Prerequisites" section at the top of any tutorial or guide.
- Every command shown must be copy-pasteable. No placeholder paths without explanation.
- Link between documents freely. If you mention SLAM in the navigation doc, link to the SLAM tutorial.

### 3.4 Inline Code Documentation

- Every Python file must have a module-level docstring explaining its purpose.
- Every ROS 2 node class must have a docstring listing its subscriptions, publications, parameters, and purpose.
- Every function must have a docstring if it is longer than 5 lines.
- Use type hints in all Python code.
- For C++/Arduino code, use Doxygen-style comments for all public functions.

---

## 4. Workflow Pipeline Integrity

The system is a pipeline. Respect the data flow:

```
LiDAR Scan → SLAM (map + pose) → Camera Frames → YOLO Detection →
Sensor Fusion (project to map) → Semantic Map → NLP Input →
LLM Planner (structured goal) → Nav2 (path planning) → Local Obstacle Avoidance → Robot Motion
```

### Rules:

- Every module must be **independently testable**. You must be able to run SLAM without YOLO, or YOLO without Nav2.
- Modules communicate ONLY through ROS 2 interfaces (topics, services, actions). No shared global state, no direct function imports between modules.
- When building a new module, ALWAYS define its ROS 2 interfaces (message types, topic names) FIRST before writing logic.
- New custom messages go in `src/robot_msgs/`. Never define messages inside individual module packages.
- Every integration between two modules must be documented with a data-flow diagram showing topic names, message types, and expected frequencies.

---

## 5. Testing and Verification

### Before Any Module is Considered "Done":

1. **Unit Test**: Does the module's core logic work with synthetic/mock data?
2. **Node Test**: Does the ROS 2 node start, subscribe, and publish correctly?
3. **Integration Test**: Does it work when connected to its upstream module?
4. **Documentation Test**: Is the module's README complete? Can someone else run it from the README alone?

### Testing Rules:

- Write tests BEFORE or ALONGSIDE code, never as an afterthought.
- Use `pytest` for Python, `gtest` for C++.
- For ROS 2 integration tests, use `launch_testing`.
- Record a rosbag for every major test scenario. Store the command to replay it.
- Every test must be runnable with a single command documented in the module's README.

---

## 6. Git and Version Control

- **Branching**: Use `feature/<module-name>` branches. Never push directly to `main`.
- **Commits**: Use conventional commit messages: `feat(slam):`, `fix(navigation):`, `docs(yolo):`, `test(fusion):`.
- **Pull Requests**: Every PR must reference which module it affects and include a checklist:
  - [ ] Module README updated
  - [ ] Tests added/updated
  - [ ] No hardcoded parameters
  - [ ] Runs standalone
- **.gitignore**: Never commit `build/`, `install/`, `log/`, `node_modules/`, `.env`, `__pycache__/`, `*.pyc`, `.DS_Store`.
- **Large files**: Never commit large model weights, rosbags, or datasets. Use Git LFS or document download instructions.

---

## 7. Configuration and Reproducibility

- All tunable parameters MUST be in YAML config files, never in source code.
- Provide a `docker/` folder with a Dockerfile that builds the complete environment.
- Provide a `.repos` file listing all external ROS 2 dependencies for `vcs import`.
- Pin all Python dependencies in a `requirements.txt` with exact versions.
- Document the exact OS, ROS 2 distro, CUDA version (if applicable), and hardware in `docs/reference/environment.md`.

---

## 8. Research Paper Integration

- Maintain a `docs/research/` folder for all paper-related content.
- Every algorithmic design decision must have a corresponding explanation in `docs/architecture/` that maps back to the paper.
- Include a BibTeX citation in the root README.
- When implementing an algorithm from a paper or external source, cite it in both the code (docstring) and the documentation.
- Keep a living `docs/research/literature_review.md` summarizing related work.

---

## 9. Build-Test-Integrate Cycle

When adding any new module to the project, follow this exact sequence:

1. **Define** — Write the module's README first (what it does, inputs, outputs, interfaces).
2. **Stub** — Create the package skeleton (folder structure, empty launch file, config, package.xml).
3. **Implement** — Write the core logic with inline documentation.
4. **Test Standalone** — Run the module in isolation with mock data or a rosbag.
5. **Integrate** — Connect it to adjacent modules in the pipeline.
6. **Test Integrated** — Verify the full chain works end-to-end.
7. **Document** — Update all affected documentation and the root README.
8. **Review** — Walk through the changes explaining each file.

Never skip steps

---

## 10. Visual Assets and Demonstrations

- Every module should have at least one screenshot or diagram in its README.
- The root README must include video demonstrations (GitHub-hosted `.mp4` or YouTube links).
- Use Mermaid for all architecture and flow diagrams (they render natively on GitHub).
- Store all images in `assets/` with descriptive filenames (e.g., `slam_map_output.png`, not `image1.png`).
- When showing RViz or Foxglove output, annotate screenshots to explain what the viewer is seeing.

---

## 11. Naming Conventions

- **ROS 2 Packages**: `snake_case` (e.g., `lidar_slam`, `vision_detection`). *Note: Existing packages in lidarbot_ws (e.g., lidarbot_bringup, lidarbot_slam) are grandfathered in to prevent breaking hardware paths.*
- **ROS 2 Nodes**: `snake_case` (e.g., `yolo_detector_node`).
- **Topics**: `/<module>/<data>` (e.g., `/vision/detections`, `/slam/map`, `/planner/goal`).
- **Services**: `/<module>/<verb>_<noun>` (e.g., `/semantic_map/query_objects`).
- **Config Files**: `<module>_params.yaml` (e.g., `slam_params.yaml`).
- **Launch Files**: `<module>_launch.py` (e.g., `slam_launch.py`).
- **Documentation Files**: `kebab-case.md` (e.g., `hardware-wiring.md`, `yolo-setup.md`).
- **Git Branches**: `feature/<module>` (e.g., `feature/vision-detection`).
- **Commits**: `type(scope): description` (e.g., `feat(slam): add cartographer config`)