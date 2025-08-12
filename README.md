# METR4202 Team Project – Exploration & Target Search

## Overview
This repository contains our codebase for the **METR4202 Robotics & Automation** team project (Semester 2, 2025).  
Our goal is to develop an autonomous exploration and target search system for the TurtleBot3 Waffle Pi, capable of:
- Building a map of an unknown environment (SLAM).
- Efficiently exploring and covering the map.
- Detecting and localising ArUco markers with high accuracy.
- Running both in **Gazebo simulation** and on **real hardware**.

The system integrates:
- ROS 2 Humble Hawksbill
- Gazebo Classic
- SLAM Toolbox
- Navigation 2 (Nav2) stack
- OpenCV ArUco detection
- Custom ROS 2 Python nodes for exploration, perception, coordination, and monitoring

---

## Git Basics – Common Commands

| Action | Command | Notes |
|--------|---------|-------|
| **Clone the repository** | `git clone <repo_url>` | First-time setup on your machine. |
| **Check repository status** | `git status` | Shows modified/untracked files and branch. |
| **Create and switch to a new branch** | `git checkout -b <branch_name>` | Use a descriptive branch name for your feature/task. |
| **Switch to an existing branch** | `git checkout <branch_name>` | Move between branches. |
| **Pull latest changes** | `git pull` | Always run before starting new work. |
| **Stage changes** | `git add <file>` | Stage specific file(s). Use `git add .` for all changes. |
| **Commit staged changes** | `git commit -m "Short description of change"` | Write a clear, concise commit message. |
| **Push changes to remote branch** | `git push origin <branch_name>` | Upload commits to GitHub. |
| **Merge your branch into main** | 1. `git checkout main` <br> 2. `git pull` <br> 3. `git merge <branch_name>` <br> 4. `git push origin main` | Or open a Pull Request on GitHub for review. |

---

### Recommended Workflow
1. **Pull latest changes** before you start (`git pull` on your branch or main).
2. **Create a feature branch** for each new task or bugfix.
3. **Commit and push** regularly — small commits are easier to review.
4. **Test before pushing** to main to avoid breaking builds.
5. **Use Pull Requests** for major changes so they can be reviewed.

---
