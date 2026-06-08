# ARES Main

ARES, or Autonomous Rover Exploration System, is a ROS 2 Humble-based autonomous rover system for multi-rover exploration and target searching. This repository contains the main ROS 2 packages for LIDAR mapping, searching and path planning, guidance coordination, path tracking, target recognition, and VRPN-based localization.

## Repository Overview

This repository contains the ROS 2 packages used by the ARES rover system, including:
 
* RPLIDAR ROS 2 driver
* LIDAR mapping using the RPLIDAR A2M12
* Searching and planning using frontier-based exploration and RRT path planning
* Guidance-level coordination for planning, execution, stopping, and target approach behavior
* Path tracking and STM32 command interface
* AprilTag-based target recognition
* VRPN-based rover localization

## Hardware Overview

The guidance-side rover hardware includes:

* NVIDIA Jetson Orin Nano
* RPLIDAR A2M12
* STM32H723ZG
* 3 cameras

## Hardware Setup

Follow this startup order when running the system on the real rover:

1. Make sure the Jetson Orin Nano is **not connected** to the STM32H723ZG before powering on the Jetson. The Jetson may not turn on correctly if the STM32 is already connected.
2. Make sure the cameras are **not connected** to the Jetson during startup. The camera-port assignment is not currently handled automatically, so the system may not correctly identify the left, middle, and right cameras if they are connected in the wrong order.
3. Power on the Jetson Orin Nano.
4. Connect to the Jetson through SSH, or use a monitor and keyboard if they are available.
5. Connect the STM32H723ZG.
6. Connect the cameras in the required order:

   1. Left camera
   2. Middle camera
   3. Right camera

This order helps ensure that the cameras are assigned to the expected ports.

## Installation and Setup

### Prerequisites

* **OS:** Ubuntu 22.04
  Other ROS 2 Humble installation options can be found in the official ROS 2 documentation.

### 1. Install ROS 2 Humble

Follow the official ROS 2 Humble installation guide for Ubuntu 22.04:

```text
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
```

Install the **Desktop** version, not the Base version. The development tools are needed to build this project.

### 2. Install Dependencies

```bash
sudo apt-get update
sudo apt-get install ros-humble-vrpn
sudo apt-get install ros-humble-cartographer-ros
sudo apt-get install python3-colcon-common-extensions
```

### 3. Build the Project

From the root of the ROS 2 workspace, run:

```bash
source /opt/ros/humble/setup.bash
colcon build
```

### 4. Source the Project

After building, source the workspace:

```bash
source install/setup.bash
```

To avoid sourcing the workspace manually every time, you can add the following line to your `~/.bashrc`:

```bash
source ~/ares_main/install/setup.bash
```

Change `~/ares_main` to the actual path of your workspace if needed.

### 5. Launch the Rover System

Launch the full rover system using:

```bash
ros2 launch coordinator coordinator.launch.py
```

## Notes

* This repository is intended for ROS 2 Humble on Ubuntu 22.04.
* The real rover setup depends on correct hardware connection order, especially for the STM32 and cameras.
* If the target recognition node is not running or no target is detected, the rover will continue autonomous exploration.
* This repository was developed as part of the ARES multi-rover autonomous exploration system.

---
## Intro to Github

### Generate SSH Key

#### For Mac/Linux:
```bash
# Generate a new SSH key
ssh-keygen -t ed25519 -C "your_email@example.com"

# Start the ssh-agent
eval "$(ssh-agent -s)"

# Add your SSH private key to the ssh-agent
ssh-add ~/.ssh/id_ed25519

# Copy the public key to clipboard (Mac)
pbcopy < ~/.ssh/id_ed25519.pub

# Or display the public key to copy manually
cat ~/.ssh/id_ed25519.pub
```

#### For Windows:
```bash
# Generate a new SSH key using Git Bash or PowerShell
ssh-keygen -t ed25519 -C "your_email@example.com"

# Start the ssh-agent
eval "$(ssh-agent -s)"

# Add your SSH private key to the ssh-agent
ssh-add ~/.ssh/id_ed25519

# Copy the public key to clipboard (Windows)
clip < ~/.ssh/id_ed25519.pub

# Or display the public key to copy manually
cat ~/.ssh/id_ed25519.pub
```

**Next Steps:**
1. Copy the public key output
2. Go to GitHub → Settings → SSH and GPG keys
3. Click "New SSH key"
4. Paste your public key and give it a descriptive title
5. Click "Add SSH key"

### Clone the Repository

```bash
git clone https://github.com/<your-org>/<your-repo>.git
cd <your-repo>
```

### Create a New Branch
[Please reference this for info on how to config branches](docs/Branching_README.md)
_(I worked really hard on that ReadMe pls read it pretty pls I beg you)_

**Create a new branch:**
```bash
git checkout -b <system>-<component>-<feature>-branch
```
_to get into a branch that already exist forgo the -b..._

**Switch to an existing branch:**
```bash
git checkout <system>-<component>-<feature>-branch
```

### Make Changes and Commit

**Add all changes:**
```bash
git add .
git commit -m "Message for Commit"
```

**Add specific files:**
```bash
git add script.py otherFile.cpp notes.txt
git commit -m "Message for Commit"
```
### Push to Github
```bash
git push origin <your-branch-name>
```

### Pull from Github
```bash
git checkout <branch-or-main>
git pull origin <branch-or-main>
```

### Merge Branch to Main (Local)
**Note: This is for local merging only. Use Pull Requests for GitHub merging.**

```bash
git checkout main
git pull origin main
git merge <branch-name>
```

_Resolve any conflicts and test your code with the merged branch_

---
## Developer's Guide to GitHub

### Best Practices for Collaborative Development

#### Pull Often
- **Always pull before starting work**: `git pull origin <branch or main>` before creating new branches
- **Stay up to date**: Pull from main regularly to avoid merge conflicts
- **Pull before pushing**: Ensure your branch is current with the latest changes

#### Pull Requests (PR) for Main Branch
- **Always use PRs**: Never push directly to main branch
- **Create PRs for all changes**: Even small fixes should go through PR process
- **Write clear PR descriptions**: Explain what changes were made and why
- **Request reviews**: Ask team members to review your code
- **Address feedback**: Respond to review comments and make necessary changes
- **Test before merging**: Ensure all tests pass and code works as expected

---
