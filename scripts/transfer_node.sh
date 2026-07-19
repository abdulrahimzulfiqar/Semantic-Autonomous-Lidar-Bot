#!/bin/bash

# ==============================================================================
# SMART WHEELCHAIR FILE TRANSFER UTILITY
# ==============================================================================
# How does it work?
# This script copies files or folders from your local Mac computer directly to 
# the Raspberry Pi over your local network using SSH-based copy utilities.
# ==============================================================================

# Variables (Update these with your Pi's connection details)
PI_USER="pi-project" #Enter your pi user name
PI_IP="IP_ADDRESS" #Enter your pi ip address
WORKSPACE_DIR="~/lidarbot_ws" #Enter your pi workspace directory

# ------------------------------------------------------------------------------
# TEMPLATE 1: Copy a Single File (using scp)
# Format: scp /path/to/local/file user@IP:/path/to/remote/destination
# ------------------------------------------------------------------------------
# Example: Copying the IMU fall detector script:
scp /Users/teamincredibles/Desktop/Setups/Downloads/Wheelchair/lidarbot_ws/src/lidarbot/lidarbot_bringup/script/imu_fall_detector.py ${PI_USER}@${PI_IP}:${WORKSPACE_DIR}/src/lidarbot/lidarbot_bringup/script/

# ------------------------------------------------------------------------------
# TEMPLATE 2: Copy an Entire Directory (using scp -r)
# Format: scp -r /path/to/local/folder user@IP:/path/to/remote/destination
# ------------------------------------------------------------------------------
# # Example (commented out):
# scp -r /Users/teamincredibles/Desktop/Setups/Downloads/Wheelchair/lidarbot_ws/src/lidarbot/lidarbot_navigation/ ${PI_USER}@${PI_IP}:${WORKSPACE_DIR}/src/lidarbot/

# ------------------------------------------------------------------------------
# TEMPLATE 3: Efficiently Sync Workspace Directories (using rsync)
# Rsync only copies files that have changed (much faster) and ignores heavy folders.
# Format: rsync -avz --exclude 'exclude_name' /local/path/ user@IP:/remote/path/
# ------------------------------------------------------------------------------
# # Example (commented out):
# rsync -avz --exclude 'build/' --exclude 'install/' --exclude 'log/' --exclude '.git/' \
#   /Users/teamincredibles/Desktop/Setups/Downloads/Wheelchair/lidarbot_ws/ \
#   ${PI_USER}@${PI_IP}:${WORKSPACE_DIR}/

# ==============================================================================
# PULL TEMPLATES (Raspberry Pi ===> Mac Repository)
# Run these commands on your Mac terminal to retrieve files from the Pi.
# ==============================================================================

# ------------------------------------------------------------------------------
# TEMPLATE 4: Pull Benchmark Results Directory (using rsync)
# This pulls all CSV and HTML graphs from the Pi to your Mac.
# ------------------------------------------------------------------------------
# # Example (commented out - uncomment to run):
# rsync -avz ${PI_USER}@${PI_IP}:${WORKSPACE_DIR}/benchmark_results/ \
#   /Users/teamincredibles/Desktop/Setups/Downloads/Wheelchair/benchmark_results/

# ------------------------------------------------------------------------------
# TEMPLATE 5: Pull a Single File (using scp)
# Format: scp user@IP:/path/to/remote/file /local/destination/
# ------------------------------------------------------------------------------
# # Example (commented out):
# scp ${PI_USER}@${PI_IP}:${WORKSPACE_DIR}/benchmark_results/chart_qwen05b.html \
#   /Users/teamincredibles/Desktop/Setups/Downloads/Wheelchair/benchmark_results/


