#!/usr/bin/env python3

import depthai as dai
import numpy as np
import sys
import json
import os
from pathlib import Path

# ==============================================================================
#  USER CONFIGURATION
# ==============================================================================
CAMERA_MAP = {
    "192.168.2.20": "fork_cam",
    "192.168.2.21": "right_cam",
    "192.168.2.22": "left_cam",
    "192.168.2.23": "up_cam",
    "192.168.2.24": "front_cam",
    "192.168.2.25": "front_low_cam",
    "192.168.2.26": "fork_low_cam"
}
# ==============================================================================

def to_list(obj):
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    return obj

# Get list of all connected devices
deviceInfos = dai.Device.getAllAvailableDevices()

if len(deviceInfos) == 0:
    print("No DepthAI devices found!")
    sys.exit(0)

print(f"Found {len(deviceInfos)} device(s). Starting processing...\n")

full_system_data = {}
output_json_file = "system_calibration_dump.json"
temp_file = "temp_calib_dump.json"  # Temporary file for extracting raw data

for i, devInfo in enumerate(deviceInfos):
    ip_address = devInfo.name
    device_identifier = CAMERA_MAP.get(ip_address, ip_address)
    
    print(f"\n{'='*40}")
    print(f"Connecting to device #{i+1}...")
    print(f"IP: {ip_address} --> Name: {device_identifier}")
    print(f"{'='*40}")

    try:
        with dai.Device(devInfo) as device:
            mxid = device.getMxId()
            print(f"Connected successfully! MxID: {mxid}")

            calibData = device.readCalibration()

            # ------------------------------------------------------------
            # 1. Embed Raw EEPROM Data (No separate files left on disk)
            # ------------------------------------------------------------
            # We write to a temp file, read it back into memory, then delete it.
            calibData.eepromToJsonFile(temp_file)
            
            raw_eeprom_data = {}
            if os.path.exists(temp_file):
                with open(temp_file, 'r') as tf:
                    raw_eeprom_data = json.load(tf)
                os.remove(temp_file) # Clean up
            
            # ------------------------------------------------------------
            # 2. Collect Calculated Metrics
            # ------------------------------------------------------------
            camera_metrics = {
                "name": device_identifier,
                "ip_address": ip_address,
                "mxid": mxid,
                "board_name": calibData.getEepromData().boardName,
                "raw_eeprom": raw_eeprom_data  # <--- Embedded here
            }

            # --- RGB Camera ---
            M_rgb, width, height = calibData.getDefaultIntrinsics(dai.CameraBoardSocket.CAM_A)
            camera_metrics["rgb"] = {
                "default_intrinsics": to_list(M_rgb),
                "default_width": width,
                "default_height": height,
                "fov": calibData.getFov(dai.CameraBoardSocket.CAM_A)
            }

            if "OAK-1" in camera_metrics["board_name"] or "BW1093OAK" in camera_metrics["board_name"]:
                M_rgb_720 = np.array(calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, 1280, 720))
                camera_metrics["rgb"]["intrinsics_1280x720"] = to_list(M_rgb_720)
                camera_metrics["rgb"]["distortion_coeff"] = to_list(calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A))
            else:
                camera_metrics["rgb"]["intrinsics_3840x2160"] = to_list(calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, 3840, 2160))
                camera_metrics["rgb"]["intrinsics_4056x3040"] = to_list(calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, 4056, 3040))
                
                # --- Stereo Cameras ---
                camera_metrics["stereo"] = {}
                
                M_left, width_l, height_l = calibData.getDefaultIntrinsics(dai.CameraBoardSocket.CAM_B)
                camera_metrics["stereo"]["left"] = {
                    "default_intrinsics": to_list(M_left),
                    "width": width_l,
                    "height": height_l,
                    "intrinsics_1280x720": to_list(calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, 1280, 720)),
                    "distortion_coeff": to_list(calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_B)),
                    "fov": calibData.getFov(dai.CameraBoardSocket.CAM_B)
                }

                M_right = np.array(calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_C, 1280, 720))
                camera_metrics["stereo"]["right"] = {
                    "intrinsics_1280x720": to_list(M_right),
                    "distortion_coeff": to_list(calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_C))
                }

                R1 = np.array(calibData.getStereoLeftRectificationRotation())
                R2 = np.array(calibData.getStereoRightRectificationRotation())
                right_socket = calibData.getStereoRightCameraId()
                M_right_stereo = np.array(calibData.getCameraIntrinsics(right_socket, 1280, 720))

                H_left = np.matmul(np.matmul(M_right_stereo, R1), np.linalg.inv(M_left))
                H_right = np.matmul(np.matmul(M_right_stereo, R1), np.linalg.inv(M_right_stereo))

                camera_metrics["stereo"]["rectification"] = {
                    "H_left": to_list(H_left),
                    "H_right": to_list(H_right)
                }

                camera_metrics["extrinsics"] = {
                    "left_to_right": to_list(calibData.getCameraExtrinsics(dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C)),
                    "left_to_rgb": to_list(calibData.getCameraExtrinsics(dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_A))
                }

            full_system_data[device_identifier] = camera_metrics
            print(f"   >> Processed data for {device_identifier}")

    except RuntimeError as e:
        print(f"Failed to connect to device #{i+1} (IP: {ip_address})")
        print(f"Error: {e}")

# Save the SINGLE master JSON file
print(f"\n{'='*40}")
print(f"Saving combined system-wide calibration dump...")
try:
    with open(output_json_file, 'w') as f:
        json.dump(full_system_data, f, indent=4)
    print(f"Success! All data saved to: {output_json_file}")
except Exception as e:
    print(f"Error saving JSON file: {e}")
