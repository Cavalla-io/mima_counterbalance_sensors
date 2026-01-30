# Pull-Wire Encoder Configuration

This directory contains the configuration for the SICK wire-draw encoder (Model BCG13).

## Files

* **`encoder-config.yaml`**: The active configuration file used by the runtime software.
* **`encoder_calibration.py`**: Script to generate this config (located in `../scripts/`).

## Configuration Structure

### `calibration`
* **`resolution_mm_per_tick`**: `0.05` (Fixed factory spec for SICK BCG13).
* **`zero_point_ticks`**: The raw CAN integer value corresponding to 0mm (Ground).
* **`mechanical_max_ticks`**: The raw CAN integer value at the physical top stop.
* **`mechanical_max_height_mm`**: The calculated physical height of the mast.
* **`direction_scalar`**: `1` (Standard) or `-1` (Reversed) depending on mounting.

### `safety`
* **`max_allowed_height_mm`**: User-defined software limit. The system should stop before exceeding this.
* **`min_allowed_height_mm`**: Buffer for negative values (usually -10.0mm).

## Usage

**To Recalibrate:**
Run the interactive calibration script. It will automatically detect direction, zero point, and mechanical height, then prompt for safety limits.

```bash
cd ../scripts
sudo python3 encoder_calibration.py
