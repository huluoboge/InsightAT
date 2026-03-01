# Camera Intrinsics Automatic Estimation Logic

This document describes the design and implementation of the automatic camera intrinsics estimation feature in InsightAT.

## 1. Overview

The "Auto Estimate" feature allows users to automatically determine camera intrinsic parameters (primarily focal length in pixels) and image resolution based on image EXIF metadata and physical image dimensions. It also supports automatic image grouping if a single image group contains images from multiple camera types or resolutions.

## 2. Architecture (Sub-process ToolBox)

To ensure application stability and keep the core database lightweight, the estimation logic is implemented as a standalone executable: `CameraEstimator`.

- **Main Application**: `InsightAT_New` (Qt-based)
- **Algorithm Component**: `CameraEstimator` (Standalone CLI)
- **IPC Mechanism**: Standard Input/Output with JSON formatting.
- **Workflow**:
    1. UI collects image paths and system configuration.
    2. UI launches `CameraEstimator` as a sub-process via `QProcess`.
    3. UI writes an `estimator_input` JSON to the process's `stdin`.
    4. `CameraEstimator` performs analysis and writes `estimator_output` to `stdout`.
    5. UI parses the result and applies it to the project database.

## 3. Core Algorithm

### 3.1 Data Sources
The estimator combines data from multiple sources:
- **GDAL**: Used to reliably read image `Width` and `Height` in pixels, bypassing potentially incorrect EXIF resolution tags.
- **EasyExif**: Used to extract `Make`, `Model`, `FocalLength` (physical mm), and `FocalLengthIn35mmFilm` from metadata.
- **Sensor Database**: A local file (`camera_sensor_database.txt`) used to look up the physical sensor width (mm) based on camera `Make` and `Model`.

### 3.2 Mathematical Models

The primary goal is to calculate the focal length in pixels ($f_{px}$).

#### Case A: Using 35mm Equivalent Focal Length
If the EXIF contains the 35mm equivalent focal length ($f_{35mm}$), we use the **Diagonal FOV** estimation method:
$$
f_{px} = f_{35mm} \cdot \frac{\sqrt{W^2 + H^2}}{43.2666}
$$
*Where:*
- $W, H$: Image dimensions in pixels.
- $43.2666$: The diagonal length of a standard 35mm full-frame sensor ($36mm \times 24mm$).

#### Case B: Using Physical Focal Length & Sensor Database
If $f_{35mm}$ is missing but we have the physical focal length ($f_{mm}$) and the sensor width ($S_{width}$) from the database:
$$
f_{px} = \frac{f_{mm} \cdot W}{S_{width}}
$$

#### Case C: Fallback
If neither of the above can be determined, the algorithm assumes a standard wide-angle profile (35mm equivalent) as a safe starting point for bundle adjustment.

## 4. Grouping & Splitting Logic

The estimator automatically identifies if images within the request belong to different groups based on a composite key:
`GroupKey = {Make, Model, Width, Height}`

- **Consolidation**: If all images match a single key, parameters are updated for the existing group.
- **Splitting**: If multiple keys are detected, the UI prompts the user to split the current group into multiple sub-groups. Each sub-group is assigned its specific estimated camera parameters.

## 5. IPC Interface (JSON Specification)

### Input (`estimator_input`)
```json
{
  "estimator_input": {
    "image_paths": ["/path/to/img1.jpg", "/path/to/img2.jpg"],
    "sensor_db_path": "/path/to/camera_sensor_database.txt",
    "log_dir": "/path/to/logs"
  }
}
```

### Output (`estimator_output`)
```json
{
  "estimator_output": {
    "groups": [
      {
        "camera": {
          "make": "DJI",
          "model": "FC6310",
          "width": 5472,
          "height": 3648,
          "sensor_width_mm": 13.2,
          "focal_length_px": 3666.6,
          "focal_length_35mm": 24.0
        },
        "image_indices": [0, 1, 2, 3]
      }
    ]
  }
}
```

## 6. Usage and Configuration

- **Sensor Database**: Located in the application `config/` directory. New camera models can be added manually to improve estimation accuracy.
- **Loggin**: The algorithm logs detailed extraction steps to `logs/CameraEstimator.INFO` for debugging metadata issues.
- **Timeout**: The UI enforces a 10-minute timeout for large batches, though most estimations take milliseconds per image.
