# Copilot Instructions
This document describes instructions for building robot code for use with the FTC DECODE 2025 season game.
It includes field layout and coordinate system instructions for autonomous route programming.
It includes details on how April Tag data can be used to identify robot positioning on the field.

## FTC DECODE Field Layout Overview
This section defines the **FTC DECODE field layout** in terms of program coordinates and field element positions.  
It is intended for use by autonomous robot code to reference zones, artifacts, and field elements.

---

### 📐 Coordinate System Mapping

- **Origin (X=0, Y=0):** TILE tab-line location **X3** (center of field).
- **Positive Quadrants:**
  - (X=24, Y=24) → TILE tab-line **Y2**
  - (X=24, Y=-24) → TILE tab-line **W2**
- **Negative Quadrants:**
  - (X=-24, Y=-24) → TILE tab-line **W4**
  - (X=-24, Y=24) → TILE tab-line **Y4**
- **Robot Heading Reference:**
  - At (0,0), heading **0°** → facing TILE tab-line **1** (audience side).
  - At (0,0), heading **90°** → facing TILE tab-line **Z** (right side from audience perspective).

---

### 🎯 Tape Zones

- **Launch Lines (White):**
  - **Back Launch Line:** “V” shape from back corners (approx. -24,24 and 24,24) to center (0,0).
  - **Front Launch Line:** Inverted “V” spanning audience edge at (approx. -8,-24 to 8,-24).
- **Base Zones:**
  - Red Base Zone: near (X≈-8, Y≈-8).
  - Blue Base Zone: near (X≈8, Y≈-8).
- **Spike Marks (White):**
  - Six marks aligned along seams V and Z:
    - Left side: (X≈-20, Y≈-4), (X≈-20, Y≈-8), (X≈-20, Y≈-12).
    - Right side: (X≈20, Y≈-4), (X≈20, Y≈-8), (X≈20, Y≈-12).
- **Gate Zones:**
  - Blue Gate Zone: left side (X≈-24, Y≈-8 to -4).
  - Red Gate Zone: right side (X≈24, Y≈-8 to -4).
- **Loading Zones:**
  - Corners at audience side: (X≈-24, Y≈-24) and (X≈24, Y≈-24).
- **Secret Tunnel Zones:**
  - Red: left side spanning (X≈-24, Y≈-12 to -8).
  - Blue: right side spanning (X≈24, Y≈-12 to -8).
- **Depots:**
  - Directly in front of each GOAL, aligned with perimeter wall.

---

### 🏗️ Field Elements

- **Goals & Ramps:**
  - One GOAL + Upper Ramp per alliance (Red on right, Blue on left).
  - Lower Ramp assemblies connect to each GOAL.
- **Blocker Panels:**
  - Attached to each GOAL (Red and Blue).
- **Obelisk:**
  - Positioned opposite audience side, centered at (X=0, Y=24).
- **Artifact Trays:**
  - Two trays, one per alliance, adjacent to perimeter near alliance areas.

---

### 🪙 Scoring Elements (Artifacts)

- **Alliance Areas:**
  - Each alliance starts with 6 artifacts (4P, 2G) in tray.
- **Loading Zones:**
  - Each contains 3 artifacts in **PGP** configuration, against perimeter wall.
- **Spike Marks:**
  - Six sets of 3 artifacts staged in motifs (e.g., GPP).
  - Middle artifact centered on seam; outer artifacts contact the middle.

---

### 🟥 Alliance Areas

- **Red Alliance Area:** Right side of field (audience perspective).
- **Blue Alliance Area:** Left side of field (audience perspective).
- Each area: 54 in. wide × 96 in. long, marked with tape adjacent to perimeter.

---

### ✅ Usage Notes

- All coordinates are approximate translations from TILE seams into program coordinates.
- Use (X,Y) positions for autonomous navigation and artifact placement logic.
- Headings are defined relative to audience perspective:
  - 0° → facing audience side (tab-line 1).
  - 90° → facing right side (tab-line Z).

## 📷 AprilTag Data Usage Guide for FTC Robots

This section provides instructions for assisting with code that uses AprilTag detection data from the *FIRST* Tech Challenge (FTC) SDK. The goal is to use the `ftcPose` data (including translation, rotation, and derived values) to control robot movement for common tasks like aiming, driving, and aligning with AprilTags.

## 1. Understanding AprilTag `ftcPose` Data

When an AprilTag is detected, the SDK provides an `ftcPose` object containing the tag's position and orientation relative to the camera.

### Coordinate System (Camera-Relative)

*   **Y-axis (forward)**: Represents the forward distance from the camera to the tag. This is the primary distance measurement.
*   **X-axis (strafe)**: Represents the sideways offset of the tag from the camera's centerline.
    *   **Positive X**: The tag is to the **right** of the camera's center.
    *   **Negative X**: The tag is to the **left** of the camera's center.
*   **Z-axis (elevation)**: Represents the vertical offset of the tag from the camera's centerline.
    *   **Positive Z**: The tag is **above** the camera.
    *   **Negative Z**: The tag is **below** the camera.

*Note: The default unit for these distances is **inches**.*

### Rotation Values

*   **Yaw**: Rotation of the tag around the Z-axis (vertical axis). This indicates if the robot is squarely in front of the tag.
    *   **Positive Yaw**: The tag is rotated **counter-clockwise** relative to the camera's view.
    *   A **Yaw of 0 degrees** means the robot is perfectly "square" with the face of the tag.
*   **Pitch**: Rotation of the tag around the X-axis (horizontal side-to-side axis).
*   **Roll**: Rotation of the tag around the Y-axis (forward-backward axis).

*Note: The default unit for these angles is **degrees**.*

### Derived Values

These values are calculated from the core X, Y, and Z data to simplify navigation logic.

*   **Range**: The direct, straight-line distance from the camera to the center of the tag.
*   **Bearing**: The angle (in degrees) the robot needs to **turn** to point its camera directly at the tag.
    *   **Positive Bearing**: Turn **counter-clockwise** (left).
    *   **Negative Bearing**: Turn **clockwise** (right).
*   **Elevation**: The angle (in degrees) the robot's camera needs to tilt **up or down** to point directly at the tag.

## 2. Common Coding Patterns for Robot Control

Based on the robot's drivetrain, different values from the `ftcPose` are prioritized.

### Task 1: Pointing and Driving Towards a Target (Tank Drive)

This approach is for robots that cannot move sideways (strafe). The goal is to turn to face the tag and drive to a desired range. This method does **not** guarantee the robot will be "square" with the tag.

**Control Logic:**

1.  **Turning**: Use the `Bearing` value as the primary input for turning control. A proportional controller is often sufficient.
    *   `turn_power = Kp_turn * aprilTagDetection.ftcPose.bearing`
    *   The goal is to drive the `Bearing` to 0.
2.  **Forward/Backward Driving**: Use the `Range` value to control the robot's distance from the tag.
    *   `forward_power = Kp_drive * (desired_range - aprilTagDetection.ftcPose.range)`
    *   The goal is to reach a `desired_range`.

*Reference Sample OpMode: `RobotAutoDriveToAprilTagTank.java`*

### Task 2: Approaching a Target Squarely (Omni-Directional/Holonomic Drive)

This approach is for robots that can move in any direction (forward, turn, and strafe). The goal is to arrive at a specific distance directly in front of the tag, perfectly aligned.

**Control Logic (Three-Pronged PID Approach):**

This uses three independent control loops for each axis of motion, with turning as the highest priority.

1.  **Turn Control (Highest Priority)**:
    *   **Goal**: Make the robot's camera point directly at the tag.
    *   **Input**: `aprilTagDetection.ftcPose.bearing`
    *   **Output**: Rotational power for the drivetrain.
    *   **Objective**: Drive `bearing` to `0`.

2.  **Strafe Control (Sideways Motion)**:
    *   **Goal**: Move the robot sideways until it is directly in front of the tag (squarely aligned).
    *   **Input**: `aprilTagDetection.ftcPose.yaw`
    *   **Output**: Sideways (strafe) power for the drivetrain.
    *   **Objective**: Drive `yaw` to `0`.

3.  **Forward/Backward Control**:
    *   **Goal**: Move the robot to the desired standoff distance from the tag.
    *   **Input**: `aprilTagDetection.ftcPose.range`
    *   **Output**: Forward/backward power for the drivetrain.
    *   **Objective**: Drive `range` to `desired_range`.

*Reference Sample OpMode: `RobotAutoDriveToAprilTagOmni.java`*
