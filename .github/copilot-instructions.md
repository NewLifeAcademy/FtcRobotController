# Copilot Instructions – FTC DECODE Field Layout (2025–2026)

This document defines the **FTC DECODE field layout** in terms of program coordinates and field element positions.  
It is intended for use by autonomous robot code to reference zones, artifacts, and field elements.

---

## 📐 Coordinate System Mapping

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

## 🎯 Tape Zones

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

## 🏗️ Field Elements

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

## 🪙 Scoring Elements (Artifacts)

- **Alliance Areas:**
  - Each alliance starts with 6 artifacts (4P, 2G) in tray.
- **Loading Zones:**
  - Each contains 3 artifacts in **PGP** configuration, against perimeter wall.
- **Spike Marks:**
  - Six sets of 3 artifacts staged in motifs (e.g., GPP).
  - Middle artifact centered on seam; outer artifacts contact the middle.

---

## 🟥 Alliance Areas

- **Red Alliance Area:** Right side of field (audience perspective).
- **Blue Alliance Area:** Left side of field (audience perspective).
- Each area: 54 in. wide × 96 in. long, marked with tape adjacent to perimeter.

---

## ✅ Usage Notes

- All coordinates are approximate translations from TILE seams into program coordinates.
- Use (X,Y) positions for autonomous navigation and artifact placement logic.
- Headings are defined relative to audience perspective:
  - 0° → facing audience side (tab-line 1).
  - 90° → facing right side (tab-line Z).

