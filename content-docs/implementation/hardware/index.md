# Hardware & Suppliers

This section contains manufacturer information about RoboCon hardware components and suppliers.

## Overview

RoboCon robots are built using high-quality components from trusted manufacturers. This section provides detailed information about hardware suppliers, component specifications, and manufacturer documentation.

## Asset Folder Structure

This section documents the folder structure and naming conventions used for hardware assets. This information is used for enforcing folder structure, naming conventions, and for future directory reading operations.

### Asset Hierarchy

Assets are organized in a hierarchical structure:

1. **Asset Site Folder** - The root directory for a collection of selected asset classes
   - Example: `C:\Users\user01\robocon\project_Robocon_Arm - Documents`

2. **Asset Class Folder** - Contains asset folders of a specific class
   - Example: `Arm` (contains all arm-related assets)
   - Asset class folders may also exist at the root level

3. **Asset Folder** - Contains information and deliverables for a specific asset
   - Naming format: `{{AssetClass}}_{{AssetName}}` (when in Asset Site Folder)
   - Naming format: `{{AssetName}}` (when already in Asset Class Folder)
   - Example in Asset Site Folder: `Arm_Robocon Industrial 30kg`
   - Example in Asset Class Folder: `Robocon Industrial 30kg` (when inside `Arm/` folder)
   - The part before the underscore (`_`) is the **Asset Class** (only needed when in Asset Site Folder)
   - The part after the underscore is the **Asset Name**

4. **Asset Deliverable Folder** - Contains contractor deliverables for a specific version
   - Naming format: `{{YYYY-MM-DD}}_{{contractor username}}` or `{{YYYY-MM-DD}}_{{contractor username}}_{{note}}`
   - Example: `2025-03-24_zaaimvalley`
   - Example with note: `2024-11-21_zaaimvalley_Export`
   - Date format: `YYYY-MM-DD` (year-month-day)
   - Contractor username: The contractor's identifier
   - Note (optional): Third component such as "Export" or "Blender"

### Asset Folder Contents

Each **Asset Folder** contains:

- **`/Docs`** - Documentation files (PDF, DOC, DOCX) containing asset information, specifications, and technical details
- **`/Tasks`** - Task specifications for contractors, including:
  - Asset name
  - CAD Solidworks version and year
  - Whether it's a Solidworks assembly or part
  - 3D file information (Blender .blend, .fbx, etc.)
  - Color information
  - Asset description and function
  - Specifications
  - Labels and naming schemes
  - Folder organization guidelines
  - Budget and timeline
  - SharePoint folder link (public) for contractor uploads/downloads

### Asset Deliverable Folder Structure

Each **Asset Deliverable Folder** contains:

- **`/Drawings`** - Technical drawings
- **`/Renderings`** - Visual renderings of the asset
- **`/Solidworks`** - Solidworks CAD files
- **`/STEP`** - STEP format CAD files
- **`/STL`** - STL format 3D files
- **`/3D`** - Other 3D file formats (Blender .blend, .fbx, etc.)
- **`/Docs`** - Task specifications (same as in parent Asset Folder)

### Asset Types

Assets can be:
- **Parts** - Individual components
- **Components** - Sub-assemblies
- **Full Assemblies** - Complete systems

### Example Structure

```
project_Robocon_Arm - Documents/
├── Arm/                                    (Asset Class Folder)
│   ├── Robocon Industrial 30kg/           (Asset Folder - no class prefix needed)
│   │   ├── Docs/                          (Documentation)
│   │   │   ├── Technical_Spec.pdf
│   │   │   └── Manual.docx
│   │   ├── Tasks/                         (Task Specifications)
│   │   │   └── Task_Spec.docx
│   │   └── 2025-03-24_zaaimvalley/        (Asset Deliverable Folder)
│   │       ├── Drawings/
│   │       ├── Renderings/
│   │       ├── Solidworks/
│   │       ├── STEP/
│   │       ├── STL/
│   │       ├── 3D/
│   │       └── Docs/
│   └── BORUNTE BRTIRXZ1515A/              (Asset Folder - no class prefix needed)
│       └── Docs/
├── Gearbox_J1 RV-200C-142/                (Asset Folder at root - class prefix needed)
│   └── Docs/
└── Motor_EMDA100H2JD0 380V 1000W/         (Asset Folder at root - class prefix needed)
    └── Docs/
```

### Asset Class Examples

| Asset Class Name | Asset Class Short Name | Description |
|------------------|------------------------|-------------|
| Arm | Arm | Robotic arm components, assemblies, and full systems |
| Gearbox | Gearbox | Gearbox components and reduction units |
| Motor | Motor | Motor components and drive systems |

### Parsing Folder Names

When processing asset folders:

1. **Asset Class**: 
   - If folder is inside an Asset Class Folder: Use the parent folder name as Asset Class
   - If folder is in Asset Site Folder: Extract the part before the first underscore (`_`)
   - Examples:
     - `Arm/Robocon Industrial 30kg/` → Asset Class: `Arm` (from parent folder)
     - `Arm_Robocon Industrial 30kg/` (at root) → Asset Class: `Arm` (from folder name)
     - `Gearbox_J1 RV-200C-142/` → Asset Class: `Gearbox`
     - `Motor_EMDA100H2JD0 380V 1000W/` → Asset Class: `Motor`

2. **Asset Name**: 
   - If folder is inside an Asset Class Folder: Use the folder name as Asset Name
   - If folder is in Asset Site Folder: Extract the part after the first underscore
   - Examples:
     - `Arm/Robocon Industrial 30kg/` → Asset Name: `Robocon Industrial 30kg`
     - `Arm_Robocon Industrial 30kg/` (at root) → Asset Name: `Robocon Industrial 30kg`
     - `Gearbox_J1 RV-200C-142/` → Asset Name: `J1 RV-200C-142`

3. **Deliverable Folder**: Parse date, contractor, and optional note
   - `2025-03-24_zaaimvalley` → Date: `2025-03-24`, Contractor: `zaaimvalley`, Note: (none)
   - `2024-11-21_zaaimvalley_Export` → Date: `2024-11-21`, Contractor: `zaaimvalley`, Note: `Export`
   - `2024-12-12_aqeelsheikh677_Blender` → Date: `2024-12-12`, Contractor: `aqeelsheikh677`, Note: `Blender`

### Documentation Location

- Asset documentation is located in: `{{AssetFolder}}/Docs/`
- Task specifications are located in: `{{AssetFolder}}/Tasks/`
- Deliverable documentation is located in: `{{AssetDeliverableFolder}}/Docs/`

## Hardware Categories

### Robotic Arms

- **Dual-Arm Systems**: Information about dual-arm configurations and suppliers
- **Arm Specifications**: Technical details and manufacturer information
- **Arm Assets**: Individual arm components, assemblies, and full systems

### Base Systems

- **Tracked Bases**: Tracked mobility system components
- **Wheeled Bases**: Wheeled mobility system components

### Motors & Actuators

- **Motor Drivers**: CAN bus and serial motor controllers
- **Linear Actuators**: Hydraulic and electric actuation systems
- **Motors**: Individual motor specifications and documentation
- **Gearboxes**: Gearbox components and specifications

### Sensors

- **LiDAR Sensors**: 2D and 3D LiDAR systems
- **IMU Sensors**: Inertial measurement units
- **Pressure Sensors**: Pressure monitoring systems
- **Vision Systems**: Camera and depth sensing

### Power Systems

- **Battery Systems**: Power management and charging
- **Power Distribution**: Electrical systems and power supplies

## Supplier Information

Detailed supplier and manufacturer information is organized by component type. This information is for internal use and contractor reference only.

---

**Note**: This section contains proprietary supplier information. Access is restricted to RoboCon contractors and employees.

