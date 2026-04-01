# RenSim

RenSim is a modular FRC physics library for simulation, analysis, and robotics workflow integration.

### Key Capabilities (implementaion ongoing)
- **Unified Physics Engine:**
  - Rigid body, drivetrain, and mechanism simulation
  - Realistic field, collision, and sensor modeling
- **Multi-language Bindings:**
  - Native C++ core
  - Python and Java bindings for rapid prototyping and integration
- **Simulation Runtime:**
  - Plug-and-play robot code simulation
  - Sensor pipelines and WPILib bridge
- **CAD Import:**
  - Load URDF, STL, and more for custom robots and fields
- **Extensible & Modular:**
  - Add your own models, forces, and plugins

### Project Goals
- **Accuracy:** Realistic, physically-based simulation for FRC robots and game elements
- **Integration:** Seamless workflow from CAD to code to simulation
- **Accessibility:** Usable by teams of all experience levels, with Python/Java/C++ support
- **Open Source:** Community-driven, MIT-licensed

---

## Documentation
- [API Usage](api_usage.md): How to use the library in your code
- [Integrators](integrators.md): Numerical integration methods used by the physics engine
- [Vector3](vector.md): 3D vector operations and helpers
- [Quaternion](quaternion.md): 3D orientation and rotation utilities
- [Architecture](architecture.md): System design and extensibility
- [Physics Reference](physics_reference.md): Underlying models and equations

---

## Quick Start
```bash
# C++
# See examples/cpp/minimal_world.cpp

# Python
pip install frcsim-physics
python examples/python/simple_world_demo.py

# Java
# See examples/java/ShooterPredictionExample.java
```

---

## Contributing
We welcome issues, pull requests, and ideas from the FRC and robotics community.


<!--
<div align="center">
  <img src="assets/images/frc_field.png" alt="FRC Field" width="400"/>
</div>
-->

---

<div align="center">
  <b>Simulate. Innovate. Win.</b>
</div>
