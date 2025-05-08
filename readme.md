# Physics Engine Project

## Overview
This project is a custom physics engine developed in C++ that simulates rigid body dynamics, focusing on accurate collision detection, resolution, and optimization techniques to support a high number of interacting objects in real time.

The current demo implements a coin pusher simulation, where cubes are used in place of coins. The simulation supports user interactions and dynamically tracks performance as more objects are added.

Note: All detailed documentaions of techincal issues can be found at /Docs

---

## Demo: Coin Pusher Simulation
- Implements a functional coin pusher machine using cube colliders.
- Users can manipulate a bar to push coins.
- Objects ("coins") are spawned dynamically as the game progresses.
- A scoreboard tracks how many coins are successfully pushed off the stage.
- **Highlight:** Real-time simulation of dozens to hundreds of interacting bodies with stable performance.

### 🎬 Demo Video
[Watch on Bilibili](https://www.bilibili.com/video/BV1KwB5YpETv/)

### 🖼️ Preview GIF
![Coin Pusher Demo](Docs/m3/CoinPusher.gif)


---

### Milestone 1: Initial Framework

A foundational physics engine was developed, supporting basic physical interactions between objects. However, several limitations were identified:

- **Collision Detection**: Relied on the Separating Axis Theorem (SAT), which provided only a single collision point and failed to generate accurate collision normal vectors.
- **Collision Resolution**: Coupled tightly with detection logic, making debugging and future extensions difficult.
- **Accuracy Issues**: Lacked precise handling of complex collisions and didn’t correctly simulate speed, position, or friction.
- **Rotation Updates**: Incorrect update method led to unrealistic rotational behaviors.

These limitations resulted in an unrealistic simulation and a flawed initial demo.

---

### Milestone 2: Refactored Architecture with GJK + EPA

Major architectural improvements were made:

- **Decoupled Simulation Stages**: Introduced a new `Tick` function with clear separation between stages to keep object states synchronized.
- **Advanced Collision Detection**: Replaced SAT with **GJK (Gilbert–Johnson–Keerthi)** and **EPA (Expanding Polytope Algorithm)** for more accurate:
  - Contact points
  - Normals
  - Penetration depths
- **Physical Properties**: Objects now support tunable mass, friction, and restitution.
- **Material Simulation**: Enables realistic behavior for various materials.

---

### Milestone 3: Stabilization & Multithreading

#### ✅ Features Added:
- **Baumgarte Stabilization**: Prevents persistent interpenetration between rigid bodies.
- **Jitter Smoothing**: Two slope-based thresholds reduce jitter when both speed and penetration are low.
- **Contact Manifolds**: Currently supports 1 contact point per manifold (targeting 4 for future work).
- **Multithreading**:
  - Physics steps are parallelized using a thread pool.
  - Enables consistent performance even with a large number of colliding objects.

---

## 🔧 Technical Highlights

- **Collision System**: GJK + EPA
- **Resolution**: Impulse-based with Baumgarte stabilization
- **Threading**: Custom thread pool for all physics update stages
- **Tunable Parameters**: Mass, friction, restitution
- **Extensibility**: Modular Tick loop allows easy integration of new features

---

## Future Work
- Support for full contact manifold (up to 4 points).
- Implement angular friction and rolling resistance.
- Improve material interaction models.
- Export simulation data for visualization.

---

## 📹 Known Limitations

- **Jittering**: Due to single contact point per collision manifold
- **Visual Artifacts**: May appear in dense object clusters

These are planned to be addressed in future milestones with better manifold contact handling.

---

## Acknowledgments
Special thanks to the USC CS522 porfessor Artem Kovalovs and TAs for project support and inspiration.
