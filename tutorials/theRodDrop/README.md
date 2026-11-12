# `theRodDrop` Tutorial

## 1. Overview
`theRodDrop` verifies 1D body-fixed accelerating reference frame momentum coupling in [`freeBodyPimpleFoam`](../../src/solvers/freeBodyPimpleFoam/freeBodyPimpleFoam.C) for a 2D planar half-symmetry circular cylinder ($R = 2.0\text{ m}$, unit span $w = 1.0\text{ m}$) falling vertically under gravity without dynamic mesh motion (`staticFvMesh`).

Two variants are provided:
- [`theRodDrop/`](theRodDrop): Base case in air ($\rho_f = 1.204\text{ kg/m}^3$, laminar). Also used for the short CI smoke test.
- [`theRodDrop-Denser/`](theRodDrop-Denser): Dense-fluid URANS showcase ($\rho_f = 300.0\text{ kg/m}^3$, $k\text{--}\omega$ SST, $Re = 2.57 \times 10^7$), reaching terminal equilibrium at $V_{\text{sim}} = 96.34\text{ m/s}$ vs. analytical $96.36\text{ m/s}$ ($-0.02\%$, $F_{\text{aero}}/mg = 99.97\%$).

## 2. Physical & Domain Parameters
- **Geometry**: 2D half-cylinder ($R = 2.0\text{ m}$, $w = 1.0\text{ m}$, symmetry plane at $X = 0$, frontal half-area $A = 2.0\text{ m}^2$, half-volume $V = 6.283\text{ m}^3$).
- **Mesh**: 32,000-cell structured `blockMesh` grid spanning $[-10, 0] \times [-12, 22]\text{ m}$.
- **Effective Mass**: $m = 98,018\text{ kg}$ ($\rho_{\mathrm{eff}} \approx 15,600\text{ kg/m}^3$), balanced against half-cylinder surface forces (`patches (sphere)`).
- **Kinematics**: Vertical translation only (`freedom (0 1 0);`), `includeBuoyancy true;`, `g = (0 -9.81 0) m/s^2`.
- **Fluid**: $\nu = 1.5 \times 10^{-5}\text{ m}^2/\text{s}$; $\rho_f = 1.204\text{ kg/m}^3$ in `theRodDrop`, $\rho_f = 300.0\text{ kg/m}^3$ in `theRodDrop-Denser`.

## 3. Running the Simulation

To run the dense-fluid terminal-velocity showcase:
```bash
cd tutorials/theRodDrop/theRodDrop-Denser
./Allclean
./Allrun
python3 plot_motion.py
```

To run the base air variant:
```bash
cd tutorials/theRodDrop/theRodDrop
./Allclean
./Allrun
python3 plot_motion.py
```

`plot_motion.py` parses `postProcessing/fallingMotion/motion.dat` and writes `motion_tracking.png` with descent velocity, altitude trajectory, acceleration / drag-to-weight equilibrium ratio, and force components.
