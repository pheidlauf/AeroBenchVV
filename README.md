# AeroBenchVV
Additional documentation will be added to this document as it is created.

## F16_Model
This directory contains a nonlinear mathematical model of the F-16 aircraft published in _Aircraft Control and Simulation: Dynamics, Controls Design, and Autonomous Systems_. This model uses lookup tables to access the aero data for the model. As well, a polynomial model fit to the lookup table data (developed by Eugene Morelli) is included.

## F16_Sim
This directory contains all of the Matlab scripts and functions used to simulate and analyze autonomous maneuvers of the aforementioned F-16 models.

## flypath3d
"The flypath3d package a free Matlab software for 3D visualizations of missile and air target trajectories that provide a visual reference for the computer simulation. Because of its versatility and ease of use, the software can help to produce attractive presentations for various scientific or public outreach purposes."

This package was modified and tailored for this project to generate visualizations of F-16 autonomous maneuvers with a text overlay of state data and pass/fail conditions.

### Citations
Stevens, Brian L., Frank L. Lewis, and Eric N. Johnson. Aircraft control and simulation: dynamics, controls design, and autonomous systems. John Wiley & Sons, 2015.  
Morelli, Eugene A. "Global nonlinear parametric modelling with application to F-16 aerodynamics." American Control Conference, 1998. Proceedings of the 1998. Vol. 2. IEEE, 1998.
Bużantowicz W. "Matlab Script for 3D Visualization of Missile and Air Target Trajectories".International Journal of Computer and Information Technology 5 (2016)5, pp. 419-422.

### Release Documentation
Distribution A: Approved for Public Release (88ABW-2017-6379)
