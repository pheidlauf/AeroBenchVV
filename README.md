<p align="center"> <img src="gcas.gif"/> </p>

# AeroBenchVV Overview
This project contains a set of benchmark models and controllers that test automated aircraft maneuvers by performing simulations. The hope is to provide a benchmark to motivate better verification and analysis methods, working beyond models based on Dubin's car dynamics, towards the sorts of models used in aerospace engineering. Compared to actual aerospace models used in practice, however, these models are still very simple. Roughly speaking, the dynamics are nonlinear, have about 10-20 dimensions (continuous state variables), and hybrid in the sense of discontinuous ODEs, but not with jumps in the state. The dynamics are given entirely in human-readable matlab .m code, without the need for additional Matlab toolboxes. Ode45 is used to simulate the system, pass/fail specifications are checked against the simulated maneuver, and a plot (or animation) of the resultant flight path can be generated.

To run the ground collision avoidance system (GCAS) benchmark and create the output_pic.png image, output_animation.gif animation and SimResults.mat data file (in the F16_Sim folder), use Matlab to set the F16_Sim folder as your working directory and run the Main.m script. Note creating the .gif animation takes the longest time, so you might want to eventually disable that by commenting out "MakeAnimation;" at the bottom of the script.

You can modify the initial states or model to see how it affects the aircraft maneuver. There are two aircraft dynamics models available based on the well-studied F-16 aircraft, a lookup-table version (from Stevens & Lewis [1]), and a polynomial version (from Morelli [2]). You can choose the model to simulate in Main.m. There is also a partial linear version of the plant model, which is likely less accurate. See the code starting at line 116 on controlledF16.m for how this gets selected.

The low-level controller is a decoupled LQR controller, with the gains given around line 181 of RunF16Sim.m. The controller was designed by linearizing the plant around a single setpoint. More complicated, gain-scheduled versions could fairly easily be added with this setup, but that is not currently done.

You can modify the models or controllers, and see how the maneuver is affected. For example, if you change the center of gravity x position, xcg, from 0.35 to 0.2 (line 37 on subf16_morelli.m), the GCAS system will fail to recover the system. Reachability questions can then be considered about sets of initial states, and sets of model parameters, such as "What range of xcg is guaranteed recoverable?" We plan to release more specific cases with the publication which is currently being prepared.

If you run the script without calls to MakePicture or MakeAnimation (and if analysisOn and printOn are true), there will be output printed in matlab:
```
Pass Fail Conditions:  
           stable: 1  
         airspeed: 1  
            alpha: 1  
             beta: 1  
               Nz: 1  
    psMaxAccelDeg: 1  
         altitude: 1  
     maneuverTime: 1  
```    
This indicates a series of specifications for the system, and if the maneuver met them. All 1's means every specification was met. A failing specification means there was problem. For example, if your altitude goes below 0, the altitude spec will fail. The specific conditions are given starting on line 360 in RunF16Sim.m.

The autopilot and model can be used for more than just GCAS testing. See the autopilot structure in getDefaultSettings.m around line 58. In Main.m on line 52, 'autopilot.simpleGCAS = true' is assigned, indicating the high-level GCAS logic should be active. The actual high-level controller logic is performed in getAutoPilotCommands.m. This logic can also be modified for verification analysis as well. For example, line 114 sets the desired acceleration of the maneuver to 5 g's, which could be adjusted to make the setup more or less aggressive.

Additional documentation will be added as it is created.

## File Structure
### F16_Model
This directory contains a nonlinear mathematical model of the F-16 aircraft published in _Aircraft Control and Simulation: Dynamics, Controls Design, and Autonomous Systems_. This model uses lookup tables to access the aero data for the model. As well, a polynomial model fit to the lookup table data (developed by Eugene Morelli) is included.

### F16_Sim
This directory contains all of the Matlab scripts and functions used to simulate and analyze autonomous maneuvers of the aforementioned F-16 models.

### flypath3d
Flypath3d is an aircraft visualization library for Matlab [3].

"The flypath3d package a free Matlab software for 3D visualizations of missile and air target trajectories that provide a visual reference for the computer simulation. Because of its versatility and ease of use, the software can help to produce attractive presentations for various scientific or public outreach purposes."

This package was modified and tailored for this project to generate visualizations of F-16 autonomous maneuvers with a text overlay of state data and pass/fail conditions.

## Real Automated Ground Collision Avoidance System
This benchmark was inspired by a real F-16 GCAS system that was recently developed by Lockheed Martin, NASA, and the Air Force Research Lab. As of the start of 2018, the system has been confirmed as saving six aircraft (~$25 million each) and seven lives. However, this academic benchmark code and recovery logic are in no way connected to the code or methodology used in the real thing. For more information on the real system, including a video of one of the 'confirmed saves', see [this article in Aviation Week](http://aviationweek.com/air-combat-safety/auto-gcas-saves-unconscious-f-16-pilot-declassified-usaf-footage).

## Feature Requests & Issues
If you discover errors or incompatibilities within the code library, please let me know by submitting a New Issue on GitHub. Also, if you require the library to be reorganized (architecture, inputs & outputs for different functions, etc.) for better integration with  your analysis tools and you believe that the changes would benefit most people, submit a feature request using the "Issues" tool. I may be able to integrate a fix in the master branch or help advise you on how to modify the library in a fork. 

You are also more than welcome to contribute to this benchmark project. The '''getAutopilotCommands''' function, for instance, could be improved by the addition of different autonomous maneuvers or architectures. Pull requests will allow me to integrate any contributions.

## Related Work
A python implementation of the same code library is currently being developed. It is available at https://github.com/stanleybak/AeroBenchVVPython.

## Citations

If you plan on presenting or publishing work done on this benchmark, please contact us at peter.heidlauf.1@us.af.mil for the upcoming citation information. We are currently working on a paper detailing this benchmark.

[1] Stevens, Brian L., Frank L. Lewis, and Eric N. Johnson. Aircraft control and simulation: dynamics, controls design, and autonomous systems. John Wiley & Sons, 2015.  

[2] Morelli, Eugene A. "Global nonlinear parametric modelling with application to F-16 aerodynamics." American Control Conference, 1998. Proceedings of the 1998. Vol. 2. IEEE, 1998.

[3] Bużantowicz W. "Matlab Script for 3D Visualization of Missile and Air Target Trajectories".International Journal of Computer and Information Technology 5 (2016)5, pp. 419-422.

### Release Documentation
Distribution A: Approved for Public Release (88ABW-2017-6379)
