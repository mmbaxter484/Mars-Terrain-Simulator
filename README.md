# Mars-Terrain-Simulator


Introduction

Software was created using MATLAB version 2020b.

For the software developed, it has been streamlined as much as possible through the use of command window prompts. This is to save the user having to go hunting 
for specific lines to comment/uncomment and where to change key parameters. There will likely be errors that crop up, other than the ones specified in this
document. If this is the case and any questions need answered, feel free to get in touch through mmbaxter484@gmail.com.

A.2 Mars Surface Model

A.2.1 DTM Generation

• The website that the digital terrain models can be found on is: 
  https://www.uahirise.org/dtm/
• The blender software required for transformation and the necessary add-on can be found on:
  https://www.blender.org/
  https://github.com/phaseIV/Blender-Hirise-DTM-Importer
• The instructions to transfer the downloaded digital terrain model to an stl file are provided from the following sites:
  https://www.uahirise.org/dtm/howto.php
  https://hemelmechanica.nl/hirise-docs/quickstart.html
  https://www.youtube.com/watch?v=SeGe2PHTgQI&t=372s
• Depending on the computer the transfer is attempted on, it will usually struggle to get past a resolution of 3mpp, and will result in large file sizes.

A.2.2 MATLAB Code

Main File: Mars_Surface.m
Required Functions: stl_to_grid.m, segment.m, segment_inter.m

• The first point is to ensure that the DTM name is entered correctly at the start of the code to be read in.
• Next is to set the desired resolution for the overall model, it is recommended that it is set to the original resolution of the imported DTM.
• From there the code can be run, three initial plots will be displayed, the whole imported model before gridding function, the whole imported model after the
  gridding function, and a point cloud plot of the whole area. These typically can’t be interacted with (rotated, zoomed in etc) until after the subsequent prompts
  have been completed.
• There were will then be prompts presented, the first is to choose a segment square side dimension, i.e. a choice of 100 will give a 100mx100m square.
• The next prompt will be to select a specific segment, the options are to either pick the nth segment along both the x and y axis, or to use the whole plot
  of the terrain to click on a desired point, this should display the x/y coordinates which can then be entered. To choose the method the relevant lines
  can be commented or uncommented depending on which method is desired, although the second option is recommended.
• The next option is to choose the resolution of the chosen segment. This can’t be
  chosen as a specific value as the relevant function only takes integers, therefore 5 options are presented which can be chosen from.
• If a different segment is desired it is recommended that everything above the three whole terrain plots are commented out.

A.3 Rover Interaction Model

Main file: Mars_Rover_Main.m
Partner script: Mars_Surface.m or Incline_Generator.m
Required functions:
Mars_Rover_Dynamics.m, Rover_Motor_Model_v1.m, Rover_Heights.m, Rover_Euler_Angles.m, terramechanics_model.m

• The segment chosen from the surface model code can be used, or a flat incline generator has also been provided.
• The most obvious issue with this model is that if the rover exceeds the boundaries of the terrain, the code will end with an error and not display outputs.
This needs to be corrected so that the code will end instead.

A.3.1 MATLAB Code

• The parameters that can be changed outwith the prompts are the simulation time, stepsize and the rover input voltages.
• The rover starting coordinates must also be selected, it is recommended to pick these based off of the plot of the terrain segment chosen. Please note that
  the y-reference frame has been reversed from the plots obtained in the surface model code.
  If the coordinate selection prompt section is uncommented, the terrain segment will be displayed with the correct reference frame and prompts will be
  presented to pick starting position. However depending on the version of MATLAB used this isn’t always seamless, therefore it is recommended that
  the terrain plot code is left commented but copied and pasted to the command window to show the segment plot before the full code is run.
• The code can then be run. The first prompt presented will be to choose to represent the terrain using either the friction model or the terramechanics model,
  if the friction model is selected the code will run, if the terramechanics option is selected another prompt will be presented to choose between the surface
  being a representation of Mars regolith or Mars sand.
• When the code has finished running the option to run an animation will be presented. If the terrain segment is at a high resolution and large size the
  animation will likely be slow. This can be improved by uncommenting the terrain plot within the animation section of code. The angles between the top
  and bottom of the rover may look incorrect, but this is due to the axis scaling options, which can be changed.
  
Terramechanics Model

• If the terramechanics option has been selected the code will run significantly slower than the friction option. Shorter length simulations are therefore recommended.
• This could be potentially be improved by changing any of the step size, contact angle step size, sinkage step size or the force calculation accuracy boundary.
  However please note that making any of the terramechanics step sizes too large could result in the vertical force calculation missing the constraint boundary
  and not providing correct results. It should be relatively obvious if this occurs but should still be looked out for.
• The max pitch angle constraints were set based on the rover having assumed maximum input voltages of 7.2V, therefore if higher voltages were used then the
  values would need to be changed.
• As outlined in the report the lateral forces have not yet been introduced and the effect of negative slip ratio when descending slopes has not been fully tested,
  so these should be considered when using the terramechanics model.
• A separate terramechanics model not tied to the rover model has been provided to allow for testing of the wheel-terrain interaction with respect to slip ratio.
