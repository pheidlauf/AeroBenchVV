function [] = stopSimulink()
%stopSimulink Stop the simulation
   coder.extrinsic('bdroot', 'set_param');

   disp('Stopping simulation');
   set_param(bdroot,'SimulationCommand','stop');
end
