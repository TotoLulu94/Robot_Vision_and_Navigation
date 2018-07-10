% This script runs the GPS

% Configure to disable other sensor types
parameters = minislam.event_generators.simulation.Parameters();
parameters.enableGPS = true;
parameters.enableLaser = false;

% Set up the simulator and the output
simulator = minislam.event_generators.simulation.Simulator(parameters);

kalmanFilterGPSSystem = answers.kalman.GPSLocalizationSystem();
kalmanResults = minislam.mainLoop(simulator, kalmanFilterGPSSystem);

% Run the two simulations
gtsamGPSSystem = answers.gtsam.GPSLocalizationSystem();
gtsamResults = minislam.mainLoop(simulator, gtsamGPSSystem);

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(kalmanResults.optimizationTimes)
hold on
plot(gtsamResults.optimizationTimes)
