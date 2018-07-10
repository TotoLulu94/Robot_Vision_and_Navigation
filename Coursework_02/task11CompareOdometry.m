% This script runs the odometry

% Configure to disable other sensor types
parameters = minislam.event_generators.simulation.Parameters();
parameters.enableGPS = false;
parameters.enableLaser = false;

% Set up the simulator and the output
simulator = minislam.event_generators.simulation.Simulator(parameters);

% Run the two simulations
kalmanFilterOdometrySystem = answers.kalman.OdometryOnlyLocalizationSystem();
kalmanResults = minislam.mainLoop(simulator, kalmanFilterOdometrySystem);

gtsamOdometrySystem = answers.gtsam.OdometryOnlyLocalizationSystem();
gtsamResults = minislam.mainLoop(simulator, gtsamOdometrySystem);

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('iSLAM Optimization times');
clf
plot(kalmanResults.optimizationTimes, 'r')
hold on
plot(gtsamResults.optimizationTimes, 'b')
