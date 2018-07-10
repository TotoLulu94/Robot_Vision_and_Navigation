% This script can be used to develop your SLAM system

% Configure to disable unneed sensors
parameters = minislam.event_generators.simulation.Parameters();
parameters.enableGPS = false;

% Set up the simulator and the output
simulator = minislam.event_generators.simulation.Simulator(parameters);

% Run the localization system
gtsamSLAMSystem = answers.gtsam.LaserSensor2DSLAMSystem();
gtsamResults = minislam.mainLoop(simulator, gtsamSLAMSystem);

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(gtsamResults.optimizationTimes)
