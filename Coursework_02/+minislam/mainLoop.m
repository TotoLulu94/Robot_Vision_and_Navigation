function results = mainLoop(eventGenerator, localizationSystems)

% This function runs the main loop for the localization system

% Set up the graphics for output
graphicalOutput = minislam.graphics.GraphicalOutput();
graphicalOutput.initialize(eventGenerator, localizationSystems);

% Set the results structure
results = minislam.Results();

% Start the event generator
eventGenerator.start();

% Step the event generator
eventGenerator.step();

while (eventGenerator.keepRunning())
    
    % Get the step number
    stepNumber = eventGenerator.getStepNumber();
    
    % Get the events and generate the events
    events = eventGenerator.getEvents();

    localizationSystems.processEvents(events);
    
    tic
    localizationSystems.optimize();
    results.optimizationTimes(stepNumber + 1) = toc;
    
    if (rem(stepNumber, 500) == 0)
        disp(stepNumber)
    end    
    
    % Log results
    groundTruthState = eventGenerator.getGroundTruth(false);
    [x, P] = localizationSystems.getCurrentMeanAndCovarianceEstimate();
    
    results.vehicleTrueStateHistory(:, stepNumber + 1) = groundTruthState.xTrue;
    results.vehicleStateHistory(:, stepNumber + 1) = [x.x();x.y();x.theta()];
    results.vehicleCovarianceHistory(:, stepNumber + 1) = diag(P);
    
    % Draw the graphics. Note this draws once every third frame which is very smooth but
    % can be slow. The graphics only update when "true" is passed in.
    graphicalOutput.update(rem(stepNumber, 3) == 0);
    
    eventGenerator.step();
end

end