% Communications MatLab <--> Arduino
% Matlab file 1 for use with Arduino file 1
clc;
clear;
close all;

%% Parameters
SIMULATION_MODE = false;
PRINT_PERIOD = 200; %ms
numSec = 90;

%% Video output setup
dataVideo = VideoWriter('dataVideo.avi');
dataVideo.FrameRate = 5;  % Default 30
open(dataVideo);

%% Serial setup

if ~SIMULATION_MODE
    fclose(instrfind());
    s1 = serial('COM8'); % Serial port
    s1.BaudRate=115200;               % define baud rate
    set(s1, 'terminator', 'LF');    % define the terminator for println
    fopen(s1);
    timeLength = numSec*1000/PRINT_PERIOD;
else
    load('simData.mat');
    simRow = 1;
    timeLength = length(simData(:,1));
end

% Initialise real-time plots
figure
subplot(3,1,1);
plot1 = animatedline;
subplot(3,1,2);
plot2 = animatedline;
subplot(3,1,3);
plot3 = animatedline;
title('Angular Momentum');
ylabel('kgm^2/s');
xlabel('seconds');

%data = zeros(1,5);
data = [];
plottingPrediction = false;
%% Serial comms
try                             % use try catch to ensure fclose

timeIndex=0;

while (timeIndex < timeLength)
    timeIndex = timeIndex+1;
    if ~SIMULATION_MODE
        % Serial
        sample=fscanf(s1,'%s');
        sample = textscan(sample, '%f%f%f', 'Delimiter', ',');
        sample = cell2mat(sample);
        sample = [sample, 0, 0]; % Initialise calculated values to 0
    else
        % Use simulated data
        sample = simData(simRow,:);
        simRow = simRow + 1;
    end
    
    data = [data; sample]; % Add a new row to data table
    
    % Slice a 5 second chunk of arrays
    time = data(end,1)/1000;
    speed = data(end,2);
    distance = data(end,3);
    
    % Estimate angular momentum loss
    
    % Mass moment of inertia calculation
    MMI = 6.364e-1*distance(end)^2 + 2.456e-2*distance(end) + 1.247e-2; % Apply fitted polynomial
    data(end,4) = MMI; % Time slice
    
    % Angular momentum calculation
    momentum = speed(end).*MMI(end);
    data(end,5) = momentum; % Time slice

    % Live plot data
    subplot(3,1,1);
    addpoints(plot1, time(end), speed(end));
    %ylim([0 10]);
    title('Speed');
    ylabel('rad/s');
    
    subplot(3,1,2);
    addpoints(plot2, time(end), MMI(end));
    ylim([0 0.035]);
    title('Mass Moment of Inertia');
    ylabel('kgm^2');

    subplot(3,1,3);
    hold on
    addpoints(plot3, time(end), momentum(end));
    %ylim([0 0.5]);
    hold off

    drawnow;
    writeVideo(dataVideo, getframe(gcf));
end
if ~SIMULATION_MODE
    fclose(s1);
end
catch me
    if ~SIMULATION_MODE 
        fclose(s1); % always, always want to close s1
        close(dataVideo);
    end             
    rethrow(me);
end

close(dataVideo);