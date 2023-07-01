% This script initializes workspace variables for the UAV Toolbox Reference
% Application.

% Set guidance type to Obstacle Avoidance
guidanceType = 2;

% Configure the drone as a Multicopter
isDroneMulticopter = 1;

% Use cuboid environment
isPhotoRealisticSim = 1;

% Low fidelity plant model 
plantModelFi = 0;

% Show the Lidar Point Cloud
showLidarPointCloud = 1;

% No show the Video Viewer
showVideoViewer = 0;

% Do not show the UAV Animation as it flies
showUAVAnimation = 0;

% Use heading in the guidance model
useHeading = 1;

%Takeoff after 0.5
startFlightTime = 0.5;

% Do not use QGroundControl
useQGC = 0;

% No Pacing
load_system('uavPackageDelivery');
set_param('uavPackageDelivery','EnablePacing', 'off');

% Simulation Stop Time
simTime = 70;

%Show the CPA Scope
open_system('uavPackageDelivery/On Board Computer/DataProcessing/ProcessSensorData/CPA');

% Done
disp ('Project configured to simulate in a cuboid environment with obstacle avoidance');
