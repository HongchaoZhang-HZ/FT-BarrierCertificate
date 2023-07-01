function scene = uavScenarioBlock(initialPosition, initialOrientation)
%uavScenarioBlock Create cuboid scenario for simulation

%   Copyright 2021 The MathWorks, Inc.

scene = uavScenario("UpdateRate", 2, "ReferenceLocation", [75 -46 0]);
% floor
scene.addMesh("polygon", {[-150 -250; -150 200; 180 200; 180 -250], [-4 0]}, 0.651*ones(1,3));
% buildings
waypointData = load("blockbuildingdata.mat");
buildingDataNED = waypointData.wayPoints1;
buildingDataENU = buildingDataNED;
for idx = 1:numel(buildingDataNED)
    buildingDataENU{idx}(:,1) = buildingDataNED{idx}(:,2)-10.5;
    buildingDataENU{idx}(:,2) = buildingDataNED{idx}(:,1);
end
scene.addMesh("polygon", {buildingDataENU{1}(4:-1:1,:), [0 30]}, [0.3922 0.8314 0.0745]);
scene.addMesh("polygon", {buildingDataENU{2}(2:5,:), [0 30]}, [0.3922 0.8314 0.0745]);
scene.addMesh("polygon", {buildingDataENU{3}(2:10,:), [0 30]}, [0.3922 0.8314 0.0745]);
scene.addMesh("polygon", {buildingDataENU{4}(2:9,:), [0 30]}, [0.3922 0.8314 0.0745]);
scene.addMesh("polygon", {buildingDataENU{5}(1:end-1,:), [0 30]}, [0.3922 0.8314 0.0745]);
scene.addMesh("polygon", {buildingDataENU{6}(1:end-1,:), [0 15]}, [0.3922 0.8314 0.0745]);
scene.addMesh("polygon", {buildingDataENU{7}(1:end-1,:), [0 30]}, [0.3922 0.8314 0.0745]);
scene.addMesh("polygon", {buildingDataENU{8}(2:end-1,:), [0 10]}, [0.3922 0.8314 0.0745]);
scene.addMesh("polygon", {buildingDataENU{9}(1:end-1,:), [0 15]}, [0.3922 0.8314 0.0745]);
scene.addMesh("polygon", {buildingDataENU{10}(1:end-1,:), [0 30]}, [0.3922 0.8314 0.0745]);
scene.addMesh("polygon", {buildingDataENU{11}(1:end-2,:), [0 30]}, [0.3922 0.8314 0.0745]);

plat = uavPlatform("UAV", scene, "ReferenceFrame", "NED", ....
    "InitialPosition", initialPosition, "InitialOrientation", eul2quat(initialOrientation));
plat.updateMesh("quadrotor", {0.65}, [1 0 0], eul2tform([0 0 pi]));

lidarmodel = uavLidarPointCloudGenerator(...
    "AzimuthResolution", 0.3324099, "ElevationLimits", [-20 20],...
    "ElevationResolution", 1.25, "MaxRange", 90, ...
    "HasOrganizedOutput", true, "UpdateRate", 2);
uavSensor("Lidar", plat, lidarmodel, "MountingLocation", [0,0,-0.4], "MountingAngles", [0 0 180]);
end

