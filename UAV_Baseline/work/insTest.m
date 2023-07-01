
truePosition = [1 1 1];
trueVelocity = [1 1 0];
trueOrientation = [0.2 0.2 0.1];
[position, velocity, orientation] = simINS(truePosition,trueVelocity,trueOrientation)
function [position, velocity, orientation] = simINS(truePosition,trueVelocity,trueOrientation)
gTruth = struct('Position',truePosition,'Velocity',trueVelocity,'Orientation',trueOrientation);
INS = insSensor("PositionAccuracy",0.1);
insMeas = INS(gTruth);
position = insMeas.Position;
velocity = insMeas.Velocity;
orientation = insMeas.Orientation;
end