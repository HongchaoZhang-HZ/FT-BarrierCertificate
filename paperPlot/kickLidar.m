function [angle, pose, score,pcd1, pcd2,scores,bestLabel] = kickLidar(pcDataRaw, pcData2Raw)
% default angle step is 30 degree per sector 
% pcDataRaw do not has adversary
% pcData2Raw has adversary

    maxrange = 90;
    range_raw = pcDataRaw.Ranges;
    angle_raw = pcDataRaw.Angles;
    lenr = length(pcDataRaw.Ranges);
    for i = 1:lenr
        if range_raw(i) >= maxrange
            range_raw(i) = NaN;
        end
    end
    pcDataRaw = lidarScan(range_raw,angle_raw );
    
    range_raw = pcData2Raw.Ranges;
    angle_raw = pcData2Raw.Angles;
    lenr = length(pcData2Raw.Ranges);
    for i = 1:lenr
        if range_raw(i) >= maxrange
            range_raw(i) = NaN;
        end
    end
    pcData2Raw = lidarScan(range_raw,angle_raw );


angle_step = 30; cellsizes = 5;% cellsize = 6;
num_step = floor(30/360*1083);
label = 1: num_step: 1083;
num_label = length(label); % num_label = 13;

pcData1_range_raw = pcDataRaw.Ranges; 
pcData1_angle = pcDataRaw.Angles;
pcData2_range_raw = pcData2Raw.Ranges; 
pcData2_angle = pcData2Raw.Angles;

% pcList = cell(1,length(label));
poseList = zeros(length(label),3);
scoreList = zeros(length(label), 3);

for i = 1: num_label % 13
    % give NaN to specific sectors
    pcData1_range = pcData1_range_raw;
    pcData2_range = pcData2_range_raw;
    if i ~= num_label
        for j= num_step*(i-1)+1: num_step*i
            pcData2_range(j) = NaN;
            pcData1_range(j) = NaN;
        end
    else
        for j = num_step*(i-1)+1: label(end)
            pcData2_range(j) = NaN;
            pcData1_range(j) = NaN;
        end
    end
    pcList2(i)= lidarScan(pcData2_range, pcData2_angle);
    pcList1(i) = lidarScan(pcData1_range, pcData1_angle);
    % matchScans
    [~,scores_anchor] = matchScans(pcList1(i),pcList1(i),'CellSize',cellsizes);
    [pose, scores] = matchScans(pcList2(i), pcList1(i),'CellSize',cellsizes);
    poseList(i,:) = pose;
    scoreList(i,1) = scores_anchor.Score;
    scoreList(i,2) = scores.Score;
    scoreList(i,3) = abs(scoreList(i,1) - scoreList(i,2));
end

[score, bestLabel] = min(scoreList(:,3));

if bestLabel == 1
    angle = [pi, pcData2Raw.Angles(label(2)-1)];
elseif bestLabel == num_label
    angle = [pcData2Raw.Angles(num_step*(bestLabel-1)+1), pcData2Raw.Angles(end)];
else
    angle = [pcData2Raw.Angles(num_step*(bestLabel-1)+1), ...
        pcData2Raw.Angles(num_step*bestLabel)];
end
pose = poseList(bestLabel,:);
angle = angle/(2*pi)*360;
pcd1 = pcList1(bestLabel);
pcd2 = pcList2(bestLabel);
scores = scoreList;

end

