function [angle, pose, score,scores,pcData_new,pcLidar_new] = kickLidar_pcdata(pcData, pcLidar)
        % default angle step is 30 degree per sector
        % pcDataRaw do not has adversary which is estimation
        % pcData2Raw has adversary which is Lidar

        angle_step = 30; cellsizes = 5;
        num_step = floor(angle_step/360*1083);
        label = 1: num_step: 1083;
        num_label = length(label); % num_label = 13;

        pcData1_range_raw = pcData.Ranges;
        pcData1_angle = pcData.Angles;
        pcData2_range_raw = pcLidar.Ranges;
        pcData2_angle = pcLidar.Angles;

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
            pcList2 = lidarScan(pcData2_range, pcData2_angle);
            pcList1 = lidarScan(pcData1_range, pcData1_angle);
            % matchScans
            [~,scores_anchor] = matchScans(pcList1,pcList1,'CellSize',cellsizes);
            [pose, scores] = matchScans(pcList2, pcList1,'CellSize',cellsizes);
            poseList(i,:) = pose;
            scoreList(i,1) = scores_anchor.Score;
            scoreList(i,2) = scores.Score;
            scoreList(i,3) = abs(scoreList(i,1) - scoreList(i,2));
        end

        [score, bestLabel] = min(scoreList(:,3));

        if bestLabel == 1
            angle = [pi, pcLidar.Angles(label(2)-1)];
        elseif bestLabel == num_label
            angle = [pcLidar.Angles(num_step*(bestLabel-1)+1), pcLidar.Angles(end)];
        else
            angle = [pcLidar.Angles(num_step*(bestLabel-1)+1), ...
                pcLidar.Angles(num_step*bestLabel)];
        end
        pose = poseList(bestLabel,:);
        angle = angle/(2*pi)*360;
        %         pcd1 = pcList1(bestLabel);
        %         pcd2 = pcList2(bestLabel);
        scores = scoreList;
        i = bestLabel; % 13
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
        pcLidar_new = lidarScan(pcData2_range, pcData2_angle);
        pcData_new = lidarScan(pcData1_range, pcData1_angle);
        % matchScans
       
       
%         if dis == true
%             scoreList
%             poseList
%             angle
% 
%         end
    end