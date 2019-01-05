function [x,y] = coverage_agent()

global sensor_range infinity planner;

% A simple hack which blindly goes towards the goal
% samples = norm( qgoal - qstart ) / (sensor_range / 10);
% range = linspace(0,1,samples);
%
% x = qstart(1) + range*(qgoal(1) - qstart(1));
% y = qstart(2) + range*(qgoal(2) - qstart(2));

switch(planner.state)
    case 0 % Motion-to-Obstacle
        switch (planner.yDirect)
            case 1 %Up
                dist2Bnd = read_sensor(pi/1.9978,planner.qcurr);
                dist2Bnd_r = read_sensor(mod(0.0001*planner.xDirect,pi),planner.qcurr);
                if dist2Bnd < planner.safeDist 
                    x = planner.qcurr(1);
                    y = planner.qcurr(2);
                    planner.yDirect = -planner.yDirect;
                end
                
                flag = ismember(planner.qcurr,planner.critPoint);
                if dist2Bnd_r <= planner.safeDist && dist2Bnd == infinity && ~flag(1) && planner.obsChInd == 0
                    if size(planner.critPoint,1) ~= 0
                        dist = [];
                        for i=1:size(planner.critPoint,1)
                            dist(i) = norm(planner.critPoint(i,:) - planner.qcurr);
                        end
                        safeMargin = 0.25;
                        distInd = find(dist<safeMargin);
                        if distInd ~=0
                            x = planner.critPoint(distInd-1,1);
                            y = planner.critPoint(distInd-1,2);
                            planner.scanFlag = 1;
                            return;
                        end
                    end
                   
                    planner.graphMap(end,2) = max(max(planner.graphMap)) +1;
                    planner.graphMap(end+1,1) = planner.graphMap(end,2);
                    temp = planner.critPoint(end,:);
                    planner.critPoint(end,:) = planner.qcurr;
                     
                    if(planner.obsChInd == 0)
                        planner.obsChInd = size(planner.critPoint,1);
                    end
                    planner.critPoint(end+1,:) = temp; % To keep lead point
                    if dist2Bnd == infinity
                        x = planner.qcurr(1) + planner.xDirect*planner.safeDist/3;
                        y = planner.qcurr(2) + sensor_range;
                    else
                        x = planner.qcurr(1) + planner.xDirect*planner.safeDist/3;
                        y = planner.qcurr(2) + dist2Bnd-planner.safeDist;
                    end
                    return;
                end
                
                planner.surroundings = [planner.surroundings dist2Bnd_r];
                if (dist2Bnd > planner.safeDist)
                    if dist2Bnd <= sensor_range
                        %                         if ~ismember(infinity,planner.surroundings)
                        %                             planner.xDirect = -planner.xDirect;
                        %                             planner.firstMove = 1;
                        %                         end
                        if planner.qcurr(2) > planner.maxBound
                            planner.maxBound = planner.qcurr(2);
                        end
                        if planner.qcurr(2) < planner.minBound
                            planner.minBound = planner.qcurr(2);
                        end
                        
                        if planner.maxBound-sensor_range < planner.qcurr(2) || planner.minBound +sensor_range > planner.qcurr(2)
                            distance = dist2Bnd - planner.safeDist;
                            x = planner.qcurr(1);
                            y = planner.qcurr(2) + planner.yDirect*distance;
                            planner.state = 1; % Switch to boundary following later on
                            
                            planner.surroundings = [];
                        else
                            x = planner.qcurr(1);
                            y = planner.qcurr(2);
                            planner.state = 2; % Switch to boundary following later on
                            
                        end
                        planner.angle = pi/1.9978;
                    else
                        distance = (sensor_range-2*planner.safeDist)/10;
                        x = planner.qcurr(1);
                        y = planner.qcurr(2) + planner.yDirect*distance;
                    end
                end
                
            case -1 %Down
                dist2Bnd = read_sensor(3*pi/1.9978,planner.qcurr);
                dist2Bnd_r = read_sensor(mod(0.0001*planner.xDirect,pi),planner.qcurr);
                if dist2Bnd < planner.safeDist 
                    x = planner.qcurr(1);
                    y = planner.qcurr(2);
                    planner.yDirect = -planner.yDirect;
                end
                flag = ismember(planner.qcurr,planner.critPoint);
                if dist2Bnd_r <= planner.safeDist && dist2Bnd == infinity && ~flag(1) && planner.obsChInd == 0
                    if size(planner.critPoint,1) ~= 0
                        dist = [];
                        for i=1:size(planner.critPoint,1)
                            dist(i) = norm(planner.critPoint(i,:) - planner.qcurr);
                        end
                        safeMargin = 0.25;
                        distInd = find(dist<safeMargin);
                        if distInd ~=0
                            x = planner.critPoint(distInd-1,1);
                            y = planner.critPoint(distInd-1,2);
                            planner.scanFlag = 1;
                            return;
                        end
                    end
                    planner.graphMap(end,2) = max(max(planner.graphMap)) +1;
                    planner.graphMap(end+1,1) = planner.graphMap(end,2);
                    temp = planner.critPoint(end,:);
                    planner.critPoint(end,:) = planner.qcurr;
                     
                    if(planner.obsChInd == 0)
                        planner.obsChInd = size(planner.critPoint,1);
                    end
                    planner.critPoint(end+1,:) = temp; % To keep lead point
                    if dist2Bnd == infinity
                        x = planner.qcurr(1) + planner.xDirect*planner.safeDist/3;
                        y = planner.qcurr(2) - sensor_range;
                    else
                        x = planner.qcurr(1) + planner.xDirect*planner.safeDist/3;
                        y = planner.qcurr(2) - dist2Bnd-planner.safeDist;
                    end
                    planner.yDirect = 1; % To keep up-prior sensor-based coverage
                    return;
                end
                planner.surroundings = [planner.surroundings dist2Bnd_r];
                if (dist2Bnd > planner.safeDist)
                    if dist2Bnd <= sensor_range
                        %                         if ~ismember(infinity,planner.surroundings)
                        %                             planner.xDirect = -planner.xDirect;
                        %                             planner.firstMove = 1;
                        %                         end
                        if planner.qcurr(2) > planner.maxBound
                            planner.maxBound = planner.qcurr(2);
                        end
                        if planner.qcurr(2) < planner.minBound
                            planner.minBound = planner.qcurr(2);
                        end
                        
                        if planner.maxBound-sensor_range < planner.qcurr(2) || planner.minBound +sensor_range > planner.qcurr(2)
                            distance = dist2Bnd - planner.safeDist;
                            x = planner.qcurr(1);
                            y = planner.qcurr(2) + planner.yDirect*distance;
                            planner.state = 1; % Switch to boundary following later on
                            planner.surroundings = [];
                        else
                            x = planner.qcurr(1);
                            y = planner.qcurr(2);
                            planner.state = 2; % Switch to boundary following later on
                            
                        end
                        planner.angle = 3*pi/1.9978;
                    else
                        distance = (sensor_range-2*planner.safeDist);
                        x = planner.qcurr(1);
                        y = planner.qcurr(2) + planner.yDirect*distance;
                    end
                end
        end
    case 1 % Boundary Following Case
        dist2Bnd_cent = read_sensor(planner.angle, planner.qcurr); % Center ray
        dist2Bnd_lead = read_sensor(planner.angle - planner.yDirect*planner.rayAngle, planner.qcurr); % Right ray
        dist2Bnd_lag = read_sensor(planner.angle + planner.yDirect*planner.rayAngle, planner.qcurr); % Left ray
        
        %Critical Point Checker
        
        hp_cent = planner.qcurr + dist2Bnd_cent*[cos(planner.angle) sin(planner.angle)];
        hp_lead = planner.qcurr + dist2Bnd_lead*[cos(planner.angle - planner.yDirect*planner.rayAngle) sin(planner.angle - planner.yDirect*planner.rayAngle)];
        hp_lag = planner.qcurr + dist2Bnd_lag*[cos(planner.angle + planner.yDirect*planner.rayAngle) sin(planner.angle + planner.yDirect*planner.rayAngle)];
        
        line1 = [(hp_cent(1)-hp_lead(1)) (hp_cent(2)-hp_lead(2))];
        line2 = [(hp_lead(1)-hp_lag(1)) (hp_lead(2)-hp_lag(2))];
        
        if abs(dot(line2,line1)) ~= norm(line1)*norm(line2)
            planner.graphMap(end,2) = planner.graphMap(end,1) +1;
            planner.graphMap(end+1,1) = planner.graphMap(end,2);
            planner.critPoint = [planner.critPoint;planner.qcurr];
        end
        
        dist2Bnd_side = read_sensor(mod(0.0001*planner.xDirect,pi),planner.qcurr);
        
        if dist2Bnd_side == infinity
            x = planner.qcurr(1)+planner.xDirect*sensor_range/10;
            y = planner.qcurr(2);
        else
            hp_side = planner.qcurr + dist2Bnd_side*[cos(mod(0.0001*planner.xDirect,pi)) sin(mod(0.0001*planner.xDirect,pi))];
            line1 = [(hp_cent(1)-planner.qcurr(1)) (hp_cent(2)-planner.qcurr(2))];
            line2 = [(planner.qcurr(1)-hp_side(1)) (planner.qcurr(2)-hp_side(2))];
            
            %To find critical point on the border boundary
            if dot(line1,line2) < planner.minStep
                if planner.obsChInd ~= 0
                    obsStartPoint = find(planner.obsChInd==planner.graphMap(:,2));
                    if size(obsStartPoint,1) >1
                        planner.graphMap(end,2) = max(max(planner.graphMap)) +1;
                        planner.critPoint(end,:) =  planner.qcurr;
                        x = planner.qcurr(1);
                        y = planner.qcurr(2);
                        planner.finishFlag = 1;
                        return;
                    end
                    
                    
                    planner.graphMap(end,2) = planner.graphMap(end,1) +1;
                    planner.graphMap(end+1,1) = planner.graphMap(end,2)-1;
%                     planner.graphMap(end,2) = max(max(planner.graphMap)) +1;
%                     planner.graphMap(end+1,1) = planner.graphMap(end,2);
%                     planner.graphMap(end,2) = planner.graphMap(end-1,2) +1;
%                     planner.graphMap(end+1,1) = planner.graphMap(end,2)-1;
                    planner.scanFlag = -1;
                else
                    planner.graphMap(end,2) = max(max(planner.graphMap)) +1;
                    planner.graphMap(end+1,1) = planner.graphMap(end,2)-1;
                end
                x = planner.critPoint(end,1);
                y = planner.critPoint(end,2) - sensor_range;
                temp = planner.critPoint(end,:);
                planner.critPoint(end,:) = planner.qcurr;
                planner.critPoint(end+1,:) = temp; % To keep lead point
                planner.xDirect = -planner.xDirect;
                
                planner.yDirect = -planner.yDirect;
                planner.state = 0;
                
                return;
            end
            x = planner.qcurr(1) + planner.xDirect*dist2Bnd_side/20;
            y = planner.qcurr(2);
        end
        planner.state = 0;
        planner.yDirect = -planner.yDirect;
        planner.angle = mod(planner.yDirect*pi/1.9978,2*pi);
        
    case 2 % Obstacle Following Case Up prior following
        %Critical point finder
        dist2Bnd_cent = read_sensor(planner.angle, planner.qcurr); % Center ray
        if dist2Bnd_cent > sensor_range - 2*planner.safeDist
            x= planner.qcurr(1);
            y= planner.qcurr(2) + planner.yDirect*(dist2Bnd_cent - 2*planner.safeDist);
            return;
        end
        dist2Bnd_lead = read_sensor(planner.angle - planner.yDirect*planner.rayAngle, planner.qcurr); % Right ray
        dist2Bnd_lag = read_sensor(planner.angle + planner.yDirect*planner.rayAngle, planner.qcurr); % Left ray
        
        hp_cent = planner.qcurr + dist2Bnd_cent*[cos(planner.angle) sin(planner.angle)];
        hp_lead = planner.qcurr + dist2Bnd_lead*[cos(planner.angle - planner.yDirect*planner.rayAngle) sin(planner.angle - planner.yDirect*planner.rayAngle)];
        hp_lag = planner.qcurr + dist2Bnd_lag*[cos(planner.angle + planner.yDirect*planner.rayAngle) sin(planner.angle + planner.yDirect*planner.rayAngle)];
        
        line1 = [(hp_cent(1)-hp_lead(1)) (hp_cent(2)-hp_lead(2))];
        line2 = [(hp_lead(1)-hp_lag(1)) (hp_lead(2)-hp_lag(2))];
        
        % If robot encounters any critical point while scanning the
        % obstacle
        
        if (abs(abs(dot(line1,line2)) - norm(line1)*norm(line2)) > planner.minStep) 
            %To check whether the critical point is found or not
            if size(planner.critPoint,1) ~= 0
                dist = [];
                for i=1:size(planner.critPoint,1)-1
                    dist(i) = norm(planner.critPoint(i,:) - planner.qcurr);
                end
                safeMargin = 0.3;
                if size(dist,1) ~= 0
                    distInd = find(dist<safeMargin);
                    if size(distInd,2) == 0
                        distInd = 0;
                    end
                else
                    distInd = 0;
                end
                
                dist2Bnd_lead = read_sensor(planner.angle + planner.xDirect*planner.rayAngle, planner.qcurr); % Right ray
                dist2Bnd_lag = read_sensor(planner.angle - planner.xDirect*planner.rayAngle, planner.qcurr); % Left ray
        
                if distInd  == 0
                    if (dist2Bnd_lag ~= infinity && dist2Bnd_lead ~= infinity) 
                        %     R
                        %     *
                        %   *   *
                        %  *      *
                        % *         *
                        %
                        % Robot(R) 
                        planner.graphMap(end,2) = max(max(planner.graphMap)) +1;
                        planner.graphMap(end+1,1) = planner.graphMap(end,2);
                        temp = planner.critPoint(end,:);
                        planner.critPoint(end,:) = planner.qcurr;
                        planner.critPoint(end+1,:) = temp; % To keep lead point
                        
                    elseif (dist2Bnd_lag ~= infinity && dist2Bnd_lead == infinity)
                        
                        %     
                        %     *
                        %   *   *
                       % R *      *
                        % *         *
                        %   * 
                        %     *
                        % Robot(R), if robot encounters final critical
                        % point of the obstacle from the upside continue
                        
                        planner.graphMap(end,2) = max(max(planner.graphMap)) +1;
                        planner.graphMap(end+1,1) = planner.graphMap(end,2);
                        temp = planner.critPoint(end,:);
                        planner.critPoint(end,:) = planner.qcurr;
                        planner.critPoint(end+1,:) = temp; % To keep lead point
                        
                    end
                else
                    if planner.scanFlag == -1
                        %     
                        %     *
                        %   *   *
                        %  *      *
                        % *         * 
                        %*            * C
                        % *           *
                        %   *       * R
                        %     *   *
                        %       *
                        %Robot(R), Critical Point (C) is encountered while
                        %scanning from the downside for up-prior coverage
                        x = planner.critPoint(distInd-1,1);
                        y = planner.critPoint(distInd-1,2);
                        planner.graphMap(end,2) = distInd;
                        planner.graphMap(end+1,1) = planner.graphMap(end,2)-1;
                        planner.yDirect = -planner.yDirect;
                        planner.angle = mod(planner.yDirect*pi/1.9978,2*pi);
                        planner.state = 0;
                        return;
                    else
                        %     
                        %     *
                        %   *   *
                        %  *      *
                        % *         * R
                        %*            * C
                        % Robot(R), Critical point(C) is encountered 
                                               planner.graphMap(end,2) = max(max(planner.graphMap)) +1;
 %again
                        x = planner.qcurr(1) + planner.xDirect*sensor_range/20;
                        y = planner.qcurr(2);
                        planner.yDirect = -planner.yDirect;
                        planner.angle = mod(planner.yDirect*pi/1.9978,2*pi);
                        planner.state = 0;
                        return;
                    end
                end
                
                
%                 %A special case if robot encounters first obstacle
%                 %critical point again
%                 tempVar = find(planner.graphMap(:,1)==planner.obsChInd);
%                 tempInd = tempVar(1);
%                 
%                 
%                 if  (distInd ~=0) && (planner.graphMap(tempInd,2) == 0) % || (dist2Bnd_lag == infinity || dist2Bnd_lead == infinity)
%                     x = planner.qcurr(1) + planner.xDirect*sensor_range/50;
%                     y = planner.qcurr(2);
%                     planner.yDirect = -planner.yDirect;
%                     planner.angle = mod(planner.yDirect*pi/1.9978,2*pi);
%                     planner.state = 0;
%                     return;
%                     %If total scanning of the obstacle is finished go the
%                     %previous critical point of the current point which is  first
%                     %critical point of the current obstacle
%                 elseif (distInd ~=0)
%                     x = planner.critPoint(distInd-1,1);
%                     y = planner.critPoint(distInd-1,2);
%                     planner.graphMap(end,2) = distInd;
%                     planner.graphMap(end+1,1) = planner.graphMap(end,2)-1;
%                     planner.yDirect = -planner.yDirect;
%                     planner.angle = mod(planner.yDirect*pi/1.9978,2*pi);
%                     planner.state = 0;
%                     return;
%                 end
%             end
%             planner.graphMap(end,2) = max(max(planner.graphMap)) +1;
%             planner.graphMap(end+1,1) = planner.graphMap(end,2);
%             temp = planner.critPoint(end,:);
%             planner.critPoint(end,:) = planner.qcurr;
%             planner.critPoint(end+1,:) = temp; % To keep lead point
        end
        
        end
        dist2Bnd_side = read_sensor(mod(0.0001*planner.xDirect,pi),planner.qcurr);
        
        
        if dist2Bnd_side == infinity
            x = planner.qcurr(1)+planner.xDirect*sensor_range/100;
            y = planner.qcurr(2);
        else
            x = planner.qcurr(1) + planner.xDirect*dist2Bnd_side/100;
            y = planner.qcurr(2);
        end
        planner.state = 0;
        planner.yDirect = -planner.yDirect;
        planner.angle = mod(planner.yDirect*pi/1.9978,2*pi);
        planner.critPoint(end,:) = planner.qcurr;
        
end
end

