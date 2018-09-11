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
                planner.surroundings = [planner.surroundings dist2Bnd_r];
                if (dist2Bnd > planner.safeDist)
                    if dist2Bnd <= sensor_range
                        if ~ismember(infinity,planner.surroundings)
                            planner.xDirect = -planner.xDirect;
                            planner.firstMove = 1;
                        end
                        distance = dist2Bnd - planner.safeDist;
                        x = planner.qcurr(1);
                        y = planner.qcurr(2) + planner.yDirect*distance;
                        planner.state = 1; % Switch to boundary following later on
                        planner.angle = pi/1.9978;
                        planner.surroundings = [];
                    else
                        distance = (sensor_range-2*planner.safeDist);
                        x = planner.qcurr(1);
                        y = planner.qcurr(2) + planner.yDirect*distance;
                    end
                end
                
            case -1 %Down 
                dist2Bnd = read_sensor(3*pi/1.9978,planner.qcurr);
                dist2Bnd_r = read_sensor(mod(0.0001*planner.xDirect,pi),planner.qcurr);
                planner.surroundings = [planner.surroundings dist2Bnd_r];
                if (dist2Bnd > planner.safeDist)
                    if dist2Bnd <= sensor_range
                        if ~ismember(infinity,planner.surroundings)
                            planner.xDirect = -planner.xDirect;
                            planner.firstMove = 1;
                        end
                        distance = dist2Bnd - planner.safeDist;
                        x = planner.qcurr(1);
                        y = planner.qcurr(2) + planner.yDirect*distance;
                        planner.state = 1; % Switch to boundary following later on
                        planner.angle = 3*pi/1.9978;
                        planner.surroundings = [];
                    else
                        distance = (sensor_range-2*planner.safeDist);
                        x = planner.qcurr(1);
                        y = planner.qcurr(2) + planner.yDirect*distance;
                    end
                end
        end
        
    case 1 % Boundary Following Case
        dist2Bnd_c = read_sensor(planner.angle, planner.qcurr); % Center ray
        dist2Bnd_1 = read_sensor(planner.angle - planner.yDirect*planner.rayAngle, planner.qcurr); % Right ray
        dist2Bnd_2 = read_sensor(planner.angle + planner.yDirect*planner.rayAngle, planner.qcurr); % Left ray

%         if dist2Bnd_c <= planner.safeDist - planner.minStep
%             angle = mod(planner.angle + pi,2*pi);
%             dest = planner.qcurr + (planner.safeDist - dist2Bnd_c)*[cos(angle) sin(angle)];
%             x = dest(1);
%             y = dest(2);
%             return;
%         end
        switch(planner.xDirect)
            case 1 %Right
                if dist2Bnd_1 ~= infinity && dist2Bnd_2 ~= infinity
                    rayDist_1 = sqrt(dist2Bnd_1^2+dist2Bnd_c^2-2*cos(planner.rayAngle)*dist2Bnd_1*dist2Bnd_c);
                    hitAngle_1 = acos(( dist2Bnd_c^2 + rayDist_1^2-dist2Bnd_1^2)/(2*dist2Bnd_c*rayDist_1));
                    
                    rayDist_2 = sqrt(dist2Bnd_2^2+dist2Bnd_c^2-2*cos(planner.rayAngle)*dist2Bnd_2*dist2Bnd_c);
                    hitAngle_2 = acos(( dist2Bnd_c^2 + rayDist_2^2-dist2Bnd_2^2)/(2*dist2Bnd_c*rayDist_2));
                    
                    if  abs(pi - (hitAngle_2+hitAngle_1)) > planner.minStepAngle
                        planner.critPoint = [planner.critPoint;planner.qcurr];
                        if pi < (hitAngle_2+hitAngle_1) %If the critical point is Convex
                            planner.graphMap = [planner.graphMap;1];
                        else %If the critical point is Concave
                            planner.graphMap = [planner.graphMap;2];
                        end
                        
                    end
%                     hitAngle_1 = mod(hitAngle_1*planner.yDirect,2*pi);
                    hp1 = planner.qcurr + dist2Bnd_c*[cos(planner.angle) sin(planner.angle)]; %1st ray hitpoint
                    hp2 = hp1 + rayDist_1*[cos( planner.angle-planner.yDirect*(pi-hitAngle_1)) sin(planner.angle-planner.yDirect*(pi-hitAngle_1))]; %2nd ray hitpoint
                    planner.angle = mod(planner.angle-planner.yDirect*pi,2*pi);
                    dest = hp2 + (sensor_range - 2*planner.safeDist)*[cos(planner.angle) sin(planner.angle)];
                     x = dest(1);
                    y = dest(2);
                    
                    %If 2nd and 3rd ray distance measurement is infinity decrease the
                    %angle between the rays and find the closest ray
                else
                    %If at least one of the ray is measuring infinity that
                    %means there will be a critical point
                    planner.critPoint = [planner.critPoint;planner.qcurr];
                    if dist2Bnd_1 == infinity
                        planner.graphMap = [planner.graphMap;planner.yDirect];
                        rayDist_1 = sqrt(dist2Bnd_2^2+dist2Bnd_c^2-2*cos(planner.rayAngle)*dist2Bnd_2*dist2Bnd_c);
                        hitAngle_1 = real(acos(( dist2Bnd_c^2 + rayDist_1^2-dist2Bnd_2^2)/(2*dist2Bnd_c*rayDist_1)));
                        
                        hp1 = planner.qcurr + dist2Bnd_c*[cos(planner.angle) sin(planner.angle)]; %1st ray hitpoint
                        hp2 = hp1 + 2*rayDist_1*[cos(planner.angle-planner.yDirect*(-hitAngle_1+pi)) sin(planner.angle-planner.yDirect*(-hitAngle_1+pi))]; %2nd ray hitpoint
                        
                        planner.angle = mod(-planner.angle,2*pi);
                        dest = hp2 + 2*[cos(planner.angle) sin(planner.angle)];
                        x = dest(1);
                        y = dest(2);
                    elseif dist2Bnd_2 == infinity
                        planner.graphMap = [planner.graphMap;planner.yDirect];
                        rayDist_2 = sqrt(dist2Bnd_1^2+dist2Bnd_c^2-2*cos(planner.rayAngle)*dist2Bnd_1*dist2Bnd_c);
                        hitAngle_2 = real(acos(( dist2Bnd_c^2 + rayDist_2^2-dist2Bnd_1^2)/(2*dist2Bnd_c*rayDist_2)));
                        
                        hp1 = planner.qcurr + dist2Bnd_c*[cos(planner.angle) sin(planner.angle)]; %1st ray hitpoint
                        hp2 = hp1 + 2*rayDist_2*[cos(planner.angle - planner.yDirect*(-hitAngle_2+pi)) sin(planner.angle-planner.yDirect*(-hitAngle_2+pi))]; %2nd ray hitpoint
                        
                        planner.angle = mod(planner.angle-planner.yDirect*pi ,2*pi);
                        dest = hp2 + planner.safeDist*[cos(planner.angle) sin(planner.angle)];
                        x = dest(1);
                        y = dest(2);
                    end
                end
            case -1 %Left
                %First the robot has to go to last critical point because the shape of the obstacle describer graph
                %will be open,unclosed. After coming to critical point it
                %has to continue scanning from the opposite direction.
                if planner.firstMove == 1
                    dest = planner.critPoint(end,:);
                    x = dest(1);
                    y = dest(2);
                    planner.firstMove = 0;
                    planner.secondMove = 1;
                    return;
                elseif planner.secondMove == 1
                    % If it is convex starting point (Lagging ray is going to infinity) ==> 1 
                    % If it is convex ending point (Leading ray is going to infinity) ==> 2
                    % If it is convex critical point ==> 3
                    % If it is concav critical point ==> 4
                    if planner.graphMap(end) == -1
                        planner.angle = 3*pi/1.9978;
                        rayAngle = planner.rayAngle;
                        while 1
                            dist = read_sensor(planner.angle + rayAngle,planner.qcurr);
                            if(dist == infinity)
                                rayAngle = rayAngle - planner.rayAngle/50;
                            else
                                break;
                            end
                        end
                        rayDist = sqrt(dist^2+dist2Bnd_c^2-2*cos(rayAngle)*dist*dist2Bnd_c);
                        hitAngle = acos(( dist2Bnd_c^2 + rayDist^2-dist^2)/(2*dist2Bnd_c*rayDist));
                        hp1 = planner.qcurr + dist2Bnd_c*[cos(planner.angle) sin(planner.angle)]; %1st ray hitpoint
                        hp2 = hp1 + 2*rayDist*[cos(planner.angle + pi - hitAngle) sin(planner.angle + pi - hitAngle)]; %2nd ray hitpoint
                        planner.angle = planner.angle - pi/2 + hitAngle;
                        dest = hp2 + dist/3*[cos(planner.angle) sin(planner.angle)];
                        planner.angle = 3*pi/1.9978;
                    else
                        planner.angle = pi/1.9978;
                        rayAngle = planner.rayAngle;
                        while 1
                            dist = read_sensor(planner.angle - rayAngle,planner.qcurr);
                            if(dist == infinity)
                                rayAngle = rayAngle - planner.rayAngle/50;
                            else
                                break;
                            end
                        end
                        rayDist = sqrt(dist^2+dist2Bnd_c^2-2*cos(rayAngle)*dist*dist2Bnd_c);
                        hitAngle = acos(( dist2Bnd_c^2 + rayDist^2-dist^2)/(2*dist2Bnd_c*rayDist));
                        hp1 = planner.qcurr + dist2Bnd_c*[cos(planner.angle) sin(planner.angle)]; %1st ray hitpoint
                        hp2 = hp1 + rayDist*[cos(planner.angle - pi + hitAngle) sin(planner.angle - pi + hitAngle)]; %2nd ray hitpoint
                        planner.angle = planner.angle - pi + hitAngle + atan2(dist,rayDist);
                        roboDist = sqrt(dist^2 + rayDist^2);
                        dest = hp2 + roboDist*[cos(planner.angle) sin(planner.angle)];
                        planner.angle = pi/1.9978;
                    end
                    x = dest(1);
                    y = dest(2);
                    planner.secondMove = 0;
                    planner.thirdMove = 1;
                    return;
                elseif planner.thirdMove == 1
                    % Do boundary following in here
                    
                else % To continue normal scanning motion 
                    
                    if dist2Bnd_1 ~= infinity && dist2Bnd_2 ~= infinity
                        rayDist_1 = sqrt(dist2Bnd_1^2+dist2Bnd_c^2-2*cos(planner.rayAngle)*dist2Bnd_1*dist2Bnd_c);
                        hitAngle_1 = acos(( dist2Bnd_c^2 + rayDist_1^2-dist2Bnd_1^2)/(2*dist2Bnd_c*rayDist_1));
                        
                        rayDist_2 = sqrt(dist2Bnd_2^2+dist2Bnd_c^2-2*cos(planner.rayAngle)*dist2Bnd_2*dist2Bnd_c);
                        hitAngle_2 = acos(( dist2Bnd_c^2 + rayDist_2^2-dist2Bnd_2^2)/(2*dist2Bnd_c*rayDist_2));
                        
                        if  abs(pi - (hitAngle_2+hitAngle_1)) > planner.minStepAngle
                            planner.critPoint = [planner.critPoint;planner.qcurr];
                            if pi < (hitAngle_2+hitAngle_1) %If the critical point is Convex
                                planner.graphMap = [planner.graphMap;1];
                            else %If the critical point is Concave
                                planner.graphMap = [planner.graphMap;2];
                            end
                            
                        end
                        hitAngle_2 = mod(hitAngle_2*planner.yDirect,2*pi);
                        hp1 = planner.qcurr + dist2Bnd_c*[cos(planner.angle) sin(planner.angle)]; %1st ray hitpoint
                        hp2 = hp1 + rayDist_2*[cos( planner.angle-planner.yDirect*(pi-hitAngle_2)) sin( planner.angle-planner.yDirect*(pi-hitAngle_2))]; %2nd ray hitpoint
                        planner.angle = mod(-planner.angle,2*pi);
                        dest = hp2 + (sensor_range - 2*planner.safeDist)*[cos(planner.angle) sin(planner.angle)];
                        x = dest(1);
                        y = dest(2);
                        
                        %If 2nd and 3rd ray distance measurement is infinity decrease the
                        %angle between the rays and find the closest ray
                    else
                        %If at least one of the ray is measuring infinity that
                        %means there will be a critical point
                        planner.critPoint = [planner.critPoint;planner.qcurr];
                        if dist2Bnd_1 == infinity
                            planner.graphMap = [planner.graphMap;planner.yDirect];
                            rayDist_1 = sqrt(dist2Bnd_2^2+dist2Bnd_c^2-2*cos(planner.rayAngle)*dist2Bnd_2*dist2Bnd_c);
                            hitAngle_1 = real(acos(( dist2Bnd_c^2 + rayDist_1^2-dist2Bnd_2^2)/(2*dist2Bnd_c*rayDist_1)));
                            
                            hp1 = planner.qcurr + dist2Bnd_c*[cos(planner.angle) sin(planner.angle)]; %1st ray hitpoint
                            hp2 = hp1 + rayDist_1*[cos(planner.angle-planner.yDirect*(-hitAngle_1+pi)) sin(planner.angle-planner.yDirect*(-hitAngle_1+pi))]; %2nd ray hitpoint
                            
                            planner.angle = mod(-planner.angle,2*pi);
                            dest = hp2 + 2*[cos(planner.angle) sin(planner.angle)];
                            x = dest(1);
                            y = dest(2);
                        elseif dist2Bnd_2 == infinity
                            planner.graphMap = [planner.graphMap;planner.yDirect];
                            rayDist_2 = sqrt(dist2Bnd_1^2+dist2Bnd_c^2-2*cos(planner.rayAngle)*dist2Bnd_1*dist2Bnd_c);
                            hitAngle_2 = real(acos(( dist2Bnd_c^2 + rayDist_2^2-dist2Bnd_1^2)/(2*dist2Bnd_c*rayDist_2)));
                            
                            hp1 = planner.qcurr + dist2Bnd_c*[cos(planner.angle) sin(planner.angle)]; %1st ray hitpoint
                            hp2 = hp1 + 2*rayDist_2*[cos(planner.angle - planner.yDirect*(-hitAngle_2+pi)) sin(planner.angle-planner.yDirect*(-hitAngle_2+pi))]; %2nd ray hitpoint
                            
                            planner.angle = mod(-planner.angle,2*pi);
                            dest = hp2 + planner.safeDist*[cos(planner.angle) sin(planner.angle)];
                            x = dest(1);
                            y = dest(2);
                        end
                    end
                end
        end
        planner.state = 0;
        planner.yDirect = -1*planner.yDirect;
        planner.angle = mod(planner.yDirect*pi/1.9978,2*pi);
end 
end