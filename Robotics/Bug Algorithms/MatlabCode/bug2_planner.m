function [x,y] = bug2_planner( qstart, qgoal)

global sensor_range infinity planner;

% A simple hack which blindly goes towards the goal
% samples = norm( qgoal - qstart ) / (sensor_range / 10);
% range = linspace(0,1,samples);
%
% x = qstart(1) + range*(qgoal(1) - qstart(1));
% y = qstart(2) + range*(qgoal(2) - qstart(2));


if planner.state ~= 0
    dist = [];
    for i=1:size(planner.mMap,1)
        dist(i) = norm(planner.mMap(i,:) - planner.qcurr);
    end
    if (min(dist) > 0.3)%(sensor_range - 2*planner.safeDist)*tan(planner.rayAngle)*2)
        currLine = mod(atan2(qgoal(2)-planner.qcurr(2), qgoal(1)-planner.qcurr(1))+2*pi,2*pi);
        if planner.boundDir == 0
            if ((currLine - planner.rayAngle/5) < mod(planner.mLine,pi)  && ...
                    (mod(planner.mLine,pi) < currLine)) || ...
                    ((currLine - planner.rayAngle/5) < planner.mLine  && ...
                    planner.mLine < currLine)
                planner.state = 0;
                planner.mMap = [planner.mMap;planner.qcurr];
            end
        else
            if ((currLine ) < mod(planner.mLine,pi)  && ...
                    (mod(planner.mLine,pi) < currLine + planner.rayAngle/5)) || ...
                    ((currLine) < planner.mLine  && ...
                    planner.mLine < currLine+ planner.rayAngle/5)
                planner.state = 0;
                planner.mMap = [planner.mMap;planner.qcurr];
            end
        end
    end
end
switch(planner.state)
    case 0
        % Motion-to-Goal state
        planner.angle = atan2(qgoal(2)-planner.qcurr(2),qgoal(1)-planner.qcurr(1))+2*pi;
        distance = read_sensor(planner.angle,planner.qcurr);
        velPro = norm(planner.qcurr - qgoal)/norm(qstart-qgoal);
        if velPro <= 1e-4 %to check the robot is finished or not
            planner.finished = 1;
            x = planner.qcurr(1);
            y = planner.qcurr(2);
            return;
        end
        if(distance == infinity)
            %velocity proportion to adjust the speed according to distance between robot and the goal
            
            dest =  planner.qcurr +sensor_range*velPro/10*[cos(planner.angle) sin(planner.angle)];
            x = dest(1);
            y = dest(2);
        else
            dest =  planner.qcurr + (distance -planner.safeDist)*[cos(planner.angle) sin(planner.angle)];
            if ((distance <= planner.safeDist + planner.minStep))
                planner.state = 1;
                planner.boundStart = 1;
                x = planner.qcurr(1);
                y = planner.qcurr(2);
                planner.mMap = [planner.mMap;[x y]];
                return;
            end
            x = dest(1);
            y = dest(2);
        end
    case 1
        % Boundary-Following state
        
        if planner.boundStart == 1
            planner.angle = atan2(qgoal(2)-planner.qcurr(2),qgoal(1)-planner.qcurr(1))+2*pi;
            rayAngle =  pi/8 ;%planner.rayAngle/30 ; %To decide which direction has to be choosen
            distRay1 = read_sensor(planner.angle+rayAngle, planner.qcurr);
            distRay2 = read_sensor(planner.angle-rayAngle, planner.qcurr);
            planner.obstacleMap = [];
            
            if distRay2>distRay1
                planner.boundDir = 0; %right direction
            else
                planner.boundDir = 1; %left direction
            end
            planner.boundStart = 0;
        end
        switch (planner.boundDir)
            case 0
                distRay1 = read_sensor(planner.angle, planner.qcurr);
                distRay2 = read_sensor(planner.angle-planner.rayAngle, planner.qcurr);
                
                if distRay1 <= planner.safeDist - planner.minStep
                    dest = planner.qcurr - (2*planner.safeDist - distRay1)*[cos(planner.angle) sin(planner.angle)];
                    x = dest(1);
                    y = dest(2);
                    return;
                end
                
                
                %If 2nd ray distance measurement is infinity decrease the
                %angle between the rays and find the closest ray
                if distRay2 ~= infinity
                    rayDist = sqrt(distRay2^2+distRay1^2-2*cos(planner.rayAngle)*distRay2*distRay1);
                    hitAngle = acos(( distRay1^2 + rayDist^2-distRay2^2)/(2*distRay1*rayDist));
                    hp1 = planner.qcurr + distRay1*[cos(planner.angle) sin(planner.angle)]; %1st ray hitpoint
                    hp2 = hp1 + rayDist*[cos(hitAngle+planner.angle-pi) sin(hitAngle+planner.angle-pi)]; %2nd ray hitpoint
                    planner.angle = mod(planner.angle + hitAngle - pi/2,2*pi);
                    dest = hp2 + (sensor_range - 2*planner.safeDist)*[cos(planner.angle-pi) sin(planner.angle-pi)];
                    x = dest(1);
                    y = dest(2);
                else
                    rayAngle = planner.rayAngle;
                    while 1
                        dist = read_sensor(planner.angle - rayAngle,planner.qcurr);
                        if(dist == infinity)
                            rayAngle = rayAngle - planner.rayAngle/50;
                        else
                            break;
                        end
                    end
                    
                    rayDist = sqrt(dist^2+distRay1^2-2*cos(rayAngle)*dist*distRay1);
                    hitAngle = acos(( distRay1^2 + rayDist^2-dist^2)/(2*distRay1*rayDist));
                    hp1 = planner.qcurr + distRay1*[cos(planner.angle) sin(planner.angle)]; %1st ray hitpoint
                    hp2 = hp1 + rayDist*[cos(hitAngle+planner.angle-pi) sin(hitAngle+planner.angle-pi)]; %2nd ray hitpoint
                    planner.angle = planner.angle -rayAngle;
                    roboDist = (sensor_range - 2*planner.safeDist)/cos(rayAngle);
                    dest = hp2 + roboDist*[cos(planner.angle) sin(planner.angle)];
                    planner.angle = mod(planner.angle - 2*rayAngle +pi,2*pi);
                    x = dest(1);
                    y = dest(2);
                    
                end
                
            case 1
                distRay1 = read_sensor(planner.angle, planner.qcurr);
                distRay2 = read_sensor(planner.angle+planner.rayAngle, planner.qcurr);
                
                if distRay1 <= planner.safeDist - planner.minStep
                    dest = planner.qcurr - (2*planner.safeDist - distRay1)*[cos(planner.angle) sin(planner.angle)];
                    x = dest(1);
                    y = dest(2);
                    return;
                end
                
                %If 2nd ray distance measurement is infinity decrease the
                %angle between the rays and find the closest ray
                if distRay2 ~= infinity
                    rayDist = sqrt(distRay2^2+distRay1^2-2*cos(planner.rayAngle)*distRay2*distRay1);
                    hitAngle = acos(( distRay1^2 + rayDist^2-distRay2^2)/(2*distRay1*rayDist));
                    hp1 = planner.qcurr + distRay1*[cos(planner.angle) sin(planner.angle)]; %1st ray hitpoint
                    hp2 = hp1 + rayDist*[cos(planner.angle+pi-hitAngle) sin(planner.angle+pi-hitAngle)]; %2nd ray hitpoint
                    planner.angle = mod(planner.angle - hitAngle + pi/2,2*pi);
                    dest = hp2 + (sensor_range - 2*planner.safeDist)*[cos(planner.angle+pi) sin(planner.angle+pi)];
                    x = dest(1);
                    y = dest(2);
                else
                    rayAngle = planner.rayAngle;
                    while 1
                        dist = read_sensor(planner.angle + rayAngle,planner.qcurr);
                        if(dist == infinity)
                            rayAngle = rayAngle - planner.rayAngle/50;
                        else
                            break;
                        end
                    end
                    
                    rayDist = sqrt(dist^2+distRay1^2-2*cos(rayAngle)*dist*distRay1);
                    hitAngle = acos(( distRay1^2 + rayDist^2-dist^2)/(2*distRay1*rayDist));
                    hp1 = planner.qcurr + distRay1*[cos(planner.angle) sin(planner.angle)]; %1st ray hitpoint
                    hp2 = hp1 + rayDist*[cos(planner.angle+pi-hitAngle) sin(planner.angle+pi-hitAngle)]; %2nd ray hitpoint
                    planner.angle = planner.angle +rayAngle;
                    roboDist = (sensor_range - 2*planner.safeDist)/cos(rayAngle);
                    dest = hp2 + roboDist*[cos(planner.angle) sin(planner.angle)];
                    planner.angle = mod(planner.angle + 2*rayAngle+pi,2*pi);
                    x = dest(1);
                    y = dest(2);
                    
                end
        end
        
        
end

end

