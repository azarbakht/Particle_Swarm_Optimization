% Code repository: http://www.mathworks.com/matlabcentral/fx_files/11559/1/content/html/PSO.html


% swarm(index, [1=location, 2=velocity, 3=best position, 4=best value], [1=x, 2=y % components or the value component])

iterations = 100;
inertia = 1.4;
betaInertia = 0.5;
correctionFactor1 = 1.8;
correctionFactor2 = 1.9;
swarmSize = 100;
maxVelocity = 10;

% ---- initial swarm position -----
index = 1;
for i = 1 : 10
    for j = 1 : 10
        swarm(index, 1, 1) = i;
        swarm(index, 1, 2) = j;
        index = index + 1;
    end
end

swarm(:, 4, 1) = Inf;           % best minimum value so far; positive infinity
swarm(:, 2, :) = 0;             % initial velocity

% iterations
for iter = 1 : iterations
    
    % evaluating positions
    for i = 1 : swarmSize
        swarm(i, 1, 1) = swarm(i, 1, 1) + swarm(i, 2, 1)/1.3;     %update x position
        swarm(i, 1, 2) = swarm(i, 1, 2) + swarm(i, 2, 2)/1.3;     %update y position
        x = swarm(i, 1, 1);
        y = swarm(i, 1, 2);
        
        val = EvaluateParticle(x,y);
        
        if val < swarm(i, 4, 1)                 % if new position is better
            swarm(i, 3, 1) = swarm(i, 1, 1);    % update best x,
            swarm(i, 3, 2) = swarm(i, 1, 2);    % best y postions
            swarm(i, 4, 1) = val;               % and best value
        end
    end
    % betaInertia = rand;
    [temp, gbest] = min(swarm(:, 4, 1));        % global best position
    
    % updating velocities
    for i = 1 : swarmSize
        swarm(i, 2, 1) = betaInertia*inertia*swarm(i, 2, 1) + correctionFactor1*rand*(swarm(i, 3, 1) - swarm(i, 1, 1)) + correctionFactor2*rand*(swarm(gbest, 3, 1) - swarm(i, 1, 1));   %x velocity component
        swarm(i, 2, 2) = betaInertia*inertia*swarm(i, 2, 2) + correctionFactor1*rand*(swarm(i, 3, 2) - swarm(i, 1, 2)) + correctionFactor2*rand*(swarm(gbest, 3, 2) - swarm(i, 1, 2));   %y velocity component
        
        if swarm(i, 2, 1) > maxVelocity
            swarm(i, 2, 1) = maxVelocity;
        end
                
        if swarm(i, 2, 1) < -maxVelocity
            swarm(i, 2, 1) = -maxVelocity;
        end
        
        if swarm(i, 2, 2) > maxVelocity
            swarm(i, 2, 2) = maxVelocity;
        end  

        
        if swarm(i, 2, 2) < -maxVelocity
            swarm(i, 2, 2) = -maxVelocity;
        end
    end
    
    
    % Plot
    clf
    plot(swarm(:, 1, 1), swarm(:, 1, 2), 'x')   % drawing swarm movements
    axis([-10 10 -10 10]);
    pause(.2)
end

x, y, val





