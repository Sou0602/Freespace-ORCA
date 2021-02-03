agents = [
    addAgent('1', [-5,-5], [0 0], [5,5],1),
    addAgent('2', [5 -5],   [0 0 ], [-5 5],1),
    addAgent('3', [-5 5],  [0 0 ], [5 -5],1),
    addAgent('4', [5,5],  [0 0], [-5 ,-5],1),
   % addAgent('3', [-5 5],  [0 0 ], [5 -5],1),
    ];

dt = 0.1;
counter = 0;
axisLimits = [-12 12 -12 12]/2;
maxIterations = 500;

while counter < maxIterations
    
    for i = 1:length(agents)
     if(agents(i).gflag ~= 1) 
        obstacles = [];
        for j = 1:length(agents)
            if i ~= j
                if inSensorRange(agents(i), agents(j))
                    obstacles = [obstacles; agents(j).name];
                end
            end
        end
        agents(i).obs = obstacles;
     end
    end
    
sides = [];

    for m = 1 :length(agents)
      if(agents(m).gflag ~= 1) 
    agents(m).path = [agents(m).path ; agents(m).position];
      if isempty(agents(m).obs)
      agents(m).side = 2;
     
      else
       px = agents(m).position(1);
       py = agents(m).position(2);
       vm = agents(m).vmax;
       agents(m).sqpoly = polyshape([px-vm,px-vm,px+vm,px+vm],[py-vm,py+vm,py+vm,py-vm]);
       agents(m).ospace = [];
       for n = 1 : length(agents(m).obs)
           xint1 = 0;
           yint1 = 0;
           xint2 = 0;
           yint2 = 0;
           oname = str2double(agents(m).obs(n));
           obstacle = agents(oname);
           [x1,y1,x2,y2]=CC(agents(m).position, obstacle.position, agents(m).radius + obstacle.radius, agents(m).radius);
           [x1o,y1o,x2o,y2o] = VO(x1,y1,x2,y2,obstacle.velocity);
           [xint1,yint1] = getintersection(x1o,y1o,agents(m).sqpoly);
           [xint2,yint2] = getintersection(x2o,y2o,agents(m).sqpoly);
           if n == 1
           agents(m).vospace = constructpoly(agents(m).position + obstacle.velocity,xint1,yint1,xint2,yint2);
           agents(m).ospace  = [agents(m).ospace;agents(m).vospace];
           elseif n > 1
           ospace = constructpoly(agents(m).position + obstacle.velocity,xint1,yint1,xint2,yint2);
           agents(m).ospace = [agents(m).ospace;ospace];
           agents(m).vospace = union(agents(m).vospace,ospace);
           end
           obstacle = [];
       end
 
%%%%%%%%%
agents(m).freespace = agents(m).sqpoly;
agents(m).freespace = subtract(agents(m).freespace,agents(m).vospace);


[agents(m).rshape , agents(m).lshape] = dividevshape(agents(m));
 [xc,yc] = centroid(agents(m).rshape);
 agents(m).rcentroid = [xc,yc];
 [xc,yc] = centroid(agents(m).lshape);
 agents(m).lcentroid = [xc,yc];
      end
    else
    agents(m).freespace = agents(m).sqpoly;
    agents(m).side = 2;
    end
    end
    
sides = allotment_weights(agents);

    for i = 1 : length(agents)
        if ~isempty(sides)
        agents(i).side = sides(i);
        end
    end
    %}
    for i = 1 : length(agents)
        if agents(i).gflag ~= 1
        if agents(i).side == 2
            agents(i).velocity = ((agents(i).goal - agents(i).position) / norm(agents(i).goal - agents(i).position)) * agents(i).vmax;
        else
        agents(i).velocity = getControls(agents,i,dt);
        end
        agents(i).position = agents(i).position + agents(i).velocity * dt;
        goaldist = norm(agents(i).position - agents(i).goal);
        if goaldist < agents(i).radius/2
            agents(i).gflag = 1;
        end
        else
            agents(i).gflag = 1;
            agents(i).velocity = [0,0];
            agents(i).position = agents(i).position;
        end
    end
   
    plotSimulation(agents, counter, dt, axisLimits, true);
    
    sumg = 0;
    for k = length(agents)
        sumg = sumg + agents(k).gflag;
    end
    if sumg == length(agents)
            break;
    end
    counter = counter + 1;
end


