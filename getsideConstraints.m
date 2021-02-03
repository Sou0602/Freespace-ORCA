function [c,ceq] = getsideConstraints(agents,i, u, dt)
c = [];
ceq = [];
freespace = agents(i).freespace;
if agents(i).side == 0
    velocityspace = agents(i).rshape;
elseif agents(i).side == 1
    velocityspace = agents(i).lshape;
else
    velocityspace = freespace;
end

vel = [u(1) + agents(i).position(1) , u(2) + agents(i).position(2)];
if ~isempty(agents(i).obs)
for k = 1:length(agents(i).obs)
   
    xvo = agents(i).ospace(k).Vertices(:,1);
    yvo = agents(i).ospace(k).Vertices(:,2);
    
    vo_apex = [xvo(1) yvo(1)];
    pint1 = [xvo(2) yvo(2)];
    pint2 = [xvo(3) yvo(3)];
    
    leg1 = (pint1 - vo_apex)/norm((pint1 - vo_apex));
    leg2 = (pint2 - vo_apex)/norm((pint2 - vo_apex));
    
    norm1 = [leg1(2) -leg1(1)];
    norm2 = [leg2(2) -leg2(1)];
    
    c(end+1) = -1 * (dot(vel-vo_apex,norm1));
    c(end+1) = -1 * (dot(vel-vo_apex,norm2));
    
end
end
velocity = agents(i).velocity;
vel_norm = velocity / norm(velocity);
norm_vel = [vel_norm(2) -vel_norm(1)];

if agents(i).side == 0
    c(end+1) =  -1*(dot(vel-agents(i).position,-norm_vel));
elseif agents(i).side == 1
    c(end+1) =  -1*(dot(vel-agents(i).position,norm_vel));
end

end