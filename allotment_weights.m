function sides = allotment_weights(agents)

% minimum freespace gets priority
% weights shift from surrounding agents to the agent
% side with maximum weight has least priority
% also think about the case where freespaces intersect

for m = 1:length(agents)
    if(agents(m).gflag~=1)
% Give weights to all the surrounding obstacle free space sides
% Whichever side of the freespace is closest to the agent_position gets the
% weight
if ~isempty(agents(m).obs)
for j = 1:length(agents(m).obs)
    %{
    rdist = [norm(agents(m).rcentroid - agents(m).obs(j).rcentroid),norm(agents(m).rcentroid - agents(m).obs(j).lcentroid)];
    ldist = [norm(agents(m).rcentroid - agents(m).obs(j).rcentroid),norm(agents(m).rcentroid - agents(m).obs(j).lcentroid)];
    dist = [rdist,ldist];
    %}
    k = str2double(agents(m).obs(j));
    dist = [norm(agents(m).position - agents(k).rcentroid),norm(agents(m).position - agents(k).lcentroid)];
    [~,min_ind] = min(dist);
%let default side is right and equal to 0, left is equal to 1 
% Assign rshape weights
% Assign lshape weights
if dist(1) ~= dist(2)
   if min_ind == 1
       agents(k).rweights = agents(k).rweights + 1;
   elseif min_ind == 2
       agents(k).lweights = agents(k).lweights + 1;
   end
else
    agents(k).rweights = agents(k).rweights + 0.5;
    agents(k).lweights = agents(k).lweights + 0.5;
end

% Complete giving weights
end

else
    agents(m).lweights = -1;
    agents(m).rweights = -1;
end
    end
end
sides = [];
% Choosing the choose the side with minimum weight or max-free space(if same weight then max)
for m = 1:length(agents)
    weights = [agents(m).rweights,agents(m).lweights];
    [~,sidemin] = min(weights);
    if weights(1) < 0 || agents(m).gflag == 1
        agents(m).side = 2;
    else
    if agents(m).rweights ~= agents(m).lweights
        if sidemin == 1
            agents(m).side = 0;
        else
            agents(m).side = 1;
        end
    else
        agents(m).side  = 0;
    end
    end
    sides = [sides,agents(m).side];
end

end