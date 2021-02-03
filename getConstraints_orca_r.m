function [c, ceq] = getConstraints_orca_r(agents,i,control, dt)
%getConstraints - Description
%
% Syntax: [c ceq] = getConstraints(agent, obstacles)
%
    c = [];
    ceq = [];
    
    % Time horizon
    tau = 0.8*2 ;    
    for k = 1:length(agents(i).obs)
        
        oname = str2double(agents(i).obs(k));
        % Refer the paper for explanation on these terms
        vRel = agents(i).velocity - agents(oname).velocity;
        vAb = agents(i).velocity + agents(oname).velocity;
        pAb = ( -agents(i).position + agents(oname).position)/tau;
        
        % Finding pAb perpendicular
        r = 4*agents(i).radius/ tau  ;
        l = abs(sqrt(sum(pAb.^2) - r^2));

        pAblen = sqrt(sum(pAb.^2));
        m = [
            l -r;
            r  l
        ];

        qL = (pAb * m') * ( 1/ sum(pAb.^2));
        qR = (pAb * m ) * ( 1/ sum(pAb.^2));
        pAbL = [qL(2) -qL(1)];
        pAbR = [qR(2) -qR(1)];
     
        %collision cone check
        if ((pAblen)^2-(dot(pAb,vRel)^2)/sum(vRel.^2))<=r^2  

        
            d1=abs((qL(2)*vRel(1) - qL(1)*vRel(2))/sqrt(qL(1)^2+qL(2)^2));%distance to left side of the cone
            d2=abs((qR(2)*vRel(1) - qR(1)*vRel(2))/sqrt(qR(1)^2+qR(2)^2));%distance to the right side of the cone
            nM=[pAbL/sqrt(sum(pAbL.^2));pAbR/sqrt(sum(pAbR.^2))];
            
            
            u = d2*nM(2,:);
            nor = nM(2,:); 


            c(end+1)=-1*((control(1)-(agents(i).velocity(1)+.5*u(1)))*nor(1)+ (control(2)-(agents(i).velocity(2)+.5*u(2)))*nor(2));
 
        end

    end

  
end