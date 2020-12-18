n1 = [1 0 0]';
ang = 18.5;
n2 = [sin(ang*pi/180) 0 cos(ang*pi/180)]';
ma = 10000;
vs = [];
inc = pi/60;
for roll = -pi:inc:pi
    for pitch = -pi/2+inc:inc:pi/2-inc
        for yaw = 0
            q = real(rpyToQuat([roll pitch yaw]));
            R = quatToR(q);
            
            v1 = R*n1;
            v2 = R*n2;
            v1(3) = 0;
            v2(3) = 0;
            
            if norm(v1) == 0 || norm(v2) ==0
                a = pi/2;
            else
                
                v1 = v1 / norm(v1);
                v2 = v2 / norm(v2);
            
            
                a = real(acos(dot(v1,v2)) - pi/2);
                ma = min(abs(a),ma);
                
            end
  
            v1 = R*n1;
            v2 = R*n2;
            vs = [vs ; roll pitch yaw a v1' v2'];
        end
    end
end
            
        

