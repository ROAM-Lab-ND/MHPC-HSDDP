function [ quat_dot ] = omegaToQuatRates( omega,quat,innerOmega,kStab)
% omegaToQuatRates evaluates quaternion derivative based on angular vel.
%   [ quat_dot ] = omegaToQuatRates( omega,quat,innerOmega,kStab)
%   omega: angular velocity
%   quat:  current quaternion 
%          quat= [q0,q1,q2,q3] assumed to follow q0 = cos(angle / 2) while
%          [q1,q2,q3] = axis*sin(angle / 2) for angle/axis expression of R
%   innerOmega: boolean 1 means omega is s.t. \dot{R} = cross(omega)*R
%                       0 means omega is s.t. \dot{R} = R * cross(omega)
%   kStab: optimal stabilization term which keeps |quatdot| = 1 in
%          simulation

if nargin == 2
e0 = quat(1);
e1 = quat(2);
e2 = quat(3);
e3 = quat(4);

quat_dot = 1/2*[-e1 -e2 -e3;e0 e3 -e2; -e3 e0 e1; e2 -e1 e0]*omega;
elseif nargin>=3
    e0 = quat(1);
    ehat = quat(2:4);
    e0dot = -1/2*dot(omega,ehat);
    if innerOmega == 1
       ehatdot = 1/2*(e0*omega-cross(ehat)*omega);
    else
       ehatdot = 1/2*(e0*omega+cross(ehat)*omega); 
    end
    quat_dot = [e0dot ; ehatdot];    
end

if nargin==4
   quat_dot = quat_dot + kStab*norm(omega)*(1-norm(quat))*quat;
end
        
        
end

