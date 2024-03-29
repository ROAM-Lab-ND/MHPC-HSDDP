function [euler] = quatern2euler(q)
%QUATERN2EULER Converts a quaternion orientation to ZYX Euler angles
%
%   q = quatern2euler(q)
%
%   Converts a quaternion orientation to ZYX Euler angles where phi is a
%   rotation around X, theta around Y and psi around Z.
%
%   For more information see:
%   http://www.x-io.co.uk/node/8#quaternions
%
%	Date          Author          Notes
%	27/09/2011    SOH Madgwick    Initial release
%     R = zeros(3,3);
%     R(1,1,:) = 2.*q(:,1).^2-1+2.*q(:,2).^2;
%     R(2,1,:) = 2.*(q(:,2).*q(:,3)-q(:,1).*q(:,4));
%     R(3,1,:) = 2.*(q(:,2).*q(:,4)+q(:,1).*q(:,3));
%     R(3,2,:) = 2.*(q(:,3).*q(:,4)-q(:,1).*q(:,2));
%     R(3,3,:) = 2.*q(:,1).^2-1+2.*q(:,4).^2;
% 
%     phi = atan2(R(3,2,:), R(3,3,:) );
%     theta = -atan(R(3,1,:) ./ sqrt(1-R(3,1,:).^2) );
%     psi = atan2(R(2,1,:), R(1,1,:) );
% 
%     euler = [phi(1,:)'; theta(1,:)'; psi(1,:)'];
q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);

euler = [atan2(2*q2*q3+2*q0*q1,q3^2-q2^2-q1^2+q0^2);
       -asin(2*q1*q3-2*q0*q2);
       atan2(2*q1*q2+2*q0*q3,q1^2+q0^2-q3^2-q2^2)];
   
end

