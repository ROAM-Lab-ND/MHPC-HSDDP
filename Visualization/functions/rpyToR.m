function [ R ] = rpyToR( rpy )
% rpyToR takes earth-fixed sequenial roll-pitch-yaw euler angles to a
% rotation matrix
%   [R] = rpyToR(rpy)
%   note: earth-fixed roll-pitch-yaw is the same as body-fixed
%   yaw-pitch-roll sequence 

    R = quatToR(rpyToQuat(rpy));
end

