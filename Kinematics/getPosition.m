function ICSPos = getPosition(q,linkidx, contactLoc)
% This function gets the position of a contact point w.r.t. ICS.
% ContactLoc is defined in local frame
% Link index convention: body 1, f hip 2, f knee 3, b hip 4, b knee 5, 

% % if linkidx = 6 (7), get front(back) foot position
% if nargin<3 && any(linkidx == [6, 7])
%     contactLoc = mc2D.kneeLoc;
%     if linkidx == 6
%         linkidx = 3;
%     elseif linkidx == 7
%         linkidx = 5;
%     end
% end

T  = getKinematics(q);
contactLoc_aug = [contactLoc(1), 0, contactLoc(3), 1]';
ICSPos = T{linkidx}*contactLoc_aug;

% remove y component and homogeneuous component
ICSPos([2,4],:) = [];

end