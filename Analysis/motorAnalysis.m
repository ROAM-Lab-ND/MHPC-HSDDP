function energy = motorAnalysis(simTrajectory)
% @simTrajecory is a structure array of X, U, Y, t, Xopt and Kopt
time = [simTrajectory(:).t];
torque = [simTrajectory(:).U];
X = [simTrajectory(:).X];
speed = X(11:14, :);
energy = zeros(1,4);
for joint = 1:4
    [~, ~, energySeries] = get_motor_performance(time, torque(joint,:)/2, speed(joint,:));
    energy(joint) = energySeries(end);
end
end