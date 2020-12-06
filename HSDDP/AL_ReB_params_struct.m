classdef AL_ReB_params_struct
properties
    % lambda should have the same size as the eq_constr
    % delta should have the same size as the ineq_constr
    % all others are scalars
    sigma           % penalty coefficient
    lambda          % Lagrange multiplier
    delta           % ReB relaxation param
    eps_ReB         % ReB weighting param
    eps_smooth = 1  % relaxation update param
end
end