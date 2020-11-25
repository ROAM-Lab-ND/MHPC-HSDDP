classdef struct_AL_ReB_params
properties
    % lambda should have the same size as the eq_constr
    % delta should have the same size as the ineq_constr
    % all others are scalars
    sigma       % penalty coefficient
    lambda      % Lagrange multiplier
    beta_sigma  % penalty update param
    delta       % ReB relaxation param
    eps_ReB     % ReB weighting param
    beta_delta  % relaxation update param
end
end