function WB_terminal_constr_support(WBMC2D)       
    q = sym('q', [7,1], 'real');
    qd = sym('qd', [7,1], 'real');
    x = [q; qd];
    
    ground = -0.404;
    Pos_ffoot = WBMC2D.getPosition(q, 3, [0,-WBMC2D.kneeLinkLength]');
    h_FL1 = Pos_ffoot(2) - ground;
    hx_FL1 = jacobian(h_FL1, x);
    hxx_FL1 = hessian(h_FL1, x);
    
    Pos_bfoot = WBMC2D.getPosition(q, 5, [0,-WBMC2D.kneeLinkLength]');
    h_FL2 = Pos_bfoot(2) - ground;
    hx_FL2 = jacobian(h_FL2, x);
    hxx_FL2 = hessian(h_FL2, x);
    
    matlabFunction(h_FL1, hx_FL1, hxx_FL1, 'file', 'CostAndConstraint/WB_FL1_terminal_constr', 'vars', {x});
    matlabFunction(h_FL2, hx_FL2, hxx_FL2, 'file', 'CostAndConstraint/WB_FL2_terminal_constr', 'vars', {x});
    
    fprintf('Terminal constrait functions generated successfully!\n');
end