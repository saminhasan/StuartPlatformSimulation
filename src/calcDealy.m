function delay =  calcDealy(order, tauK)
    syms t tau k
    % symbolic y(t)
    y_sym = t- order*tau + tau*exp(-t/tau)* symsum( (order - k)*(t/tau)^k / factorial(k), k, 0, order-1 );
    % pretty(y_sym)
    % 4) Solve symbolically y(t)=1 for general n,tau
    % sol_general = solve(y_sym == 1, t);
    % → returns a RootOf(...) object, since no elementary closed‑form exists.
    %solve numerically
    y_num = subs(y_sym, tau, tauK);
    initial_guess = order*0.01 + 1;    % ≈ n*tau + 1
    t_sol = vpasolve(y_num == 1, t, initial_guess);
    % fprintf('Numeric solution : ', double(t_sol));
    delay = double(t_sol-1);
end