function cost = calc_cost(X, Xref, U, p)
%% Calculate Cost
% X: State series (13 x N)
% Xref: reference over a horizon (13 x N)
% U: control input (12 X N-1)
% p: parameters (Q_weight, R_weight)

Q = p.Q_weight;
R = p.R_weight;

state_error = X - Xref;
state_cost = 0.5 * sum(dot(state_error, Q * state_error, 1));
control_cost = 0.5 * sum(dot(U, R * U, 1));

cost = state_cost + control_cost;
end
