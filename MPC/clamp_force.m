function u_clamped = clamp_force(u, contact, p)
% Clamp contact forces to the friction pyramid.
%
% Stance:
%   f_min <= fz <= f_Max
%   |fx| <= mu*fz
%   |fy| <= mu*fz
%
% Swing:
%   force = 0

u_clamped = zeros(12, 1);

for leg = 1:4
    idx = 3*(leg-1) + (1:3);

    if contact(leg) ~= 1
        % Swing-leg force
        u_clamped(idx) = 0;
        continue
    end

    % Clamp normal force first
    fz = min(max(u(idx(3)), p.f_min), p.f_Max);

    % Friction pyramid depends on the clamped fz
    friction_limit = p.mu*fz;

    fx = min(max(u(idx(1)), ...
        -friction_limit), friction_limit);

    fy = min(max(u(idx(2)), ...
        -friction_limit), friction_limit);

    u_clamped(idx) = [fx; fy; fz];
end

end