function print_solver_summary(step, total_steps, final_cost, iter, solver_log)
% Print constrained iLQR iteration summary.

fprintf([ ...
    'step %d / %d:\n' ...
    '  J_final     = %.6f\n' ...
    '  iteration   = %d\n'], ...
    step, total_steps, final_cost, iter);

%% Active-set trace
fprintf('  [ActiveSet] : ');

if isempty(solver_log.active_count)
    fprintf('none');
else
    fprintf('%d', solver_log.active_count(1));

    for k = 2:numel(solver_log.active_count)
        fprintf(' --(+%d/-%d)--> %d', ...
            solver_log.active_added(k), ...
            solver_log.active_removed(k), ...
            solver_log.active_count(k));
    end
end

fprintf('\n');

%% Cost and decision trace
fprintf('  [J]         : ');

if isempty(solver_log.accepted)
    fprintf('none');
else
    for k = 1:numel(solver_log.accepted)
        if k > 1
            fprintf(' | ');
        end

        if solver_log.accepted(k)
            status = 'ACCEPT';
        else
            status = 'REJECT';
        end

        fprintf([ ...
            '%.4f -> %.4f ' ...
            '[%s, red=%+.2e]'], ...
            solver_log.J_before(k), ...
            solver_log.J_candidate(k), ...
            status, ...
            solver_log.reduction(k));
    end
end

fprintf('\n\n');

end