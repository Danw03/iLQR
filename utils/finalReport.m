function finalReport(history)
%% FINALREPORT Print solver summary.

solver_time = history.solver_time;
solver_cost = history.solver_cost;
solver_iter = history.solver_iter;

steps = numel(solver_time);

total_solver_time = sum(solver_time);
mean_solver_time = mean(solver_time);
max_solver_time = max(solver_time);

solver_frequency = steps / total_solver_time;

total_cost = sum(solver_cost);
mean_cost = mean(solver_cost);

mean_iteration = mean(solver_iter);
max_iteration = max(solver_iter);

fprintf('==================================================\n\n');

fprintf('  Total solver time    : %.4f s\n', total_solver_time);
fprintf('  Mean solver time     : %.6f s\n', mean_solver_time);
fprintf('  Maximum solver time  : %.6f s\n', max_solver_time);
fprintf('  Solver frequency     : %.2f Hz\n', solver_frequency);

fprintf('\n');

fprintf('  Initial cost         : %.6f\n', solver_cost(1));
fprintf('  Final cost           : %.6f\n', solver_cost(end));
fprintf('  Total cost           : %.6f\n', total_cost);
fprintf('  Mean cost            : %.6f\n', mean_cost);

fprintf('\n');

fprintf('  Mean iteration       : %.2f\n', mean_iteration);
fprintf('  Maximum iteration    : %d\n', max_iteration);

fprintf('\n==================================================\n\n');

end