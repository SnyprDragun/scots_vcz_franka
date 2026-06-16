% fastrack_nlink_3_solve.m (N_LINKS=3, same discretization as SCOTS)
% Grid: [-pi/2,pi/2] x [-2,2] with [3,5] points/joint to match SCOTS POS_PTS=3,VEL_PTS=5

clear; clc; close all;
N_LINKS = 3;

home_dir = getenv('HOME');
if isempty(home_dir), home_dir = getenv('USERPROFILE'); end
HELPEROC_PATH  = fullfile(home_dir, 'helperOC');
TOOLBOXLS_PATH = fullfile(home_dir, 'LS_HJ_Reachability');

if ~exist(fullfile(HELPEROC_PATH, 'dynSys', '@DynSys'), 'dir')
    error('helperOC not found. Edit HELPEROC_PATH.');
end
if ~exist(fullfile(TOOLBOXLS_PATH, 'Kernel', 'Grids', 'processGrid.m'), 'file')
    error('toolboxls not found. Edit TOOLBOXLS_PATH.');
end

addpath(genpath(HELPEROC_PATH));
addpath(genpath(TOOLBOXLS_PATH));

% Link parameters (match nlink_reach_example.cc exactly)
L_link   = [0.5, 0.4, 0.3, 0.25, 0.2, 0.15, 0.1];
m_link   = [2.0, 1.5, 1.0, 0.8, 0.6, 0.4, 0.2];
r_link   = [0.25, 0.2, 0.15, 0.125, 0.1, 0.075, 0.05];
I_eff    = [1.0, 0.6, 0.4, 0.3, 0.15, 0.1, 0.06];  % SCOTS values
tau_max  = [15, 8, 6, 5, 3, 2, 1.5];
g_acc    = 9.81;

ut_max   = tau_max(1:N_LINKS) ./ I_eff(1:N_LINKS);
up_max   = 0.05 * ones(1, N_LINKS);
d_max_nom = 0.1 * ones(1, N_LINKS);

% Grid: match SCOTS preset (POS_PTS=3, VEL_PTS=5, VEL_RANGE=2.0)
g_min = [-pi/2; -2];
g_max = [pi/2; 2];
N_grid = [3; 5];

fprintf('\n========== FaSTrack N=%d (SCOTS preset) ==========\n', N_LINKS);
TEB_nom = nan(1, N_LINKS);
t_nom   = nan(1, N_LINKS);

t_total_nom = 0;
for j = 1:N_LINKS
    fprintf('  Joint %d: ut_max=%.2f, up_max=%.3f, d_max=%.3f ... ', ...
            j, ut_max(j), up_max(j), d_max_nom(j));
    [V, dV, teb, elapsed] = solve_hj_joint(g_min, g_max, N_grid, ut_max(j), up_max(j), d_max_nom(j));
    TEB_nom(j) = teb;
    t_nom(j)   = elapsed;
    t_total_nom = t_total_nom + elapsed;
    fprintf('TEB=%.6f, %.1f s\n', teb, elapsed);
end

fprintf('\nNominal synthesis time total = %.2f s\n\n', t_total_nom);

% Mass sweep: 1g -> 1000g, 10 steps
N_MASS_STEPS = 10;
N_TRIALS = 1;
masses_g = linspace(1, 1000, N_MASS_STEPS);

fprintf('%10s | %10s | %10s\n', 'mass [g]', 'successes', 'success %%');
fprintf('%s\n', repmat('-', 1, 38));

rng(0);
for m_idx = 1:N_MASS_STEPS
    mass_kg = masses_g(m_idx) / 1000.0;
    L_total = sum(L_link(1:N_LINKS));
    d_extra  = mass_kg * g_acc * L_total ./ I_eff(1:N_LINKS);
    d_actual = d_max_nom + d_extra;

    successes = 0;
    for trial = 1:N_TRIALS
        ok = simulate_trial(N_LINKS, ut_max, up_max, d_actual);
        successes = successes + ok;
    end
    pct = 100.0 * successes / N_TRIALS;
    fprintf('%10.1f | %10d | %9.1f%%\n', masses_g(m_idx), successes, pct);
end

fprintf('\nDone.\n');

%-----
function ok = simulate_trial(NJ, ut_max, up_max, d_actual)
dt = 0.02;
horizon_steps = 200;
tol = 0.1;

if any(d_actual >= ut_max), ok = false; return; end

r = zeros(1, NJ);
v = zeros(1, NJ);
for k = 1:horizon_steps
    for j = 1:NJ
        u_t = -ut_max(j) * sign(v(j) + 1e-12);
        u_p = -up_max(j) * sign(r(j) + 1e-12);
        d   = d_actual(j) * (2*rand() - 1);
        r(j) = r(j) + dt * (v(j) - u_p);
        v(j) = v(j) + dt * (u_t + d);
    end
end
ok = all(abs(r) < tol);
end

function [V, dV, teb, elapsed] = solve_hj_joint(g_min, g_max, N, ut_max, up_max, dmax)
g = createGrid(g_min, g_max, N);
data0 = abs(g.xs{1});

dyn = RelDyn1D(up_max, ut_max, dmax);

schemeData.grid     = g;
schemeData.dynSys   = dyn;
schemeData.uMode    = 'min';
schemeData.dMode    = 'max';
schemeData.accuracy = 'high';

tau = 0 : 0.02 : 40;

extraArgs.stopConverge       = true;
extraArgs.convergeThreshold  = 1e-4;
extraArgs.keepLast           = true;
extraArgs.visualize          = false;

t0 = tic;
[V, ~, ~] = HJIPDE_solve(data0, tau, schemeData, 'maxVWithV0', extraArgs);
elapsed = toc(t0);

[~, i_r] = min(abs(g.vs{1}));
[~, i_v] = min(abs(g.vs{2}));
teb = V(i_r, i_v);

dV = computeGradients(g, V);
end
