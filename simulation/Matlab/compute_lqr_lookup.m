%% compute_lqr_lookup.m
% Precompute LQR gain K for a grid of H_eff values. Saves to lqr_lookup.mat
% for use in lqr_gain_from_lean.m (Simulink MATLAB Function block), since
% lqr() is not supported for code generation.
%
% Run this script after changing physical parameters in compute_lqr_ballbot.m
% so the lookup table matches. Save path: same directory as this script.
% When building the Simulink model, ensure this directory is on the MATLAB
% path (or that lqr_lookup.mat is in the model's folder) so coder.load can find it.
%
% === Simulink: what to set ===
% The LQR block (lqr_gain_from_lean) gets all params from this script and
% lqr_lookup.mat — no block parameters to set for LQR. After changing
% values here, run this script and rebuild the model so the block uses the
% new .mat. If your Simulink plant/dynamics (e.g. Simscape) has its own
% mass and inertia, set them to match the values below so the plant matches
% the LQR design.

%% Same physical parameters as compute_lqr_ballbot.m / lqr_gain_from_lean.m

% Mass [kg]: platform (structure under rider) and person (rider) separate
M_platform = 50.0;   % platform mass
M_person   = 30.0;   % person (torso block) mass
M_ASSY     = M_platform + M_person;   % total assembly mass for gravity term

G = 9.81;   % gravity [m/s^2]

% CoM heights above ball center [m]. Person: frame at bottom of torso block
% (rotation about bottom of block); bottom of block 0.40 m above ball; torso
% 0.2 x 0.2 x 0.5 m (L x W x H); CoM at center of block; orientation in world frame.
H_platform = 0.2;   % platform CoM height above ball center
H_person   = 0.65;  % person CoM: 0.40 (bottom of block) + 0.5/2 (half torso height)
H_CM       = (M_platform * H_platform + M_person * H_person) / M_ASSY;   % mass-weighted combined CoM height

% Rotational inertia [kg·m^2] about pivot (ball center). Estimated from geometry:
% Platform: ~0.5 x 0.4 x 0.05 m plate, 50 kg, CoM 0.2 m -> J_cm + m*d^2.
% Person: torso 0.2 x 0.2 x 0.5 m, 30 kg, CoM 0.65 m -> J_cm + m*d^2.
J_platform_roll  = 2.7;    % platform roll (X): plate + 50*0.2^2
J_platform_pitch = 3.0;   % platform pitch (Y)
J_platform_yaw   = 3.7;   % platform yaw (Z)
J_person_roll    = 13.4;  % person (torso 0.2x0.2x0.5) roll
J_person_pitch   = 13.4;  % person pitch
J_person_yaw     = 12.9;  % person yaw
J_roll  = J_platform_roll  + J_person_roll;
J_pitch = J_platform_pitch + J_person_pitch;
J_yaw   = J_platform_yaw   + J_person_yaw;

% Viscous damping [N·m·s] opposing angular velocity (roll, pitch, yaw)
B_roll  = 0.5;
B_pitch = 0.5;
B_yaw   = 0.3;

% Friction coefficients (e.g. for ball–ground or drive friction model)
mu_static  = 0.5;   % static friction coefficient
mu_dynamic = 0.2;   % dynamic friction coefficient

% LQR state and input weights: Q on [roll, pitch, yaw, omega_roll, omega_pitch, omega_yaw], R on [tau_roll, tau_pitch, tau_yaw]
Q = diag([500, 200, 50, 10, 10, 5]);
R = 0.05 * eye(3);

%% Grid of effective CoM heights [m] (same range as in lqr_gain_from_lean: 0.02 to H_CM)
H_eff_min = 0.02;   % minimum H_eff [m] (clamp in lean block to avoid singular LQR)
H_eff_max = H_CM;   % maximum H_eff [m] (upright combined CoM height)
N = 31;             % number of lookup breakpoints (fixed for code gen)
H_eff_vec = linspace(H_eff_min, H_eff_max, N);

%% Precompute LQR gain K for each H_eff
K_table = zeros(3, 6, N);
for i = 1:N
    H_eff = H_eff_vec(i);
    g_roll  = M_ASSY * G * H_eff / J_roll;
    g_pitch = M_ASSY * G * H_eff / J_pitch;
    A = [0 0 0 1 0 0;
         0 0 0 0 1 0;
         0 0 0 0 0 1;
         g_roll  0       0    -B_roll/J_roll   0              0;
         0       g_pitch 0     0               -B_pitch/J_pitch 0;
         0       0       0     0               0              -B_yaw/J_yaw];
    B = [0 0 0;
         0 0 0;
         0 0 0;
         1/J_roll  0         0;
         0         1/J_pitch 0;
         0         0         1/J_yaw];
    K_table(:,:,i) = lqr(A, B, Q, R);
end

%% Save for use in lqr_gain_from_lean.m (coder.load in Simulink)
% K_table: [3 x 6 x N] gains; H_eff_vec, H_eff_min, H_eff_max, N for indexing
save('lqr_lookup.mat', 'H_eff_vec', 'K_table', 'H_eff_min', 'H_eff_max', 'N');
fprintf('Saved lqr_lookup.mat with K_table [3 x 6 x %d] and H_eff_vec [1 x %d].\n', N, N);
fprintf('\n--- Simulink: use these in plant/dynamics if you have mass/inertia blocks ---\n');
fprintf('  M_platform=%.1f kg, M_person=%.1f kg, H_platform=%.2f m, H_person=%.2f m\n', M_platform, M_person, H_platform, H_person);
fprintf('  J_roll=%.1f, J_pitch=%.1f, J_yaw=%.1f kg·m^2  (B_roll=%.1f, B_pitch=%.1f, B_yaw=%.1f)\n', J_roll, J_pitch, J_yaw, B_roll, B_pitch, B_yaw);

%% Export C header for Arduino (controlCode/arduinoCode/control_lqr_lookup.h)
% Run from simulation/Matlab; writes to ../../controlCode/arduinoCode/control_lqr_lookup.h
out_path = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'controlCode', 'arduinoCode', 'control_lqr_lookup.h');
fid = fopen(out_path, 'w');
if fid < 0
  warning('Could not write %s (run from simulation/Matlab)', out_path);
else
  fprintf(fid, '/* Auto-generated by compute_lqr_lookup.m - do not edit */\n');
  fprintf(fid, '#ifndef CONTROL_LQR_LOOKUP_H\n#define CONTROL_LQR_LOOKUP_H\n\n');
  fprintf(fid, '#define LQR_LOOKUP_N    %d\n', N);
  fprintf(fid, '#define LQR_H_EFF_MIN  %.6ff\n', H_eff_min);
  fprintf(fid, '#define LQR_H_EFF_MAX  %.6ff\n', H_eff_max);
  fprintf(fid, '#define LQR_H_CM       %.6ff\n\n', H_CM);
  fprintf(fid, 'static const float K_LQR_TABLE[%d][3][6] = {\n', N);
  for i = 1:N
    fprintf(fid, '  {');
    for r = 1:3
      fprintf(fid, '{');
      for c = 1:6
        fprintf(fid, '%.6ff', K_table(r, c, i));
        if c < 6, fprintf(fid, ','); end
      end
      fprintf(fid, '}');
      if r < 3, fprintf(fid, ','); end
    end
    fprintf(fid, '}');
    if i < N, fprintf(fid, ','); end
    fprintf(fid, '\n');
  end
  fprintf(fid, '};\n\n#endif\n');
  fclose(fid);
  fprintf('Wrote %s for Arduino LQR lookup.\n', out_path);
end
