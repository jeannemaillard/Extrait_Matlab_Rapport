% % Bornes pour le problème d'optimisation correspondant aux limites du
% tripteron en position, orientation et facteur d'agrandissement du rayon
lb = [-0.07, -0.82,  0.35, -0.7, -0.122173, 0, 1]; % Inclure le rayon dans les bornes inférieures
ub = [0.29, -0.22, 0.5,  0.85398, 0.174533,  0, 1.6];

% Initialiser une cellule pour stocker toutes les solutions trouvées
all_results = cell(0, 1);
% Nombre de solutions souhaitées
num_solutions = 500; 
tic;
for i = 1:num_solutions
    % Initialisation du centre à chaque itération
    center_init = [lb(1:6) + (rand(1, 6) .* (ub(1:6) - lb(1:6))), lb(7) + (rand() * (ub(7) - lb(7)))];

    % Définir la fonction objectif à minimiser (négatif pour la maximisation)
    objective = @(center) calculate_error_traj(center, trajectory_data, rayon_sphere, rayon_sphere4);
    objective2 = @(center) calculate_error_traj2(center, trajectory2_data, rayon_sphere6, rayon_sphere4);
    combined_objective = @(center) [objective(center) ; objective2(center); -center(7)];
    
    % Définir la fonction contrainte non lineaire
    nonlcon = @(center) check_trajectories(center, trajectory_data, trajectory2_data, rayon_sphere, rayon_sphere4, rayon_sphere6);
    
    % Appeler fgoalattain pour ajuster les coordonnées du centre
    options = optimoptions(@fgoalattain, 'Display', 'off');
    [center, fval, ~, flag] = fgoalattain(combined_objective, center_init, [0, 0, 0], [1, 1, 1], [], [], [], [], lb, ub, nonlcon, options);
    % disp(fval)
    
    if flag > 0
        all_results{end + 1} = center';
        sol=toc
    end
end
toc
% Afficher les résultats du problème d'optimisation
disp('Solutions trouvées :');
for i = 1:numel(all_results)
    disp(['Solution ', num2str(i), ': ', mat2str(all_results{i})]);
end



%Fonction d'application de la rotation aux trajectoires
function [x_rot, y_rot, z_rot] = apply_rotation_3d(x, y, z, theta_y, theta_z, theta_x)
    % Matrice de rotation autour de l'axe y
    R_y = [cos(theta_y)  0 sin(theta_y);
           0             1  0;
           -sin(theta_y) 0 cos(theta_y)];
    % Matrice de rotation autour de l'axe z
    R_z = [cos(theta_z) -sin(theta_z) 0;
           sin(theta_z)  cos(theta_z) 0;
           0            0            1];
    R_x = [1, 0, 0;
          0, cos(theta_x), -sin(theta_x);
          0, sin(theta_x), cos(theta_x)]; % Matrice de rotation autour de l'axe X
    R = R_x* R_z * R_y;
    % Appliquer la rotation
    coords_rot = R * [x'; y'; z'];
    x_rot = coords_rot(1, :)';
    y_rot = coords_rot(2, :)';
    z_rot = coords_rot(3, :)';
end
%Verification si les points de la trajectoire sont dans l'espace de travail
function points_inside = compute_points_inside(x_traj, y_traj, z_traj, rayon_sphere6, rayon_sphere4)
    % Vérifier si chaque point de la trajectoire est à la fois à l'intérieur de l'espace de travail et à l'extérieur de l'enveloppe convexe
    points_inside = false(1, numel(x_traj));

    for i = 1:length(x_traj)
        % Calculer la distance entre le point de trajectoire et le centre de la sphère
        distance = sqrt((x_traj(i) - 0)^2 + (y_traj(i) - 0)^2 + (z_traj(i) - 0.36)^2);
        if distance <= rayon_sphere6 && distance > rayon_sphere4
            points_inside(i) = true;
        end
    end
end

function points_inside2 = compute_points_inside2(x_traj2, y_traj2, z_traj2,rayon_sphere6, rayon_sphere4)
    % Vérifier si chaque point de la trajectoire est à la fois à l'intérieur de l'espace de travail et à l'extérieur de l'enveloppe convexe
    points_inside2 = false(1, numel(x_traj2));

    for i = 1:length(x_traj2)
        % Calculer la distance entre le point de trajectoire et le centre de la sphère
        distance2 = sqrt((x_traj2(i) - 0)^2 + (y_traj2(i) - 0)^2 + (z_traj2(i) - 0.36)^2);
        if distance2 <= rayon_sphere6 && distance2 > rayon_sphere4
            points_inside2(i) = true;
        end
    end
end

%Fonctions coûts d et d2
function error_traj = calculate_error_traj(center, trajectory_data, rayon_sphere, rayon_sphere4)
    x_center = center(1);
    y_center = center(2);
    z_center = center(3);
   
    theta_z = center(4);
    theta_y = center(5);
    theta_x = center(6);
    max_radius = center(7);

    [x_traj_rot, y_traj_rot, z_traj_rot] = apply_rotation_3d(trajectory_data(:, 1)/1000, trajectory_data(:, 2)/1000, trajectory_data(:, 3)/1000, theta_z, theta_y, theta_x);
    x_traj = x_traj_rot + x_center;
    y_traj = y_traj_rot + y_center;
    z_traj = z_traj_rot + z_center;


    points_inside = compute_points_inside(x_traj, y_traj, z_traj, rayon_sphere, rayon_sphere4);
    distance = sqrt((x_traj(~points_inside) - 0).^2 + (y_traj(~points_inside) - 0).^2 + (z_traj(~points_inside) - 0.36).^2);
    error_traj = sum(distance);
end

function error_traj2 = calculate_error_traj2(center, trajectory2_data, rayon_sphere6, rayon_sphere4)
    x_center = center(1);
    y_center = center(2);
    z_center = center(3);
    theta_z = center(4);
    theta_y = center(5);
    theta_x = center(6);
    max_radius = center(7);
    
   
    [x_traj2_rot, y_traj2_rot, z_traj2_rot] = apply_rotation_3d(trajectory2_data(:, 1), trajectory2_data(:, 2), trajectory2_data(:, 3), theta_y, theta_z, theta_x);
    x_traj2 =x_traj2_rot + x_center;
    y_traj2 =y_traj2_rot + y_center;
    z_traj2 = z_traj2_rot + z_center;

    points_inside2 = compute_points_inside(x_traj2, y_traj2, z_traj2, rayon_sphere6, rayon_sphere4);
    distance2 = sqrt((x_traj2(~points_inside2) - 0).^2 + (y_traj2(~points_inside2) - 0).^2 + (z_traj2(~points_inside2) - 0.36).^2);
    error_traj2 = sum(distance2);
end


% Fonction contrainte pour vérifier si les trajectoires sont dans leur espace de travail
function [c, ceq] = check_trajectories(center, trajectory_data, trajectory2_data, rayon_sphere, rayon_sphere4, rayon_sphere6)
    x_center = center(1);
    y_center = center(2);
    z_center = center(3);
    theta_z = center(4);
    theta_y = center(5);
    theta_x = center(6);
    max_radius = center(7);

    Rz = [cos(theta_z), -sin(theta_z), 0;
          sin(theta_z), cos(theta_z), 0;
          0, 0, 1];
    Ry = [cos(theta_y), 0, sin(theta_y);
          0, 1, 0;
          -sin(theta_y), 0, cos(theta_y)];
    Rx = [1, 0, 0;
          0, cos(theta_x), -sin(theta_x);
          0, sin(theta_x), cos(theta_x)]; 

    traj1_rotated = (Rx*Rz * Ry * (trajectory_data(:, 1:3)' / 1000 * max_radius))' + [x_center, y_center, z_center];
    traj2_rotated = (Rx*Rz * Ry * (trajectory2_data(:, 1:3)' * max_radius))' + [x_center, y_center, z_center];

    c = []; % Initialize inequality constraints
    ceq = []; % Initialize equality constraints

    for j = 1:length(traj1_rotated)
        distance1 = sqrt((traj1_rotated(j, 1) - 0)^2 + (traj1_rotated(j, 2) - 0)^2 + (traj1_rotated(j, 3) - 0.36)^2);
        distance2 = sqrt((traj2_rotated(j, 1) - 0)^2 + (traj2_rotated(j, 2) - 0)^2 + (traj2_rotated(j, 3) - 0.36)^2);

        c = [c; distance1 - rayon_sphere; rayon_sphere4 - distance1; distance2 - rayon_sphere6; rayon_sphere4 - distance2];
    end
end
