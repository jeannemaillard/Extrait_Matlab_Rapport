% Initialiser la cellule de résultats finaux
final_results_al = cell(1, 1);
% Définir les objectifs et poids pour l'optimisation
goal_error = [0, 0];
weights = [1, 1];
num = 1455;
% Initialiser le compteur de CI trouvées
num_CI_found = 0;

% Initialiser les tableaux pour les différences --> sert pour l'analyse 
diff_x_arrayobj = zeros(num, 4); 
diff_y_arrayobj = zeros(num, 4);
diff_z_arrayobj = zeros(num, 4);
diff_x_arrayobj2 = zeros(num, 4);
diff_y_arrayobj2 = zeros(num, 4);
diff_z_arrayobj2 = zeros(num, 4);

% Initialiser la matrice de données de la seconde trajectoire
trajectory2_data = zeros(size(trajectory_data));

% Définir le seuil de temps en secondes
time_max = 75; % Ajustez ce seuil selon vos besoins

% Boucle à travers tous les centres de trajectoire optimisés
for k = 1:size(sorted_results)
    fprintf('Traitement du centre %d\n', k);

    % Démarrer le chronométrage
    tic;

    % Récupérer les coordonnées du centre de trajectoire k
    x_center_k = sorted_results(k, 1); % Coordonnées x
    y_center_k = sorted_results(k, 2);% Coordonnées y
    z_center_k = sorted_results(k, 3);% Coordonnées z
    theta_z = sorted_results(k, 4); % Rotation autour de z
    theta_y = sorted_results(k, 5); % Rotation autour de y
    theta_x = deg2rad(0); % Rotation autour de x
    rayon =sorted_results(k, 7); % Rayon variable pour chaque solution
   
    %creer trajectoire 2 pour chaque solutions géometriques
    traj1_rotated = (trajectory_data(:, 1:3)' / 1000)' * rayon ;
    % Appliquer le décalage
    x_traj1 = traj1_rotated(:, 1);
    y_traj1 = traj1_rotated(:, 2);
    z_traj1 = traj1_rotated(:, 3);
    % Calculer la distance radiale de la trajectoire originale
    r_traj = sqrt(x_traj1.^2 + y_traj1.^2 + z_traj1.^2);
    r_increase = 0.152; % Espacement entre les trajectoires
    % Calculer le facteur d'augmentation pour chaque coordonnée
    scale_factor = (r_traj + r_increase) ./ r_traj;
    % Mise à jour des coordonnées avec le nouveau rayon
    x_traj21 = x_traj1 .* scale_factor;
    y_traj21 = y_traj1 .* scale_factor;
    z_traj21 = z_traj1 .* scale_factor;
    % Combinaison des données avec rayon variable en une seule matrice
    trajectory2 = [x_traj21, y_traj21, z_traj21];
    trajectory = [x_traj1, y_traj1, z_traj1];

    % Initialiser le compteur de CI trouvées
    num_CI_found = 0;
  
    % Boucle jusqu'à ce que le nombre de CI désirées soit atteint ou le temps soit écoulé
    while num_CI_found < num_CI_desired
        % Initialiser les matrices de résultats pour cette CI
        result_angles = zeros(num, 7);
        result_coord = zeros(num, 3);
        
        % Itérer sur chaque point de la trajectoire
        for i = 1:num       
            % Appliquer les rotations à la trajectoire
            % Matrices de rotation
            Rz = [cos(theta_z), -sin(theta_z), 0;
                  sin(theta_z), cos(theta_z), 0;
                  0, 0, 1]; % Rotation autour de l'axe Z
            Ry = [cos(theta_y), 0, sin(theta_y);
                  0, 1, 0;
                  -sin(theta_y), 0, cos(theta_y)]; % Rotation autour de l'axe Y
            R_x = [1, 0, 0;
                  0, cos(theta_x), -sin(theta_x);
                  0, sin(theta_x), cos(theta_x)];
        
            traj_rotated= (Rx * Rz * Ry * trajectory(:, 1:3)')' + sorted_results(k, 1:3);
            traj2_rotated= (Rx * Rz * Ry * trajectory2(:, 1:3)')' + sorted_results(k, 1:3);
        
            %changement de nom de variable
            x_traj = traj_rotated(:, 1);
            y_traj = traj_rotated(:, 2);
            z_traj = traj_rotated(:, 3);
            x_traj2 = traj2_rotated(:, 1);
            y_traj2 = traj2_rotated(:, 2);
            z_traj2 = traj2_rotated(:, 3);

            x_desired = x_traj(i);
            y_desired = y_traj(i);
            z_desired = z_traj(i);
            
            x_desired2 = x_traj2(i);
            y_desired2 = y_traj2(i);
            z_desired2 = z_traj2(i);

            % Si c'est le premier point de la trajectoire, initialiser theta_init avec des angles aléatoires
            if i == 1
                 theta_init = mod(butee_rad(:, 1)' + rand(1, 7).*(butee_rad(:, 2) - butee_rad(:, 1))', 2*pi);
            else
                % Pour les points suivants, utiliser les angles optimaux précédents comme theta_init
                theta_init = result_angles(i-1, :);
            end
            
            % Optimisation de la cinématique inverse
            [angles_opt, flag] = optimize_inverse_kinematics(dbs, dse, dew, dwf, robot, result_angles, env, env2, x_desired, y_desired, z_desired, x_desired2, y_desired2, z_desired2, theta_init, butee_rad, goal_error, weights);
           
            % Vérifier si une collision se produit
            if flag < 0 
                break; % Sortir de la boucle for et recommencer avec de nouvelles conditions initiales
            end
            
            %solutions angulaires dans result_angles
            result_angles(i, :) = angles_opt;
           
          
            % Calculer les coordonnées calculées --> sert à la verification
            % de la precision de suivi
            theta1 = angles_opt(1);
            theta2 = angles_opt(2);
            theta3 = angles_opt(3);
            theta4 = angles_opt(4);
            theta5 = angles_opt(5);
            theta6 = angles_opt(6);
            theta7 = angles_opt(7);
            %Equations cinematique directe
            x_desired_calculated = dwf * (cos(theta6) * (sin(theta4) * (sin(theta1) * sin(theta3) - cos(theta1) * cos(theta2) * cos(theta3)) + cos(theta1) * cos(theta4) * sin(theta2)) - sin(theta6) * (cos(theta5) * (cos(theta4) * (sin(theta1) * sin(theta3) - cos(theta1) * cos(theta2) * cos(theta3)) - cos(theta1) * sin(theta2) * sin(theta4)) + sin(theta5) * (cos(theta3) * sin(theta1) + cos(theta1) * cos(theta2) * sin(theta3)))) + dew * (sin(theta4) * (sin(theta1) * sin(theta3) - cos(theta1) * cos(theta2) * cos(theta3)) + cos(theta1) * cos(theta4) * sin(theta2)) + dse * cos(theta1) * sin(theta2);
            y_desired_calculated = dse * sin(theta1) * sin(theta2) - dew * (sin(theta4) * (cos(theta1) * sin(theta3) + cos(theta2) * cos(theta3) * sin(theta1)) - cos(theta4) * sin(theta1) * sin(theta2)) - dwf * (cos(theta6) * (sin(theta4) * (cos(theta1) * sin(theta3) + cos(theta2) * cos(theta3) * sin(theta1)) - cos(theta4) * sin(theta1) * sin(theta2)) - sin(theta6) * (cos(theta5) * (cos(theta4) * (cos(theta1) * sin(theta3) + cos(theta2) * cos(theta3) * sin(theta1)) + sin(theta1) * sin(theta2) * sin(theta4)) + sin(theta5) * (cos(theta1) * cos(theta3) - cos(theta2) * sin(theta1) * sin(theta3))));
            z_desired_calculated = dbs + dew * (cos(theta2) * cos(theta4) + cos(theta3) * sin(theta2) * sin(theta4)) + dwf * (sin(theta6) * (cos(theta5) * (cos(theta2) * sin(theta4) - cos(theta3) * cos(theta4) * sin(theta2)) + sin(theta2) * sin(theta3) * sin(theta5)) + cos(theta6) * (cos(theta2) * cos(theta4) + cos(theta3) * sin(theta2) * sin(theta4))) + dse * cos(theta2);
    
            %Valeurs des coordonnées effecteur dans result_angles
            result_coord(i, :) = [x_desired_calculated, y_desired_calculated, z_desired_calculated];
            
            %Equations cinématique directe
            x6_calc = dew * (sin(theta4) * (sin(theta1) * sin(theta3) - cos(theta1) * cos(theta2) * cos(theta3)) + cos(theta1) * cos(theta4) * sin(theta2)) + dse * cos(theta1) * sin(theta2);
            y6_calc = dse * sin(theta1) * sin(theta2) - dew * (sin(theta4) * (cos(theta1) * sin(theta3) + cos(theta2) * cos(theta3) * sin(theta1)) - cos(theta4) * sin(theta1) * sin(theta2));
            z6_calc = dbs + dew * (cos(theta2) * cos(theta4) + cos(theta3) * sin(theta2) * sin(theta4)) + dse * cos(theta2);
            
            %Vérification de la precision
            tolerance = 0.009;
            % Vérifier si les coordonnées calculées sont égales aux coordonnées désirées
            if abs(x6_calc - x_desired2) > tolerance || abs(y6_calc - y_desired2) > tolerance || abs(z6_calc - z_desired2) > tolerance
                break; %si respecte pas break
            end
            
            %Calcul différences --> sert dans l'analyse
            % Calculer les différences
            diff_x = abs(x_desired - x_desired_calculated);
            diff_y = abs(y_desired - y_desired_calculated);
            diff_z = abs(z_desired - z_desired_calculated);
            % Stocker les différences dans les tableaux
            diff_x_arrayobj(i, k) = diff_x;
            diff_y_arrayobj(i, k) = diff_y;
            diff_z_arrayobj(i, k) = diff_z;
            % Calculer les différences
            diff_x2 = abs(x_desired2 - x6_calc);
            diff_y2 = abs(y_desired2 - y6_calc);
            diff_z2 = abs(z_desired2 - z6_calc);
            % Stocker les différences dans les tableaux
            diff_x_arrayobj2(i, k) = diff_x2;
            diff_y_arrayobj2(i, k) = diff_y2;
            diff_z_arrayobj2(i, k) = diff_z2;
            
            %Vérification de la vitesse articulaire
            if i > 1
                angular_velocity = diff(rad2deg(result_angles(i, :))); % Calcul de la vitesse articulaire
                if (abs(angular_velocity)) < 17 
                   disp('Possible reconfiguration.');
                   break;
                end
            end

            %Vérification de la cyclicité des angles
            if i == num
                angle_diff = abs((result_angles(1, 1) - result_angles(num, 1)));
                if any(angle_diff > 0.004)
                    disp('Les angles du premier et du dernier point ne sont pas les mêmes. Trajectoire non cyclique.');
                    break;
                end
            end
        end
        
        elapsed_time = toc;
        % fprintf('Temps écoulé pour le centre %d \n', k, elapsed_time );
        if elapsed_time > time_max
            fprintf('Temps écoulé pour le centre %d a dépassé le seuil de %f secondes. Passer au centre suivant.\n', k, time_max);
            break; % Passer au centre suivant
        end
     
        % Si tous les points de la trajectoire ont été atteints sans collision
        if i == num % Tous les points de la trajectoire ont été atteints avec succès
            num_CI_found = num_CI_found + 1;
            Temps_total = toc
            final_results_al{k}{num_CI_found} = struct('angles_opt', result_angles, 'coord', result_coord, 'center', [x_center_k, y_center_k, z_center_k]);
        end
    end

end

%Affichage des résutats
filename14 = 'Resultats_Cinematique.txt';
% Ouvrir le fichier en écriture
fid12= fopen(filename14, 'w');
for k = 1:size(sorted_results)
    fprintf('Résultats pour le centre %d :\n', k);
    disp(sorted_results(k,:));
    % Vérifier si des résultats sont disponibles pour ce centre de trajectoire
    if ~isempty(final_results_al{k})
        valid_centers(k) = true; % Marquer le centre comme valide
        % Parcourir toutes les configurations initiales trouvées pour ce centre de trajectoire
        for ci = 1:numel(final_results_al{k})
            % Accéder aux résultats pour cette configuration initiale
            result = final_results_al{k}{ci};
            % Parcourir tous les points de la trajectoire dans cette configuration initiale
            for i = 1:num
                % Accéder aux angles optimaux pour ce point de la trajectoire
                angles_opt = result.angles_opt(i,:);
                              
                % Afficher les résultats
                fprintf('%10.3f %10.3f %10.3f %10.3f %10.3f %10.3f %10.3f\n', ...
                    angles_opt);
                fprintf(fid12, '%10.4f %10.4f %10.4f %10.4f %10.4f %10.6f %10.6f\n', angles_opt);
            
            end
        end
    else
        disp('Aucun résultat valide trouvé pour ce centre de trajectoire.');
    end
end
% Fermer le fichier
fclose(fid12);



%Fonction pour appliquer la rotation
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

function [angles_opt, flag] = optimize_inverse_kinematics(dbs, dse, dew, dwf, robot, result_angles, env, env2, x_desired, y_desired, z_desired, x_desired2, y_desired2, z_desired2, theta_init, butee_rad, goal_error, weights)
    % Définir les bornes
    lb = butee_rad(:, 1);  % Borne inférieure
    ub = butee_rad(:, 2);  % Borne supérieure

    % Fonction coût d
    % tic;
    objective = @(theta) calculate_error(dbs, dse, dew, dwf, x_desired, y_desired, z_desired, theta);
    % time_objective1 = toc;

    % Fonction coût d2
    % tic;
    objective2 = @(theta) calculate_error2(dbs, dse, dew, dwf, x_desired2, y_desired2, z_desired2, theta);
    % time_objective2 = toc;

    % Fonctions contraintes
    % tic;
    nonlcon = @(angles_opt) contraintes(angles_opt,robot, result_angles, env, env2, x_desired, y_desired, z_desired, x_desired2, y_desired2, z_desired2);
    % time_constraints = toc;

    % Poids pour les erreurs de position
    weight_position_error = weights(1);
    weight_position_error2 = weights(2);
    
    % fonction objectif combinée
    combined_objective = @(theta) [weight_position_error * objective(theta); weight_position_error2 * objective2(theta)];
    
    % Options pour l'optimisation
    options = optimoptions(@fgoalattain, 'Display', 'off');
    
    % Optimiser en respectant les contraintes
    [angles_opt, ~, ~, flag] = fgoalattain(combined_objective, theta_init, goal_error, weights, [], [], [], [], lb, ub, nonlcon, options);
end

%Fonction coût d
function error = calculate_error(dbs, dse, dew, dwf, x_desired, y_desired, z_desired, theta)
    % tic;   
    theta1 = theta(1);
    theta2 = theta(2);
    theta3 = theta(3);
    theta4 = theta(4);
    theta5 = theta(5);
    theta6 = theta(6);
    theta7 = theta(7);
    % Calculer la position actuelle à partir des angles d'articulation
    x = dwf*(cos(theta6)*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*cos(theta4)*sin(theta2)) - sin(theta6)*(cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3)))) + dew*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*cos(theta4)*sin(theta2)) + dse*cos(theta1)*sin(theta2);
    y = dse*sin(theta1)*sin(theta2) - dew*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - cos(theta4)*sin(theta1)*sin(theta2)) - dwf*(cos(theta6)*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - cos(theta4)*sin(theta1)*sin(theta2)) - sin(theta6)*(cos(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + sin(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3))));
    z = dbs + dew*(cos(theta2)*cos(theta4) + cos(theta3)*sin(theta2)*sin(theta4)) + dwf*(sin(theta6)*(cos(theta5)*(cos(theta2)*sin(theta4) - cos(theta3)*cos(theta4)*sin(theta2)) + sin(theta2)*sin(theta3)*sin(theta5)) + cos(theta6)*(cos(theta2)*cos(theta4) + cos(theta3)*sin(theta2)*sin(theta4))) + dse*cos(theta2);
     
    % Calculer l'erreur entre la position désirée et la position actuelle
    error = norm([x_desired, y_desired, z_desired] - [x, y, z]);
    % fprintf('Temps pour la fonction objectif 1 : %f secondes\n', toc);
    
end

%Fonction coût d2
function error2 = calculate_error2(dbs, dse, dew, dwf, x_desired2, y_desired2, z_desired2, theta)
% tic;      
theta1 = theta(1);
    theta2 = theta(2);
    theta3 = theta(3);
    theta4 = theta(4);
    theta5 = theta(5);
    theta6 = theta(6);
    theta7 = theta(7);
    % Calculer la position actuelle à partir des angles d'articulation
    x6 = dew*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*cos(theta4)*sin(theta2)) + dse*cos(theta1)*sin(theta2);
    y6 = dse*sin(theta1)*sin(theta2) - dew*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - cos(theta4)*sin(theta1)*sin(theta2));
    z6 = dbs + dew*(cos(theta2)*cos(theta4) + cos(theta3)*sin(theta2)*sin(theta4)) + dse*cos(theta2);

    % Calculer l'erreur entre la position désirée et la position actuelle
    error2 = norm([x_desired2, y_desired2, z_desired2] - [x6, y6, z6]);
    % fprintf('Temps pour la fonction objectif 2 : %f secondes\n', toc);
    % 
   
end

%Fonction contrainte pour l'évitement des singularités
function singularite = calculer_proximite_singuliere(angles_opt, dbs, dse, dew, dwf)
    % Initialiser le vecteur de proximité aux zones singulières
    num_solutions = size(angles_opt, 1);
    singularite = false(num_solutions, 1);
    
    % Boucle sur toutes les solutions d'angles
    for i = 1:size(angles_opt, 1)
        
        theta = angles_opt(i, :);
        % Extraire les valeurs de theta individuellement
        theta1 = theta(1);
        theta2 = theta(2);
        theta3 = theta(3);
        theta4 = theta(4);
        theta5 = theta(5);
        theta6 = theta(6);
        theta7 = theta(7);

        % Dérivées partielles de x par rapport à chaque angle theta
        d_x_theta1 = dwf*(-dse*sin(theta1)*sin(theta2) - dew*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - cos(theta4)*sin(theta1)*sin(theta2)) - dwf*(cos(theta6)*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - cos(theta4)*sin(theta1)*sin(theta2)) - sin(theta6)*(cos(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + sin(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3)))) + dse*cos(theta1)*sin(theta2));
        d_x_theta2 = dwf*(dse*cos(theta1)*cos(theta2) - dew*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*cos(theta4)*sin(theta2)) - dwf*(cos(theta6)*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*cos(theta4)*sin(theta2)) - sin(theta6)*(cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3)))));
        d_x_theta3 = dwf*(sin(theta6)*(cos(theta5)*(cos(theta2)*sin(theta4) - cos(theta3)*cos(theta4)*sin(theta2)) + sin(theta2)*sin(theta3)*sin(theta5)) + cos(theta6)*(cos(theta2)*cos(theta4) + cos(theta3)*sin(theta2)*sin(theta4)));
        d_x_theta4 = dwf*(-dse*(cos(theta4)*sin(theta1)*sin(theta2) - sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1))) - dew*(sin(theta1)*sin(theta4)*sin(theta2) + cos(theta1)*cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3))));
        d_x_theta5 = -dwf*(sin(theta6)*(cos(theta5)*(sin(theta1)*sin(theta2)*sin(theta4) - cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3))) - cos(theta6)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3))) + dew*cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - sin(theta1)*sin(theta4)*sin(theta2)));
        d_x_theta6 = -dwf*(sin(theta6)*(cos(theta6)*(cos(theta5)*(cos(theta2)*sin(theta4) - cos(theta3)*cos(theta4)*sin(theta2)) + sin(theta2)*sin(theta3)*sin(theta5)) - sin(theta5)*sin(theta6)*(cos(theta2)*cos(theta4) + cos(theta3)*sin(theta2)*sin(theta4))) - cos(theta6)*(cos(theta5)*(cos(theta2)*sin(theta4) - cos(theta3)*cos(theta4)*sin(theta2)) + sin(theta2)*sin(theta3)*sin(theta5)));
        d_x_theta7 = 0;
        
        d_y_theta1 = dse*cos(theta1)*sin(theta2) + dew*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - cos(theta4)*sin(theta1)*sin(theta2)) + dwf*(cos(theta6)*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - cos(theta4)*sin(theta1)*sin(theta2)) - sin(theta6)*(cos(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + sin(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3))));
        d_y_theta2 = dse*sin(theta1)*cos(theta2) - dew*(sin(theta4)*sin(theta1)*sin(theta2) - cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3))) - dwf*(cos(theta6)*(sin(theta4)*sin(theta1)*sin(theta2) + cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3))) - sin(theta6)*(cos(theta5)*(cos(theta4)*sin(theta1)*sin(theta2) + cos(theta1)*sin(theta3)) + sin(theta1)*sin(theta2)*sin(theta4)*sin(theta5)) + sin(theta5)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)));
        d_y_theta3 = dwf*(sin(theta6)*(cos(theta5)*(cos(theta2)*sin(theta4) - cos(theta3)*cos(theta4)*sin(theta2)) + sin(theta2)*sin(theta3)*sin(theta5)) + cos(theta6)*(cos(theta2)*cos(theta4) + cos(theta3)*sin(theta2)*sin(theta4)));
        d_y_theta4 = -dwf*(sin(theta6)*(cos(theta5)*(cos(theta2)*cos(theta4) + cos(theta3)*sin(theta2)*sin(theta4)) - cos(theta6)*(cos(theta2)*sin(theta4) - cos(theta3)*cos(theta4)*sin(theta2))) - cos(theta6)*(cos(theta5)*(cos(theta2)*cos(theta4) + cos(theta3)*sin(theta2)*sin(theta4)) - sin(theta2)*sin(theta3)*sin(theta5)));
        d_y_theta5 = dwf*(sin(theta6)*(sin(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - sin(theta1)*sin(theta4)*sin(theta2)) - cos(theta5)*(cos(theta6)*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*cos(theta4)*sin(theta2)) + sin(theta6)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3)))) - dew*cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - sin(theta1)*sin(theta4)*sin(theta2)));
        d_y_theta6 = -dwf*(cos(theta6)*(sin(theta6)*(sin(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - sin(theta1)*sin(theta4)*sin(theta2)) - cos(theta5)*(cos(theta6)*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*cos(theta4)*sin(theta2)) + sin(theta6)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3)))) - sin(theta5)*sin(theta6)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - sin(theta1)*sin(theta4)*sin(theta2))));
        d_y_theta7 = 0; 

        d_z_theta1 = 0;
        d_z_theta2 = dbs + dew*(cos(theta4)*cos(theta2) - cos(theta3)*sin(theta2)*sin(theta4)) + dwf*(sin(theta6)*(cos(theta5)*(cos(theta4)*cos(theta2) - cos(theta3)*sin(theta2)*sin(theta4)) + sin(theta2)*sin(theta3)*sin(theta5)) + cos(theta6)*(cos(theta4)*cos(theta2) - cos(theta3)*sin(theta2)*sin(theta4))) + dse*cos(theta2);
        d_z_theta3 = dwf*(-sin(theta6)*(cos(theta5)*(cos(theta2)*cos(theta4) + cos(theta3)*sin(theta2)*sin(theta4)) - cos(theta6)*(cos(theta2)*sin(theta4) - cos(theta3)*cos(theta4)*sin(theta2))) + cos(theta6)*(cos(theta5)*(cos(theta2)*cos(theta4) + cos(theta3)*sin(theta2)*sin(theta4)) - sin(theta2)*sin(theta3)*sin(theta5)));
        d_z_theta4 = dew*(-sin(theta2)*sin(theta4) + cos(theta2)*cos(theta3)*cos(theta4)) - dwf*(sin(theta6)*(cos(theta5)*(sin(theta2)*sin(theta4) - cos(theta2)*cos(theta3)*cos(theta4)) + cos(theta2)*sin(theta3)*sin(theta5)) + cos(theta6)*(sin(theta2)*sin(theta4) - cos(theta2)*cos(theta3)*cos(theta4)));
        d_z_theta5 = -dwf*(sin(theta6)*(cos(theta6)*(cos(theta5)*(sin(theta2)*sin(theta4) - cos(theta2)*cos(theta3)*cos(theta4)) + cos(theta2)*sin(theta3)*sin(theta5)) - sin(theta5)*sin(theta6)*(sin(theta2)*sin(theta4) - cos(theta2)*cos(theta3)*cos(theta4))) - cos(theta6)*(cos(theta6)*(cos(theta5)*(sin(theta2)*sin(theta4) - cos(theta2)*cos(theta3)*cos(theta4)) + cos(theta2)*sin(theta3)*sin(theta5)) - sin(theta6)*(sin(theta5)*(cos(theta2)*sin(theta4) - cos(theta3)*cos(theta4)*sin(theta2)) + sin(theta2)*sin(theta3)*cos(theta5))));
        d_z_theta6 = -dwf*(sin(theta6)*(sin(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - sin(theta1)*sin(theta4)*sin(theta2)) - cos(theta5)*(cos(theta6)*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*cos(theta4)*sin(theta2)) + sin(theta6)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3)))) - cos(theta6)*(cos(theta5)*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - sin(theta1)*sin(theta4)*sin(theta2)) - sin(theta5)*sin(theta6)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + sin(theta1)*sin(theta4)*sin(theta2))));
        d_z_theta7 = 0;   

        % Construire la Jacobienne
        J = [d_x_theta1, d_x_theta2, d_x_theta3, d_x_theta4, d_x_theta5, d_x_theta6, d_x_theta7; 
             d_y_theta1, d_y_theta2, d_y_theta3, d_y_theta4, d_y_theta5, d_y_theta6, d_y_theta7;
             d_z_theta1, d_z_theta2, d_z_theta3, d_z_theta4, d_z_theta5, d_z_theta6, d_z_theta7];

        % Vérifier le rang de la matrice jacobienne
        if rank(J) < min(size(J))
            singularite(i) = true;
        end
    end
end


%Fonction contrainte pour l'évitement des collisions
function collision_detected = collisions(angles_opt, robot, env)
 num_solutions = size(angles_opt, 1);
 collision_detected = false(num_solutions, 1);

     % Boucle sur toutes les solutions d'angles
    for i = 1:size(angles_opt, 1)
        angles = angles_opt(i, :)';
        [isColliding,separationDist,witnessPts]=checkCollision(robot, angles_opt', env1, IgnoreSelfCollision="on") ; %Si ignoreSelfCollision est on alors iscolling est une matrice 1X1 sINON c'est 1x2
        if isColliding==1
           collision_detected(i) = true;
        end
       
    end
end

%Fonction contrainte pour la cyclicite
function cyclicite = verifier_cyclicite(result_angles)
    % Comparer les angles du premier et du dernier point de la trajectoire
    angle_diff = abs(result_angles(1, :) - result_angles(end, :));
    % Définir la tolérance
    tolerance = 0.1; % Ajustez ce seuil selon vos besoins
    % Vérifier si les différences d'angles sont toutes inférieures à la tolérance
    cyclicite = all(angle_diff < tolerance);
end

%Fonction contrainte pour éviter les reconfigurations
function reconfiguration_detected = verifier_reconfig(result_angles)
        angular_velocity = diff(rad2deg(result_angles(i, :))); % Calcul de la vitesse articulaire
        reconfiguration_detected = all((abs(angular_velocity)) < 17); % Vérifier si les différences d'angles sont inférieures au seuil
end

%Fonction contrainte
function [c, ceq] = contraintes(angles_opt, robot, result_angles, env, env2, x_desired, y_desired, z_desired, x_desired2, y_desired2, z_desired2)
    dbs = 0.36;
    dse = 0.42;
    dew = 0.4;
    dwf = 0.152;
    % Vérifier la proximité singulière
    singularite = calculer_proximite_singuliere(angles_opt, dbs, dse, dew, dwf);
    cyclicite = verifier_cyclicite(result_angles);
    collision_detected = collisions(angles_opt, robot, env);
    reconfiguration_detected = verifier_reconfig(result_angles);
  
    if ~singularite && cyclicite && ~collision_detected && ~reconfiguration_detected
        c = 0; % Contrainte respectée
    else
        c = 1; % Contrainte violée
    end

    ceq=[];
end
