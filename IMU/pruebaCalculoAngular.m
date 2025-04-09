%alfa es el ángulo del que parto y theta al que llego

% Prueba de cálculo de distancia angular dirigida (en grados)
clear; clc;

% Valores de ejemplo para theta y alfa
theta_vals = [0, 10, 90, 180, 270, 350 300 80];
alfa_vals  = [270, 350, 10, 0, 90, 180 80 300];

fprintf('   theta      alfa     diferencia\n');
fprintf('  -------    -------   ------------\n');

for i = 1:length(theta_vals)
    theta = theta_vals(i);
    alfa = alfa_vals(i);
    
    % Cálculo de la diferencia angular dirigida
    delta = mod((theta - alfa + 180), 360) - 180;
    
    fprintf('%8d    %8d     %8.1f°\n', theta, alfa, delta);
end
