% Leer el CSV generado por el ESP32
datos = readtable('datos_ir_izquierdo.csv');

% Extraer columnas
voltajes = datos.voltaje_v;
distancias = datos.distancia_cm;

% Modelo: d = a / (v - b)
modelo = @(p, v) p(1) ./ (v - p(2));  % p(1) = a, p(2) = b

% Estimación inicial de parámetros
p0 = [10, 0.1];

% Ajuste no lineal (mínimos cuadrados)
p_opt = lsqcurvefit(modelo, p0, voltajes, distancias);

% Parámetros ajustados
a = p_opt(1);
b = p_opt(2);
fprintf('Parámetros ajustados: a = %.4f, b = %.4f\n', a, b);

% Evaluar modelo ajustado para graficar
v_fit = linspace(min(voltajes), max(voltajes), 200);
d_fit = modelo(p_opt, v_fit);

% Gráfica
figure;
plot(voltajes, distancias, 'ko', 'MarkerSize', 6, 'LineWidth', 1.5); hold on;
plot(v_fit, d_fit, 'b-', 'LineWidth', 2);
xlabel('Voltaje (V)', 'FontSize', 14);
ylabel('Distancia (cm)', 'FontSize', 14);
title('Curva calibración IR: Distancia vs Voltaje', 'FontSize', 16);
legend('Datos del sensor', 'Ajuste no lineal', 'Location', 'northeast');
grid on;
