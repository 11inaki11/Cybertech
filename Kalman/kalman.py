import numpy as np

class KalmanFilterRobot:
    def __init__(self, wheel_radius, wheel_distance, encoder_res, sensor_offsets):
        self.R = wheel_radius  # Radio de las ruedas [cm]
        self.L = wheel_distance  # Distancia entre ruedas [cm]
        self.encoder_res = encoder_res  # Pulsos por vuelta
        self.sensor_offsets = sensor_offsets  # Distancias de sensores al centro [cm]
        
        # Estado inicial [x, y, theta]
        self.x = np.array([[0.0], [0.0], [0.0]])
        
        # Covarianza inicial
        self.P = np.eye(3) * 0.1
        
        # Matriz de ruido del proceso
        self.Q = np.eye(3) * 0.01
        
        # Matriz de ruido de la medición (ajustar según sensores)
        self.Rm = np.eye(3) * 0.5
    
    def predict(self, pulses_l, pulses_r):
        """ Predicción del estado basada en los encoders """
        # Convertir pulsos a distancia recorrida por cada rueda
        d_l = (2 * np.pi * self.R * pulses_l) / self.encoder_res
        d_r = (2 * np.pi * self.R * pulses_r) / self.encoder_res
        
        # Movimiento del robot
        d_center = (d_r + d_l) / 2.0
        d_theta = (d_r - d_l) / self.L
        
        theta_new = self.x[2, 0] + d_theta
        x_new = self.x[0, 0] + d_center * np.cos(theta_new)
        y_new = self.x[1, 0] + d_center * np.sin(theta_new)
        
        self.x = np.array([[x_new], [y_new], [theta_new]])
        
        # Jacobiano de la función de transición
        F = np.eye(3)
        
        # Actualizar covarianza
        self.P = F @ self.P @ F.T + self.Q
    
    def update(self, sensor_readings, expected_distances):
        """ Actualización basada en los sensores """
        # Medición observada
        z = np.array(sensor_readings).reshape(3, 1)
        
        # Medición esperada
        h_x = np.array(expected_distances).reshape(3, 1)
        
        # Innovación
        y = z - h_x
        
        # Matriz de observación
        H = np.eye(3)
        
        # Covarianza de la innovación
        S = H @ self.P @ H.T + self.Rm
        
        # Ganancia de Kalman
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Actualizar estado
        self.x += K @ y
        
        # Actualizar covarianza
        self.P = (np.eye(3) - K @ H) @ self.P
    
    def get_state(self):
        return self.x.flatten()
