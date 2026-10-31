"""

Move to specified pose

Author: Daniel Ingram (daniel-s-ingram)
        Atsushi Sakai (@Atsushi_twi)
        Seied Muhammad Yazdian (@Muhammad-Yazdian)
        Wang Zheng (@Aglargil)

Traduccion de comentarios al español: Maria Molina       

P. I. Corke, "Robotics, Vision & Control", Springer 2017, ISBN 978-3-319-54413-7

"""

import matplotlib.pyplot as plt
import numpy as np
import sys
import pathlib

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))
from utils.angle import angle_mod 
from random import random

# angle_mod es una función que normaliza un ángulo a un rango específico, generalmente [-π, π] o [0, 2π], 
# dependiendo de la implementación. Esto es útil en robótica y control de movimiento para evitar problemas 
# con ángulos que se desbordan o se vuelven ambiguos.


class PathFinderController:
    """
    Crea una instancia de PathFinderController para la navegación de un 
    robot con ruedas de 3 grados de libertad (3-DOF) en un plano 2D.
    
    Parameters
    ----------
    Kp_rho : La ganancia de velocidad lineal para desplazar el robot a lo largo de una línea hacia el objetivo.
            - Controla qué tan rápido avanza hacia el objetivo (velocidad lineal)
    Kp_alpha : La ganancia de velocidad angular para rotar el robot hacia el objetivo. 
                - Controla la agresividad con la que el robot gira hacia el punto objetivo
    Kp_beta : La ganancia de velocidad angular de compensación que permite una convergencia suave hacia el 
                ángulo objetivo (es decir, ayuda a que la orientación del robot se vuelva paralela al ángulo 
                objetivo).
                -Controla el suavizado para alinearse con el ángulo final deseado
    """

    def __init__(self, Kp_rho, Kp_alpha, Kp_beta): 
        self.Kp_rho = Kp_rho #ganancia de velocidad lineal
        self.Kp_alpha = Kp_alpha #ganancia de velocidad angular
        self.Kp_beta = Kp_beta #ganancia de velocidad angular de compensación

    def calc_control_command(self, x_diff, y_diff, theta, theta_goal): 
        """
        Devuelve el comando de control para las velocidades lineal y angular, así como la distancia al objetivo.

        Parámetros
        ----------
        x_diff : Posición del objetivo respecto a la posición actual del robot en la dirección x
        y_diff : Posición del objetivo respecto a la posición actual del robot en la dirección y
        theta : Ángulo de orientación actual del robot respecto al eje x
        theta_goal: Ángulo objetivo del robot respecto al eje x

        Retorno
        -------
        rho : Distancia entre el robot y la posición objetivo
        v : Comando de velocidad lineal
        w : Comando de velocidad angular
                
        """

        # Descripción de las variables locales:
        # - alpha es el ángulo hacia el objetivo con respecto a la orientación del robot
        # - beta es el ángulo entre la posición del robot y la posición del objetivo, más el ángulo del objetivo
        # - Kp_rho*rho y Kp_alpha*alpha desplazan el robot a lo largo de una línea hacia el objetivo
        # - Kp_beta*beta rota dicha línea para que sea paralela al ángulo del objetivo
        #
        # Nota:
        # restringimos alpha y beta (diferencias angulares) al rango
        # [-pi, pi] para evitar comportamientos inestables; por ejemplo, que la diferencia pase
        # de 0 rad a 2*pi rad con un giro leve
                
        # La velocidad v tiene siempre un signo constante que depende del valor inicial de α.

        rho = np.hypot(x_diff, y_diff)  # Calcula la distancia entre el robot y el objetivo
        v = self.Kp_rho * rho # velocidad lineal = ganancia lineal * distancia al objetivo

        alpha = angle_mod(np.arctan2(y_diff, x_diff) - theta) #alpha = ángulo objetivo - angulo robot
        beta = angle_mod(theta_goal - theta - alpha) #beta = ángulo objetivo del robot - angulo robot - alpha
        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            # recalcular alpha para situarlo en el rango [-pi/2, pi/2]
            alpha = angle_mod(np.arctan2(-y_diff, -x_diff) - theta)
            beta = angle_mod(theta_goal - theta - alpha)
            w = self.Kp_alpha * alpha - self.Kp_beta * beta #velocidad angular = ganancia angular * alpha - ganancia angular de compensación * beta
            v = -v
        else:
            w = self.Kp_alpha * alpha - self.Kp_beta * beta

        return rho, v, w


# Parametros de simulacion
controller1 = PathFinderController(9, 15, 3) #Kp_rho = 9, Kp_alpha = 15, Kp_beta = 3
controller2 = PathFinderController(4.5, 7.5, 1.5) #Kp_rho = 4.5, Kp_alpha = 7.5, Kp_beta = 1.5
dt = 0.01 #delta de tiempo para la simulación (en segundos)
MAX_SIM_TIME = 5  # segundos, el robot dejara de moverse cuando el tiempo exceda este valor
#PLOT_FONT_SIZE = 8
#AT_TARGET_ACCEPTANCE_THRESHOLD = 0.01

# Especificaciones del Robot
MAX_LINEAR_SPEED = 15 #maxima velocidad lineal del robot
MAX_ANGULAR_SPEED = 7 #maxima velocidad angular del robot


show_animation = True 
#all_robots_are_at_target = False

#mueve el robot desde una posición inicial a una posición objetivo  
def move_two_robots(x_start, y_start, theta_start, x_goal, y_goal, theta_goal):
    # Estado inicial Robot 1
    x1, y1, theta1 = x_start, y_start, theta_start
    x_traj1, y_traj1, v_traj1, w_traj1 = [x1], [y1], [0], [0]

    # Estado inicial Robot 2 (empieza en la misma posición inicial)
    x2, y2, theta2 = x_start, y_start, theta_start
    x_traj2, y_traj2, v_traj2, w_traj2 = [x2], [y2], [0], [0]

    rho1 = np.hypot(x_goal - x1, y_goal - y1)
    rho2 = np.hypot(x_goal - x2, y_goal - y2)
    t = 0
    while (rho1 > 0.001 or rho2 > 0.001) and t < MAX_SIM_TIME:
        t += dt 
        
        # --- Actualización Robot 1 ---
        if rho1 > 0.01:
            x_diff1 = x_goal - x1
            y_diff1 = y_goal - y1
            rho1, v1, w1 = controller1.calc_control_command(x_diff1, y_diff1, theta1, theta_goal)

            if abs(v1) > MAX_LINEAR_SPEED:
                v1 = np.sign(v1) * MAX_LINEAR_SPEED
            if abs(w1) > MAX_ANGULAR_SPEED:
                w1 = np.sign(w1) * MAX_ANGULAR_SPEED

            theta1 += w1 * dt
            x1 += v1 * np.cos(theta1) * dt
            y1 += v1 * np.sin(theta1) * dt
        
        x_traj1.append(x1)
        y_traj1.append(y1)
        v_traj1.append(v1) #funcion append agrega el valor de v a la lista v_traj
        w_traj1.append(w1)

        # --- Actualización Robot 2 ---
        if rho2 > 0.01:
            x_diff2 = x_goal - x2
            y_diff2 = y_goal - y2
            rho2, v2, w2 = controller2.calc_control_command(x_diff2, y_diff2, theta2, theta_goal)

            if abs(v2) > MAX_LINEAR_SPEED:
                v2 = np.sign(v2) * MAX_LINEAR_SPEED
            if abs(w2) > MAX_ANGULAR_SPEED:
                w2 = np.sign(w2) * MAX_ANGULAR_SPEED

            theta2 += w2 * dt
            x2 += v2 * np.cos(theta2) * dt
            y2 += v2 * np.sin(theta2) * dt

        x_traj2.append(x2)
        y_traj2.append(y2)
        v_traj2.append(v2) #funcion append agrega el valor de v a la lista v_traj
        w_traj2.append(w2)

        if rho1 <= 0.01 and rho2 <= 0.01:
            t=MAX_SIM_TIME  # Si ambos robots llegaron, se termina la simulación
            plt.pause(1.0)

        # --- Animación ---
        if show_animation:
            plt.cla()
            
            # Flecha de Inicio
            plt.arrow(
                x_start, y_start, np.cos(theta_start), np.sin(theta_start),
                color="black", width=0.1
            )
            # Flecha de Objetivo
            plt.arrow(
                x_goal, y_goal, np.cos(theta_goal), np.sin(theta_goal),
                color="g", width=0.1
            )
            plot_two_vehicles(x1, y1, theta1, x_traj1, y_traj1, x2, y2, theta2, x_traj2, y_traj2, t, rho1, rho2)
            
    return x_traj1, y_traj1, v_traj1, w_traj1, x_traj2, y_traj2, v_traj2, w_traj2

#dibuja el robot y su trayectoria en un plano 2D.
def plot_two_vehicles(x1, y1, theta1, x_traj1, y_traj1, x2, y2, theta2, x_traj2, y_traj2, t, rho1, rho2):  # pragma: no cover
    #print(f"Tiempo recibido por la gráfica: {t:.2f} s")  
    # Corners of triangular vehicle when pointing to the right (0 radians)
    p1_i = np.array([0.5, 0, 1]).T
    p2_i = np.array([-0.5, 0.25, 1]).T
    p3_i = np.array([-0.5, -0.25, 1]).T
    
    # --- Dibujar Robot 1 (Azul / Negro) ---
    T1 = transformation_matrix(x1, y1, theta1)
    p1_1 = np.matmul(T1, p1_i)
    p2_1 = np.matmul(T1, p2_i)
    p3_1 = np.matmul(T1, p3_i)

    plt.plot([p1_1[0], p2_1[0]], [p1_1[1], p2_1[1]], "k-")
    plt.plot([p2_1[0], p3_1[0]], [p2_1[1], p3_1[1]], "k-")
    plt.plot([p3_1[0], p1_1[0]], [p3_1[1], p1_1[1]], "k-")
    plt.plot(x_traj1, y_traj1, "b--", label="Robot 1 (Kp original)")

    # --- Dibujar Robot 2 (Rojo / Negro) ---
    T2 = transformation_matrix(x2, y2, theta2)
    p1_2 = np.matmul(T2, p1_i)
    p2_2 = np.matmul(T2, p2_i)
    p3_2 = np.matmul(T2, p3_i)

    plt.plot([p1_2[0], p2_2[0]], [p1_2[1], p2_2[1]], "r-")
    plt.plot([p2_2[0], p3_2[0]], [p2_2[1], p3_2[1]], "r-")
    plt.plot([p3_2[0], p1_2[0]], [p3_2[1], p1_2[1]], "r-")
    plt.plot(x_traj2, y_traj2, "r--", label="Robot 2 (Kp mitad)")

    plt.legend(loc="upper right")
    
    # Evaluamos si llegaron (True si rho <= 0.01, de lo contrario False)
    estado_1 = "True" if rho1 <= 0.01 else "False"
    estado_2 = "True" if rho2 <= 0.01 else "False"

    # --- CUADRO DE TEXTO PARA EL TIEMPO Y ESTADOS ---
    # Muestra el tiempo transcurrido en la parte superior con 2 decimales y si llegaron los robots
    texto_info = (
        f" Tiempo: {t:.2f} s\n"
        f" Robot 1 (Llegó): {estado_1}\n"
        f" Robot 2 (Llegó): {estado_2}"
    )
    
    plt.text(0.5, 21.5, texto_info, fontsize=9, fontweight='bold',
             bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.5'))    

    # Para parar la simulacion com la telca ESC.
    plt.gcf().canvas.mpl_connect(
        "key_release_event", lambda event: [exit(0) if event.key == "escape" else None]
    )

    plt.xlim(0, 25)
    plt.ylim(0, 25)

    plt.pause(dt)

#genera una matriz de transformación 2D para un robot en un plano. Esta matriz combina rotación y traslación, 
# permitiendo convertir coordenadas locales del robot a coordenadas globales.

def transformation_matrix(x, y, theta): 
    return np.array(
        [
            [np.cos(theta), -np.sin(theta), x],
            [np.sin(theta), np.cos(theta), y],
            [0, 0, 1],
        ]
    )

# aquí se ejecuta el bucle n veces para randomizar diferentes posiciones iniciales y objetivos
def main():
    for i in range(3): #

        #se general las posiciones iniciales y objetivos de manera aleatoria dentro de un rango 
        # de 0 a 20 para x e y, y de -π a π para theta
        
        x_start = 20.0 * random()
        y_start = 20.0 * random()
        theta_start: float = 2 * np.pi * random() - np.pi 
        x_goal = 20 * random()
        y_goal = 20 * random()
        theta_goal = 2 * np.pi * random() - np.pi
        print(
            f"Initial x: {round(x_start, 2)} m\nInitial y: {round(y_start, 2)} m\nInitial theta: {round(theta_start, 2)} rad\n"
        )
        print(
            f"Goal x: {round(x_goal, 2)} m\nGoal y: {round(y_goal, 2)} m\nGoal theta: {round(theta_goal, 2)} rad\n"
        )
        

        move_two_robots(x_start, y_start, theta_start, x_goal, y_goal, theta_goal)


if __name__ == "__main__":
    main()
