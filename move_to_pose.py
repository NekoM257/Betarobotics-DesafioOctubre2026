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
controller = PathFinderController(9, 15, 3) #Kp_rho = 9, Kp_alpha = 15, Kp_beta = 3
dt = 0.01 #delta de tiempo para la simulación (en segundos)
MAX_SIM_TIME = 10  # segundos, el robot dejara de moverse cuando el tiempo exceda este valor


# Especificaciones del Robot
MAX_LINEAR_SPEED = 15 #maxima velocidad lineal del robot
MAX_ANGULAR_SPEED = 7 #maxima velocidad angular del robot


show_animation = True 

#mueve el robot desde una posición inicial a una posición objetivo  
def move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal):
    x = x_start 
    y = y_start
    theta = theta_start

    x_diff = x_goal - x 
    y_diff = y_goal - y

    x_traj, y_traj, v_traj, w_traj = [x], [y], [0], [0] #func

    rho = np.hypot(x_diff, y_diff)
    t = 0
    while rho > 0.001 and t < MAX_SIM_TIME:
        t += dt 
        x_traj.append(x)
        y_traj.append(y)

        x_diff = x_goal - x
        y_diff = y_goal - y

        rho, v, w = controller.calc_control_command(x_diff, y_diff, theta, theta_goal)

        #si las velocidades exceden sus maximos, los limitan a su maxima velocidad 

        #funcion np.sign devuelve el signo de v, es decir, si v es positivo devuelve 1, 
        # si es negativo devuelve -1 y si es cero devuelve 0.
        if abs(v) > MAX_LINEAR_SPEED: 
            v = np.sign(v) * MAX_LINEAR_SPEED 

        if abs(w) > MAX_ANGULAR_SPEED:
            w = np.sign(w) * MAX_ANGULAR_SPEED

        v_traj.append(v) #funcion append agrega el valor de v a la lista v_traj
        w_traj.append(w)

        theta = theta + w * dt #angulo del robot = angulo del robot + la velocidad angular* delta de tiempo
        x = x + v * np.cos(theta) * dt  #posición x del robot = posición x + componente x de la velocidad  
                                        #lineal * delta de tiempo
        y = y + v * np.sin(theta) * dt #posición y del robot = posición y + componente y de la velocidad  
                                        #lineal * delta de tiempo
        
        if show_animation:  # pragma: no cover 
            plt.cla()
            plt.arrow(
                x_start,
                y_start,
                np.cos(theta_start),
                np.sin(theta_start),
                color="r",
                width=0.1,
            )
            plt.arrow(
                x_goal,
                y_goal,
                np.cos(theta_goal),
                np.sin(theta_goal),
                color="g",
                width=0.1,
            )
            plot_vehicle(x, y, theta, x_traj, y_traj)

    return x_traj, y_traj, v_traj, w_traj

#dibuja el robot y su trayectoria en un plano 2D.
def plot_vehicle(x, y, theta, x_traj, y_traj):  # pragma: no cover
    # Corners of triangular vehicle when pointing to the right (0 radians)
    p1_i = np.array([0.5, 0, 1]).T
    p2_i = np.array([-0.5, 0.25, 1]).T
    p3_i = np.array([-0.5, -0.25, 1]).T

    
    T = transformation_matrix(x, y, theta)
    p1 = np.matmul(T, p1_i)
    p2 = np.matmul(T, p2_i)
    p3 = np.matmul(T, p3_i)

    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], "k-")
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], "k-")
    plt.plot([p3[0], p1[0]], [p3[1], p1[1]], "k-")

    plt.plot(x_traj, y_traj, "b--")

    # Para parar la simulacion com la telca ESC.
    plt.gcf().canvas.mpl_connect(
        "key_release_event", lambda event: [exit(0) if event.key == "escape" else None]
    )

    plt.xlim(0, 20)
    plt.ylim(0, 20)

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
    for i in range(5): #

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
        move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal)


if __name__ == "__main__":
    main()
