"""

Move to specified pose with waypoints (with Robot class)

Author: Daniel Ingram (daniel-s-ingram)
        Atsushi Sakai (@Atsushi_twi)
        Seied Muhammad Yazdian (@Muhammad-Yazdian)
        Wang Zheng (@Aglargil)

Modificación del archivo original para permitir que el usuario ingrese una lista de 
    waypoints (puntos de referencia) y elija entre dos modos de operación: Modo Carrera 
    (fluido) o Modo Rescate / Reabastecimiento. Esto permite que el robot se mueva a 
    través     de múltiples puntos de manera más flexible y controlada, dependiendo del 
    escenario deseado: Maria Molina
      

P. I. Corke, "Robotics, Vision & Control", Springer 2017, ISBN 978-3-319-54413-7

"""

import matplotlib.pyplot as plt
import numpy as np
import sys
import pathlib
import json
import os

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
MAX_SIM_TIME = 30  # segundos, el robot dejara de moverse cuando el tiempo exceda este valor


# Especificaciones del Robot
MAX_LINEAR_SPEED = 15 #maxima velocidad lineal del robot
MAX_ANGULAR_SPEED = 7 #maxima velocidad angular del robot


show_animation = True 

#mueve el robot desde una posición inicial a una posición objetivo  
def move_along_waypoints(x_start, y_start, theta_start, waypoints, modo_operacion):
    x = x_start 
    y = y_start
    theta = theta_start

    # Índice para llevar el control de cuál waypoint estamos visitando (empezamos desde el objetivo 1, índice 1)
    current_goal_index = 1
    
    if len(waypoints) > 1:
        goal = waypoints[current_goal_index]
    else:
        goal = waypoints[0] # Por si acaso solo hay un punto

    x_goal = goal["x"]
    y_goal = goal["y"]
    theta_goal = goal["theta"]

    x_traj, y_traj, v_traj, w_traj = [x], [y], [0], [0] #func
    t = 0
    pausa_reabastecimiento = 0  # Usado solo si elige el Modo Rescate
    #rho = np.hypot(x_diff, y_diff)
   
    while current_goal_index < len(waypoints) and t < MAX_SIM_TIME:
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

        # --- LÓGICA SEGÚN EL MODO (Bloque B) ---
        if modo_operacion == "1":  # MODO CARRERA (Fluido)
            # Si está cerca del objetivo actual (ej. a menos de 0.5 metros), pasa al siguiente de inmediato
            if rho < 0.5:
                current_goal_index += 1
                if current_goal_index < len(waypoints):
                    goal = waypoints[current_goal_index]
                    x_goal, y_goal, theta_goal = goal["x"], goal["y"], goal["theta"]

        elif modo_operacion == "2":  # MODO RESCATE / REABASTECIMIENTO
            # Si llega al objetivo con alta precisión (rho < 0.01)
            if rho <= 0.01:
                # Simulamos un tiempo de espera/reabastecimiento (ej. 1 segundo detenido)
                if pausa_reabastecimiento < 0.5:
                    pausa_reabastecimiento += dt
                    v, w = 0, 0  # Robot completamente detenido
                else:
                    pausa_reabastecimiento = 0
                    current_goal_index += 1
                    if current_goal_index < len(waypoints):
                        goal = waypoints[current_goal_index]
                        x_goal, y_goal, theta_goal = goal["x"], goal["y"], goal["theta"]

        v_traj.append(v) #funcion append agrega el valor de v a la lista v_traj
        w_traj.append(w)

        total_waypoints = len(waypoints) - 1
        num_waypoint_actual = min(current_goal_index-1, total_waypoints)

        theta += w * dt #angulo del robot = angulo del robot + la velocidad angular* delta de tiempo
        x += v * np.cos(theta) * dt  #posición x del robot = posición x + componente x de la velocidad  
                                        #lineal * delta de tiempo
        y += v * np.sin(theta) * dt #posición y del robot = posición y + componente y de la velocidad  
                                        #lineal * delta de tiempo
        
        if show_animation:  # pragma: no cover 
            plt.cla()
            plt.arrow(
                waypoints[0]["x"], waypoints[0]["y"],
                np.cos(waypoints[0]["theta"]), np.sin(waypoints[0]["theta"]),
                color="r", width=0.1,
            )
            plt.arrow(
                x_goal, y_goal,
                np.cos(theta_goal), np.sin(theta_goal),
                color="g", width=0.1,
            )
            plot_vehicle(x, y, theta, x_traj, y_traj, t, num_waypoint_actual, total_waypoints, waypoints)
    
    # --- PAUSA AL FINAL DEL RECORRIDO ---
    print("¡Recorrido completado con éxito!")
    
    # Mantener la última visualización abierta durante unos segundos o hasta cerrar
    tiempo_pausa_final = 0.5  # segundos que se quedará estático al terminar
    t_pausa = 0
    
    while t_pausa < tiempo_pausa_final and show_animation:
        t_pausa += dt
        plt.pause(dt)

    return x_traj, y_traj, v_traj, w_traj



#dibuja el robot y su trayectoria en un plano 2D.
def plot_vehicle(x, y, theta, x_traj, y_traj, t, num_waypoint_actual, total_waypoints, waypoints): # pragma: no cover
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

    # --- DIBUJAR TODOS LOS WAYPOINTS COMO CÍRCULOS ---
    for i, wp in enumerate(waypoints):
        plt.plot(wp["x"], wp["y"], "ko", markersize=3)  # 'ko' dibuja un círculo negro en cada waypoint

    plt.plot(x_traj, y_traj, "b--")

    texto_info = (
            f" Tiempo: {t:.2f} s\n"
            f" Waypoint: {num_waypoint_actual} de {total_waypoints}"
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
    while True:
        print("\n--- CONFIGURACIÓN DE WAYPOINTS ---")
        print("1. Cuadrado")
        print("2. Triángulo")
        print("3. Ingresar puntos manualmente por consola")
        print("4. Leer puntos desde un archivo JSON")
    
        opcion = input("Elige una opción (1, 2, 3 o 4): ").strip()
    
        # 1. Guardamos los puntos según la opción del usuario en una variable
        if opcion == "1":
            # Retorna una lista de puntos formando un cuadrado
            waypoints = [
                {"x": 5.0, "y": 5.0, "theta": 0.0},
                {"x": 15.0, "y": 5.0, "theta": np.pi/2},
                {"x": 15.0, "y": 15.0, "theta": np.pi},
                {"x": 5.0, "y": 15.0, "theta": -np.pi/2},
                {"x": 5.0, "y": 5.0, "theta": 0.0}
            ]
            break
        elif opcion == "2":
            # Retorna una lista formando un triángulo
            waypoints = [
                {"x": 10.0, "y": 15.0, "theta": 0.0},
                {"x": 15.0, "y": 5.0, "theta": np.pi},
                {"x": 5.0, "y": 5.0, "theta": np.pi/3},
                {"x": 10.0, "y": 15.0, "theta": 0.0}
            ]
            break
        elif opcion == "3":
            # Aquí podrías programar un loop para pedir coordenadas x, y por consola
            waypoints = []
            try:
                total_puntos = int(input("¿Cuántos puntos en total deseas ingresar (mínimo 2)? "))
                for i in range(total_puntos):
                    print(f"\nConfigurando punto {i+1} de {total_puntos}:")
                    x = float(input("  Ingrese coordenada X: "))
                    y = float(input("  Ingrese coordenada Y: "))
                    theta = float(input("  Ingrese orientación Theta (en radianes, ej: 0.0 o 1.57): "))
                    waypoints.append({"x": x, "y": y, "theta": theta})
                break
            except ValueError:
                print("Error: Debes ingresar valores numéricos válidos. Vuelve a intentarlo.")                
        elif opcion == "4":
            # Aquí podrías programar la lectura de puntos desde un archivo JSON
            nombre_ingresado = input("Ingresa el nombre del archivo (ej: test.json): ").strip()
            nombre_archivo = os.path.join("waypoints", nombre_ingresado)
            try:
                with open(nombre_archivo, "r", encoding="utf-8") as f:
                    waypoints = json.load(f)
                print(f"¡Se cargaron {len(waypoints)} puntos exitosamente desde {nombre_archivo}!")
                break
            except FileNotFoundError:
                print(f" No se encontró el archivo '{nombre_archivo}'. Verifica el nombre e inténtalo de nuevo.")                
        else:
            print("Opción no válida. Por favor, elige un número entre 1 y 4.")
            

    # 2. Extraemos el primer punto para que sea el inicio del robot
    punto_inicio = waypoints[0]
    x_start = punto_inicio["x"]
    y_start = punto_inicio["y"]
    theta_start = punto_inicio["theta"]

    # 3. El resto de los puntos (desde el índice 1 en adelante) son los objetivos reales
    #lista_objetivos = waypoints[1:]

    # 4. Aquí puedes preguntar también por el modo (Bloque B): Carrera o Rescate
    print("Seleccione el modo de operación:")
    print("1. Modo Carrera (Fluido / Pasa por los puntos)")
    print("2. Modo Rescate / Reabastecimiento (Se detiene en cada punto)")
    modo_operacion = input("Ingrese 1 o 2: ").strip()    

    # 5. Ahora sí, llamamos a la función principal pasándole los waypoints y el modo
    move_along_waypoints(x_start, y_start, theta_start, waypoints, modo_operacion)


if __name__ == "__main__":
    main()
