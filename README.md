🤖 BetaRobotics — Desafío Octubre 2026
Simulación de navegación de un robot móvil mediante control proporcional, comparación de ganancias y seguimiento de múltiples waypoints en Python.

Este repositorio documenta mi trabajo para el desafío BetaLab Build Series — Month 1: Move to Pose (octubre de 2026). El punto de partida fue el ejemplo move_to_pose.py de PythonRobotics. A lo largo de cuatro semanas, el ejercicio evolucionó desde comprender y comentar el código hasta comparar dos controladores y crear un simulador de rutas con dos modos de operación y lectura de archivos JSON.
Alcance: es una simulación cinemática en 2D con Matplotlib; no implica el control de un robot físico.

🎯 Objetivos
- Comprender la lógica de navegación hacia una posición y orientación objetivo (move to pose).
- Observar el efecto de modificar las ganancias proporcionales Kp_rho, Kp_alpha y Kp_beta.
- Programar la navegación secuencial a través de varios puntos de referencia (waypoints).
- Comparar el comportamiento de navegación fluida con el de parada en cada punto.
- Separar las rutas del código mediante archivos .json reutilizables.


📅 Desarrollo por semanas
Semana 1 — Comprensión del código
Archivo: [`move_to_pose.py`](move_to_pose.py)

Se estudió el programa base y se añadieron comentarios en español para explicar las funciones y los cálculos principales: controlador proporcional, distancia y errores angulares respecto al objetivo, límites de velocidad, actualización de posición y orientación, y visualización de la trayectoria. Esta versión conserva la navegación de un solo robot entre posiciones generadas aleatoriamente.

Semana 2 — Comparación de ganancias proporcionales
Archivo: [`move_to_pose_2_robots.py`](move_to_pose_2_robots.py)

Se modificó la simulación para representar dos robots al mismo tiempo, con idénticas condiciones iniciales y el mismo objetivo, pero con distintos valores de las ganancias del controlador.
Parámetro	Robot 1 (original)	Robot 2 (ganancias reducidas)
Kp_rho	        9	                        4.5
Kp_alpha	    15	                        7.5
Kp_beta	        3	                        1.5

La animación dibuja las dos trayectorias con colores diferentes y muestra el tiempo de simulación y el estado de llegada de cada robot. Esto permite observar cómo la elección de ganancias influye en la respuesta de movimiento.

Semana 3 — Navegación mediante waypoints
Archivo: [`move_to_pose_points.py`](move_to_pose_points.py)

Se volvió a utilizar un solo robot, ahora capaz de recorrer una lista ordenada de puntos. El primer elemento define la posición y orientación iniciales; los siguientes son objetivos consecutivos. Se ampliaron las opciones del simulador para elegir entre dos estrategias:
1. Modo Carrera (fluido): cuando el robot se encuentra a menos de 0.5 unidades de distancia del waypoint actual, pasa al siguiente sin realizar una parada programada.
2. Modo Rescate / Reabastecimiento: exige una proximidad de 0.01 unidades o menos y realiza una pausa simulada de 0.5 s antes de avanzar al siguiente punto.
Se incorporaron rutas predeterminadas en forma de cuadrado y triángulo, falto por desarrollar la opción para introducir coordenadas manualmente desde la consola.

Semana 4 — Modos de operación y rutas JSON
Archivo: [`move_to_pose_points.py`](move_to_pose_points.py)
Carpeta: [`waypoints/`](waypoints/)

Se termino de desarrollar la opción para introducir coordenadas manualmente desde la consola y también se incorporó la lectura de rutas desde archivos .json guardados en la carpeta waypoints/, para cambiar el recorrido sin modificar el código Python.


📁 Estructura del repositorio
Betarobotics-DesafioOctubre2026/
│
├── utils/
│   ├── __init__.py
│   ├── angle.py
│   └── ... (otros archivos originales)
│
├── waypoints/
│   └── archivos .json
│
├── move_to_pose.py
├── move_to_pose_2_robots.py
├── move_to_pose_points.py
├── requirements.txt
├── README.md
└── LICENSE

⚙️ Instalación y ejecución
Requisitos: Python 3, numpy, matplotlib y scipy.

Instala las bibliotecas:
python -m pip install numpy matplotlib scipy
Clona el repositorio:
git clone https://github.com/NekoM257/Betarobotics-DesafioOctubre2026.git

Abre la terminal en esa carpeta de PythonRobotics y ejecuta la versión deseada:
python move_to_pose.py
python move_to_pose_2_robots.py
python move_to_pose_points.py
Cada comando ejecuta una simulación distinta. Para usar un archivo JSON, inicia move_to_pose_points.py desde la carpeta que contiene waypoints/, porque el programa busca esa ruta de forma relativa.

🧭 Uso del simulador de waypoints
Al ejecutar move_to_pose_points.py aparece un menú con cuatro opciones:
Opción	Función
1	    Ruta cuadrada predefinida
2	    Ruta triangular predefinida
3	    Introducir puntos manualmente
4	    Cargar una ruta desde waypoints/ en formato JSON

Después, se elige 1 para Modo Carrera o 2 para Modo Rescate / Reabastecimiento.
Formato de los archivos JSON
Cada ruta es una lista de objetos, con las claves x, y y theta (orientación en radianes). Ejemplo de contenido para waypoints/mi_ruta.json:
[
  {"x": 5.0, "y": 5.0, "theta": 0.0},
  {"x": 15.0, "y": 5.0, "theta": 1.57},
  {"x": 15.0, "y": 15.0, "theta": 3.14},
  {"x": 5.0, "y": 15.0, "theta": -1.57},
  {"x": 5.0, "y": 5.0, "theta": 0.0}
]
En la opción 4, escribe solamente mi_ruta.json; el programa le antepone waypoints/ automáticamente. Conviene definir al menos dos puntos: el primero es el inicio y el resto son destinos.


📊 Principios de control utilizados
El controlador calcula la distancia al objetivo (rho) y los errores de dirección (alpha, beta) para determinar las velocidades lineal (v) y angular (w). Las ganancias proporcionales regulan cuánto responde el robot a esos errores:
- Kp_rho: ganancia asociada a la distancia al objetivo y a la velocidad lineal.
- Kp_alpha: ganancia asociada al giro hacia el objetivo.
- Kp_beta: ganancia de corrección de la orientación final deseada.
La simulación también limita la velocidad lineal y angular. La orientación objetivo participa en el cálculo del control, aunque el cambio de waypoint se determina por la distancia al punto, no por una tolerancia de orientación final.

🛠️ Dificultades y aspectos por mejorar
- Tiempo máximo: la versión con waypoints limita la simulación a 30 s simulados; un recorrido largo puede terminar antes de visitar todos los puntos.
- Mensaje de finalización: actualmente se imprime «¡Recorrido completado con éxito!» incluso cuando el bucle termina por el límite de tiempo. Se podría distinguir entre recorrido terminado y tiempo agotado.
- Validación de archivos: sería útil comprobar el formato de los JSON, el número de puntos y la presencia de x, y y theta antes de comenzar.

📚 Aprendizajes del desafío
El desarrollo permitió practicar lectura y documentación de código Python, control proporcional de movimiento, comparación de parámetros bajo condiciones equivalentes, representación de trayectorias con Matplotlib, manejo de listas de waypoints, interacción por consola y lectura de datos estructurados en JSON. También mostró la importancia de definir condiciones de llegada y límites de simulación coherentes con cada modo de operación.
Además, este es mi primer repositorio en GitHub y mi primera experiencia utilizando Git. A través del desafío aprendí a organizar y publicar un proyecto, registrar cambios mediante commits y mantener la documentación del trabajo con un archivo README.md. Esto representó un primer acercamiento al control de versiones y a las buenas prácticas para compartir proyectos de programación. 

🎬 Demostración

![move_to_pose_2_robots](/Betarobotics-DesafioOctubre2026/gifs/move_to_pose_2_robots.gif)
Demostracion de move_to_pose_2_robots.py
![move_to_pose_points](/Betarobotics-DesafioOctubre2026/gifs/move_to_pose_points.gif)
Demostracion de move_to_pose_points.py

🔗 Referencias y créditos
- PythonRobotics — repositorio original.(https://github.com/AtsushiSakai/PythonRobotics)
- Move to Pose — código de referencia. (https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/move_to_pose)
- Documentación de PythonRobotics. (https://atsushisakai.github.io/PythonRobotics/)
- Autores originales citados en los scripts: Daniel Ingram, Atsushi Sakai, Seied Muhammad Yazdian y Wang Zheng.
- Adaptación, comentarios y extensiones del desafío: participante de este repositorio.

📄 Licencia
Este repositorio incluye una licencia MIT. Consulta también los avisos y condiciones del proyecto original PythonRobotics al reutilizar sus componentes.
