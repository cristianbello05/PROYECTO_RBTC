#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
import math
import threading
import time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_srvs.srv import Trigger
from rclpy.executors import SingleThreadedExecutor

class TriggerNode(Node):
    def __init__(self):
        super().__init__('trigger_mision_node')
        self.srv = self.create_service(Trigger, 'iniciar_mision', self.servicio_callback)
        self.mision_iniciada = False

    def servicio_callback(self, request, response):
        self.get_logger().info('¡Orden de emergencia recibida! Desbloqueando navegador...')
        self.mision_iniciada = True
        response.success = True
        response.message = 'Misión médica autorizada. Robot en camino.'
        return response

def yaw_a_cuaternion(yaw):
    pose = PoseStamped()
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose.pose.orientation

def main():
    rclpy.init()

    # 1. Crear el nodo del servicio
    nodo_servicio = TriggerNode()
    
    # 2. CREAR UN EJECUTOR PRIVADO EXCLUSIVO PARA EL SERVICIO
    executor = SingleThreadedExecutor()
    executor.add_node(nodo_servicio)
    
    # 3. Poner el EJECUTOR (no rclpy.spin) a girar en el hilo de fondo
    hilo_spin = threading.Thread(target=executor.spin, daemon=True)
    hilo_spin.start()

    # 4. Inicializar el Comandante en el hilo principal
    navigator = BasicNavigator()

    # Ruta exacta de tu YAML
    yaml_path = '/home/cristianbellovz/s7_proyecto_rbtc/src/basic_dif_bot_description/config/mision_medica.yaml'

    nodo_servicio.get_logger().info('Servicio /iniciar_mision listo. Esperando llamado en la terminal...')

    # Bucle de espera pasiva hasta que llamas al servicio
    while rclpy.ok() and not nodo_servicio.mision_iniciada:
        time.sleep(0.5)

    if not rclpy.ok():
        return

    nodo_servicio.get_logger().info('Iniciando lectura de YAML y configuración de Nav2...')

    # Lectura de Waypoints desde el YAML
    try:
        with open(yaml_path, 'r') as file:
            data = yaml.safe_load(file)
    except Exception as e:
        nodo_servicio.get_logger().error(f'Error al leer el YAML: {e}')
        rclpy.shutdown()
        return
    
    waypoints = []
    for wp in data['mission_waypoints']:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(wp['position']['x'])
        pose.pose.position.y = float(wp['position']['y'])
        pose.pose.orientation = yaw_a_cuaternion(float(wp['orientation']['theta']))
        waypoints.append(pose)
        nodo_servicio.get_logger().info(f"Cargado punto: {wp['name']}")

    # 5. AHORA SÍ: BasicNavigator usa el ejecutor global sin chocar con el del servicio
    navigator.waitUntilNav2Active(localizer="amcl")
    nodo_servicio.get_logger().info('Nav2 confirmado. Iniciando desplazamiento autónomo hacia la zona de triaje...')
    
    navigator.followWaypoints(waypoints)

    # Monitoreo constante mientras navega
    while not navigator.isTaskComplete():
        time.sleep(0.5)

    # Confirmación Final
    resultado = navigator.getResult()
    if resultado == TaskResult.SUCCEEDED:
        nodo_servicio.get_logger().info('¡Llegada al destino exitosa! Prototipo en posición de primera respuesta.')
    else:
        nodo_servicio.get_logger().error('Alerta: La misión fue cancelada o el robot se atascó.')

    # Limpieza final limpia
    executor.shutdown()
    rclpy.shutdown()
    hilo_spin.join()

if __name__ == '__main__':
    main()
