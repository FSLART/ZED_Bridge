#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

class RecordingBridge(Node):
    def __init__(self):
        super().__init__('recording_bridge')
        
        # Escuta o tópico de trigger (pode vir do Race Director ou manual)
        self.subscription = self.create_subscription(
            Bool,
            '/system/camera_recording/trigger',
            self.trigger_callback,
            10)
        
        # Clientes para controlar o gravador
        self.start_client = self.create_client(Trigger, '/recorder/start_recording')
        self.stop_client = self.create_client(Trigger, '/recorder/stop_recording')
        
        self.is_recording = False
        self.get_logger().info('Ponte de Gravação Iniciada. À espera de trigger...')

    def trigger_callback(self, msg):
        if msg.data and not self.is_recording:
            self.start_recording()
        elif not msg.data and self.is_recording:
            self.stop_recording()

    def start_recording(self):
        # Tenta chamar o serviço do gravador
        if self.start_client.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            self.start_client.call_async(req)
            self.is_recording = True
            self.get_logger().info('COMANDO: Iniciar Gravação!')
        else:
            self.get_logger().error('Gravador não encontrado! O container está a rodar?')

    def stop_recording(self):
        if self.stop_client.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            self.stop_client.call_async(req)
            self.is_recording = False
            self.get_logger().info('COMANDO: Parar Gravação!')

def main(args=None):
    rclpy.init(args=args)
    node = RecordingBridge()
    rclpy.spin(node)
    rclpy.shutdown()