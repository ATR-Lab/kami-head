#!/usr/bin/env python3

from dynamixel_sdk import COMM_SUCCESS
from dynamixel_sdk import PacketHandler
from dynamixel_sdk import PortHandler
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from coffee_buddy_msgs.msg import SetupDynamixel
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque
POSITION_CONTROL = 3  # Value for position control mode

# Control table address
ADDR_OPERATING_MODE = 11  # Control table address is different in Dynamixel model
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

PROTOCOL_VERSION = 2.0  # Default Protocol version of DYNAMIXEL X series.

class ReadWriteCoffee(Node):
    def __init__(self):
        super().__init__('read_write_coffee')

        # Declare all ROS2 parameters for this node
        self._declare_parameters()

        # Get all parameters
        self._get_parameters()

        # Try the specified port first
        self.port_handler = PortHandler(self.device_name)
        success = self.port_handler.openPort()
        
        # If the specified port failed, try alternative ports
        if not success:
            self.get_logger().warn(f'Failed to open {self.device_name}, trying alternative ports')
            
            # List of ports to try
            alternative_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1']
            
            # Remove the already tried port from alternatives
            if self.device_name in alternative_ports:
                alternative_ports.remove(self.device_name)
                
            # Try each alternative port
            for alt_port in alternative_ports:
                self.get_logger().info(f'Trying alternative port: {alt_port}')
                self.port_handler = PortHandler(alt_port)
                success = self.port_handler.openPort()
                if success:
                    self.get_logger().info(f'Successfully opened port {alt_port}')
                    break
                    
        # If we still failed to open any port
        if not success:
            self.get_logger().error('Failed to open any port!')
            return
            
        self.get_logger().info('Succeeded to open the port.')
        
        # Initialize the packet handler
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.setBaudRate(self.baud_rate):
            self.get_logger().error('Failed to set the baudrate!')
            return
        self.get_logger().info('Succeeded to set the baudrate.')

        qos = QoSProfile(depth=10)

        self.setup_dynamixel_subscription = self.create_subscription(
            SetupDynamixel,
            'setup_dynamixel',
            self.setup_dynamixel_callback,
            qos
        )

        self.subscription = self.create_subscription(
            SetPosition,
            'set_position',
            self.set_position_callback,
            qos
        )

        self.srv = self.create_service(GetPosition, 'get_position', self.get_position_callback)

    def _declare_parameters(self):
        """Declare all ROS2 parameters for this node."""
        self.declare_parameter(
            'device_name', 
            '/dev/ttyUSB0', 
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Device name for the Dynamixel motor'
            )
        )
        self.declare_parameter(
            'baud_rate', 
            1000000, 
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='Baud rate for the Dynamixel motor'
            )
        )

    def _get_parameters(self):
        """Get parameter values from the ROS2 parameter system."""
        self.device_name = self.get_parameter('device_name').value
        self.baud_rate = self.get_parameter('baud_rate').value

    def __del__(self):
        self.packet_handler.write1ByteTxRx(self.port_handler,
                                           1,
                                           ADDR_TORQUE_ENABLE,
                                           TORQUE_DISABLE)
        self.port_handler.closePort()
        self.get_logger().info('Shutting down read_write_coffee')

    def setup_dynamixel_callback(self, msg):
        dxl_id = msg.id

        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_OPERATING_MODE, POSITION_CONTROL
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to set Position Control Mode: \
                                    {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        else:
            self.get_logger().info('Succeeded to set Position Control Mode.')

        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to enable torque: \
                                    {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        else:
            self.get_logger().info('Succeeded to enable torque.')

    def set_position_callback(self, msg):
        goal_position = msg.position

        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, msg.id, ADDR_GOAL_POSITION, goal_position
        )

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Error: \
                                    {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        elif dxl_error != 0:
            self.get_logger().error(f'Error: {self.packet_handler.getRxPacketError(dxl_error)}')
        else:
            self.get_logger().info(f'Set [ID: {msg.id}] [Goal Position: {msg.position}]')

    def get_position_callback(self, request, response):
        dxl_present_position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, request.id, ADDR_PRESENT_POSITION
        )

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        elif dxl_error != 0:
            self.get_logger().error(f'Error: {self.packet_handler.getRxPacketError(dxl_error)}')
        else:
            self.get_logger().info(f'Get [ID: {request.id}] \
                                   [Present Position: {dxl_present_position}]')

        response.position = dxl_present_position
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = ReadWriteCoffee()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()