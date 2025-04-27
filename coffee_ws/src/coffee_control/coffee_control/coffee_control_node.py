#!/usr/bin/env python3
"""ROS2 node for controlling the Delonghi coffee machine"""
import asyncio
from functools import partial
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from coffee_control_msgs.srv import CoffeeCommand
from coffee_control_msgs.msg import FunctionCall

from coffee_control.delonghi_controller import DelongiPrimadonna, AvailableBeverage

class CoffeeControlNode(Node):
    """Node that handles coffee machine control via Bluetooth"""

    def __init__(self):
        super().__init__('coffee_control_node')
        
        # Parameters
        self.declare_parameter('mac_address', '')
        self.mac_address = self.get_parameter('mac_address').value
        if not self.mac_address:
            self.get_logger().error('No MAC address provided! Please set mac_address parameter')
            return

        # Initialize controller
        self.controller = DelongiPrimadonna(self.mac_address)
        self.get_logger().info(f'Initialized coffee controller for device: {self.mac_address}')

        # Create services with reentrant callback group to allow async calls
        cb_group = ReentrantCallbackGroup()
        self.srv = self.create_service(
            CoffeeCommand,
            'coffee_command',
            self.handle_command,
            callback_group=cb_group
        )

        # Create status check timer
        self.create_timer(5.0, self.check_status, callback_group=cb_group)
        
        # Initialize event loop for async operations
        self.loop = asyncio.get_event_loop()
        
        self.get_logger().info('Coffee control node is ready')

    async def connect(self):
        """Connect to the coffee machine"""
        try:
            name = await self.controller.get_device_name()
            if name:
                self.get_logger().info(f'Connected to coffee machine: {name}')
                return True
            else:
                self.get_logger().error('Failed to connect to coffee machine')
                return False
        except Exception as e:
            self.get_logger().error(f'Error connecting to coffee machine: {e}')
            return False

    def handle_command(self, request, response):
        """Handle incoming coffee command service requests"""
        self.get_logger().info(f'Received command: {request.action} with parameter: {request.parameter}')
        
        # Run the command asynchronously
        future = asyncio.run_coroutine_threadsafe(
            self._execute_command(request.action, request.parameter),
            self.loop
        )
        
        try:
            success, message = future.result(timeout=10.0)
            response.success = success
            response.message = message
        except asyncio.TimeoutError:
            response.success = False
            response.message = "Command timed out"
        except Exception as e:
            response.success = False
            response.message = f"Error executing command: {str(e)}"
        
        return response

    async def _execute_command(self, action, parameter):
        """Execute a coffee machine command"""
        try:
            # Ensure connection
            if not self.controller.connected:
                if not await self.connect():
                    return False, "Failed to connect to coffee machine"

            # Handle different commands
            if action == "make":
                try:
                    beverage = AvailableBeverage[parameter.upper()]
                    await self.controller.beverage_start(beverage)
                    return True, f"Started making {parameter}"
                except KeyError:
                    return False, f"Unknown beverage type: {parameter}"
                
            elif action == "cancel":
                await self.controller.beverage_cancel()
                return True, "Cancelled brewing"
                
            elif action == "cuplight":
                if parameter == "on":
                    await self.controller.cup_light_on()
                    return True, "Cup light turned on"
                elif parameter == "off":
                    await self.controller.cup_light_off()
                    return True, "Cup light turned off"
                else:
                    return False, "Invalid cup light parameter (use 'on' or 'off')"
                
            elif action == "sound":
                if parameter == "on":
                    await self.controller.sound_alarm_on()
                    return True, "Sound alerts enabled"
                elif parameter == "off":
                    await self.controller.sound_alarm_off()
                    return True, "Sound alerts disabled"
                else:
                    return False, "Invalid sound parameter (use 'on' or 'off')"
                
            elif action == "energy_save":
                if parameter == "on":
                    await self.controller.energy_save_on()
                    return True, "Energy save mode enabled"
                elif parameter == "off":
                    await self.controller.energy_save_off()
                    return True, "Energy save mode disabled"
                else:
                    return False, "Invalid energy save parameter (use 'on' or 'off')"
                    
            else:
                return False, f"Unknown action: {action}"
                
        except Exception as e:
            self.get_logger().error(f'Error executing command: {e}')
            return False, str(e)

    def check_status(self):
        """Periodically check coffee machine status"""
        if not self.controller.connected:
            return
            
        # Run status check asynchronously
        future = asyncio.run_coroutine_threadsafe(
            self.controller.debug(),
            self.loop
        )
        try:
            future.result(timeout=5.0)
        except Exception as e:
            self.get_logger().warning(f'Status check failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    node = CoffeeControlNode()
    
    # Use MultiThreadedExecutor to handle async operations
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        future = asyncio.run_coroutine_threadsafe(
            node.controller.disconnect(),
            node.loop
        )
        try:
            future.result(timeout=5.0)
        except Exception as e:
            node.get_logger().error(f'Error during disconnect: {e}')
            
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
