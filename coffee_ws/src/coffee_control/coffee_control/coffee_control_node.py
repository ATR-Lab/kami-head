#!/usr/bin/env python3
"""ROS2 node for controlling the Delonghi coffee machine"""
import asyncio
from functools import partial
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bleak.exc import BleakError

from coffee_control_msgs.srv import CoffeeCommand
from coffee_control_msgs.msg import FunctionCall
from coffee_control_msgs.srv import CoffeeCommand, MachineStatusRequest

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

        # Status service
        self.status_srv = self.create_service(
            MachineStatusRequest,
            'coffee_machine/get_status',
            self.handle_status_request,
            callback_group=cb_group
        )
        
        # Initialize event loop for async operations
        self.loop = asyncio.get_event_loop()
        
        self.get_logger().info('Coffee control node is ready')

    def handle_command(self, request, response):
        """Handle incoming coffee command service requests"""
        self.get_logger().info(f'Received command: {request.action} with parameter: {request.parameter}')
        
        # Run the command asynchronously
        future = asyncio.run_coroutine_threadsafe(
            self._execute_command(request.action, request.parameter),
            self.loop
        )
        
        try:
            # Wait for command completion with timeout
            success, message = future.result(timeout=10.0)
            response.success = success
            response.message = message
        except asyncio.TimeoutError:
            self.get_logger().error('Command timed out')
            response.success = False
            response.message = "Command timed out"
        except Exception as e:
            self.get_logger().error(f'Error executing command: {e}')
            response.success = False
            response.message = f"Error executing command: {str(e)}"
        
        return response

    async def _execute_command(self, action, parameter):
        """Execute a coffee machine command"""
        try:
            # Map ROS2 service actions to controller commands
            if action == "make":
                try:
                    beverage = AvailableBeverage[parameter.upper()]
                    await self.controller.beverage_start(beverage)
                    # Update local state
                    self.controller.cooking = beverage
                    return True, f"Started making {parameter}"
                except KeyError:
                    return False, f"Unknown beverage type: {parameter}"
                except BleakError as e:
                    return False, f"Failed to start {parameter}: {str(e)}"
                
            elif action == "cancel":
                try:
                    await self.controller.beverage_cancel()
                    # Update local state
                    self.controller.cooking = AvailableBeverage.NONE
                    return True, "Cancelled brewing"
                except BleakError as e:
                    return False, f"Failed to cancel brewing: {str(e)}"
                
            elif action == "cuplight":
                try:
                    if parameter == "on":
                        await self.controller.cup_light_on()
                        return True, "Cup light turned on"
                    elif parameter == "off":
                        await self.controller.cup_light_off()
                        return True, "Cup light turned off"
                    else:
                        return False, "Invalid cup light parameter (use 'on' or 'off')"
                except BleakError as e:
                    return False, f"Failed to control cup light: {str(e)}"
                
            elif action == "sound":
                try:
                    if parameter == "on":
                        await self.controller.sound_alarm_on()
                        return True, "Sound alerts enabled"
                    elif parameter == "off":
                        await self.controller.sound_alarm_off()
                        return True, "Sound alerts disabled"
                    else:
                        return False, "Invalid sound parameter (use 'on' or 'off')"
                except BleakError as e:
                    return False, f"Failed to control sound: {str(e)}"
                
            elif action == "energy_save":
                try:
                    if parameter == "on":
                        await self.controller.energy_save_on()
                        return True, "Energy save mode enabled"
                    elif parameter == "off":
                        await self.controller.energy_save_off()
                        return True, "Energy save mode disabled"
                    else:
                        return False, "Invalid energy save parameter (use 'on' or 'off')"
                except BleakError as e:
                    return False, f"Failed to control energy save mode: {str(e)}"
            
            elif action == "power":
                try:
                    if parameter == "off":
                        await self.controller.beverage_cancel()
                        # Update local state
                        self.controller.cooking = AvailableBeverage.NONE
                        return True, "Cancelled brewing (note: machine may still be powered on)"
                    else:
                        await self.controller.power_on()
                        return True, "Power on command sent"
                except BleakError as e:
                    return False, f"Failed to control power: {str(e)}"
                    
            else:
                return False, f"Unknown action: {action}"
                
        except Exception as e:
            self.get_logger().error(f'Unexpected error executing command: {e}')
            return False, str(e)

    def handle_status_request(self, request, response):
        """Handle status request service calls"""
        self.get_logger().info('Received status request')
        
        # Run status check asynchronously
        future = asyncio.run_coroutine_threadsafe(
            self._get_status(),
            self.loop
        )
        
        try:
            # Wait for status with timeout
            future.result(timeout=4.0)
            
            # Fill response with current state
            response.device_name = self.controller.hostname
            response.model = self.controller.model
            response.status = self.controller.status
            response.steam_nozzle = self.controller.steam_nozzle
            response.current_beverage = self.controller.cooking
            response.cup_light = self.controller.switches.cup_light
            response.energy_save = self.controller.switches.energy_save
            response.sound_enabled = self.controller.switches.sounds
            
        except asyncio.TimeoutError:
            self.get_logger().error('Status request timed out')
        except Exception as e:
            self.get_logger().error(f'Error getting status: {e}')
            
        return response
    
    async def _get_status(self):
        """Get current status from the coffee machine"""
        try:
            # Try to get device name first if we don't have it
            if not self.controller.hostname:
                name = await self.controller.get_device_name()
                if name:
                    self.get_logger().info(f'Connected to device: {name}')
                else:
                    self.get_logger().warning('Could not get device name')
            
            # Send debug command to get status
            await self.controller.debug()
            
        except BleakError as e:
            self.get_logger().debug(f'Bluetooth error during status check: {e}')
            raise
        except Exception as e:
            self.get_logger().warning(f'Error getting status: {e}')
            raise

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
