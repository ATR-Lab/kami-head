#!/usr/bin/env python3
"""ROS2 node for controlling the Delonghi coffee machine"""
import asyncio
import concurrent.futures
import threading
from functools import partial
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bleak.exc import BleakError
import time

from coffee_control_msgs.srv import CoffeeCommand
from coffee_control_msgs.msg import FunctionCall
from coffee_control_msgs.srv import CoffeeCommand, MachineStatusRequest

from coffee_control.delonghi_controller import DelongiPrimadonna, AvailableBeverage

class MockCoffeeMachine:
    """
    Mock implementation of the Delonghi coffee machine for testing without a real machine.
    """
    def __init__(self, device_id="MOCK_MACHINE"):
        self.mac = device_id
        self.name = "Mock Coffee Machine"
        self.hostname = "MockDelonghi"
        self.model = "Prima Donna"
        self.cooking = AvailableBeverage.NONE
        self.connected = True
        self.is_brewing = False
        self.brew_start_time = None
        self.brew_duration = 25  # seconds
        self.status = "ready"
        self.steam_nozzle = "off"
        self.switches = type('Switches', (), {
            'cup_light': False,
            'energy_save': False,
            'sounds': True
        })()
        
    async def get_device_name(self):
        """Simulate getting the device name"""
        return self.hostname
        
    async def beverage_start(self, beverage_type):
        """Simulate starting a beverage"""
        self.cooking = beverage_type
        self.is_brewing = True
        self.brew_start_time = time.time()
        print(f"[MOCK] Starting beverage: {beverage_type}")
        return True
        
    async def beverage_cancel(self):
        """Simulate canceling a beverage"""
        if self.cooking != AvailableBeverage.NONE:
            self.cooking = AvailableBeverage.NONE
            self.is_brewing = False
            print(f"[MOCK] Cancelling beverage")
            return True
        return False
    
    async def cup_light_on(self):
        """Simulate turning cup light on"""
        self.switches.cup_light = True
        print("[MOCK] Cup light turned on")
        return True
        
    async def cup_light_off(self):
        """Simulate turning cup light off"""
        self.switches.cup_light = False
        print("[MOCK] Cup light turned off")
        return True
        
    async def sound_alarm_on(self):
        """Simulate turning sound on"""
        self.switches.sounds = True
        print("[MOCK] Sound turned on")
        return True
        
    async def sound_alarm_off(self):
        """Simulate turning sound off"""
        self.switches.sounds = False
        print("[MOCK] Sound turned off")
        return True
        
    async def energy_save_on(self):
        """Simulate turning energy save on"""
        self.switches.energy_save = True
        print("[MOCK] Energy save turned on")
        return True
        
    async def energy_save_off(self):
        """Simulate turning energy save off"""
        self.switches.energy_save = False
        print("[MOCK] Energy save turned off")
        return True
        
    async def power_on(self):
        """Simulate power on"""
        print("[MOCK] Power on")
        return True
    
    async def disconnect(self):
        """Simulate disconnecting from the machine"""
        self.connected = False
        print(f"[MOCK] Disconnected from coffee machine")
        return True
        
    async def debug(self):
        """Simulate debug info retrieval"""
        print(f"[MOCK] Debug info requested")
        return True
        
    def check_brewing_status(self):
        """Check if brewing is complete based on time elapsed"""
        if not self.is_brewing or self.brew_start_time is None:
            return False
            
        elapsed = time.time() - self.brew_start_time
        if elapsed >= self.brew_duration:
            self.is_brewing = False
            self.cooking = AvailableBeverage.NONE
            return True
        return False

class CoffeeControlNode(Node):
    """Node that handles coffee machine control via Bluetooth"""

    def __init__(self):
        super().__init__('coffee_control_node')
        
        # Parameters
        self.declare_parameter('mac_address', '')
        self.declare_parameter('use_mock_machine', False)
        
        self.use_mock = self.get_parameter('use_mock_machine').value
        self.mac_address = self.get_parameter('mac_address').value
        
        if self.use_mock:
            self.get_logger().info('Using mock coffee machine')
            self.mac_address = "MOCK_COFFEE_MACHINE"
        elif not self.mac_address:
            self.get_logger().error('No MAC address provided! Please set mac_address parameter')
            return
            
        self.get_logger().info(f'Coffee control node initialized for device: {self.mac_address}')

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
        
        # Thread pool for running async operations
        self._thread_pool = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        
        self.get_logger().info('Coffee control node is ready')

    def handle_command(self, request, response):
        """Handle incoming coffee command service requests"""
        self.get_logger().info(f'Received command: {request.action} with parameter: {request.parameter}')
        
        try:
            # Create a new event loop for this command
            loop = asyncio.new_event_loop()
            
            # Run the command in a separate thread with its own event loop
            future = self._thread_pool.submit(
                self._run_command,
                loop,
                request.action,
                request.parameter
            )
            
            # Wait for command completion with timeout
            success, message = future.result(timeout=30.0)
            response.success = success
            response.message = message
            
        except concurrent.futures.TimeoutError:
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
        # Create a new controller instance for this command
        if self.use_mock:
            controller = MockCoffeeMachine(self.mac_address)
        else:
            controller = DelongiPrimadonna(self.mac_address)
        
        try:
            # Map ROS2 service actions to controller commands
            if action == "make":
                try:
                    beverage = AvailableBeverage[parameter.upper()]
                    await controller.beverage_start(beverage)
                    return True, f"Started making {parameter}"
                except KeyError:
                    return False, f"Unknown beverage type: {parameter}"
                except BleakError as e:
                    return False, f"Failed to start {parameter}: {str(e)}"
                
            elif action == "cancel":
                try:
                    await controller.beverage_cancel()
                    return True, "Cancelled brewing"
                except BleakError as e:
                    return False, f"Failed to cancel brewing: {str(e)}"
                
            elif action == "cuplight":
                try:
                    if parameter.lower() == "on":
                        await controller.cup_light_on()
                    else:
                        await controller.cup_light_off()
                    return True, f"Cup light turned {parameter}"
                except BleakError as e:
                    return False, f"Failed to set cup light: {str(e)}"
                
            elif action == "sound":
                try:
                    if parameter.lower() == "on":
                        await controller.sound_alarm_on()
                    else:
                        await controller.sound_alarm_off()
                    return True, f"Sound turned {parameter}"
                except BleakError as e:
                    return False, f"Failed to set sound: {str(e)}"
                
            elif action == "energy_save":
                try:
                    if parameter.lower() == "on":
                        await controller.energy_save_on()
                    else:
                        await controller.energy_save_off()
                    return True, f"Energy save mode turned {parameter}"
                except BleakError as e:
                    return False, f"Failed to set energy save mode: {str(e)}"
                
            elif action == "power":
                try:
                    if parameter == "off":
                        await controller.beverage_cancel()
                        return True, "Cancelled brewing (note: machine may still be powered on)"
                    else:
                        await controller.power_on()
                        return True, "Power on command sent"
                except BleakError as e:
                    return False, f"Failed to control power: {str(e)}"
                    
            else:
                return False, f"Unknown action: {action}"
                
        except Exception as e:
            self.get_logger().error(f'Error executing command: {e}')
            return False, str(e)
        finally:
            # Always try to disconnect the controller
            try:
                await controller.disconnect()
            except Exception as e:
                self.get_logger().warning(f'Error disconnecting controller: {e}')

    def handle_status_request(self, request, response):
        """Handle status request service calls"""
        self.get_logger().info('Received status request')
        
        try:
            # Create a new event loop for this status request
            loop = asyncio.new_event_loop()
            
            # Run the status check in a separate thread with its own event loop
            future = self._thread_pool.submit(
                self._run_status_check,
                loop
            )
            
            # Wait for status with timeout
            status = future.result(timeout=10.0)
            
            # Fill response with current state
            if status:
                response.device_name = status['device_name']
                response.model = status['model']
                response.status = status['status']
                response.steam_nozzle = status['steam_nozzle']
                response.current_beverage = status['current_beverage']
                response.cup_light = status['cup_light']
                response.energy_save = status['energy_save']
                response.sound_enabled = status['sound_enabled']
            else:
                self.get_logger().warning('Failed to get machine status')
                
        except concurrent.futures.TimeoutError:
            self.get_logger().error('Status request timed out')
        except Exception as e:
            self.get_logger().error(f'Error getting status: {e}')
            
        return response
    
    async def _check_status(self):
        """Check coffee machine status"""
        # Create a new controller instance for status check
        if self.use_mock:
            controller = MockCoffeeMachine(self.mac_address)
            # For mock, no need to call debug() as the values are already set
        else:
            controller = DelongiPrimadonna(self.mac_address)
            try:
                await controller.debug()
            except BleakError as e:
                self.get_logger().debug(f'Bluetooth error during status check: {e}')
                return None
        
        try:
            # For mock, check if brewing has completed based on time
            if self.use_mock:
                controller.check_brewing_status()
                
            return {
                'device_name': controller.hostname,
                'model': controller.model,
                'status': controller.status,
                'steam_nozzle': controller.steam_nozzle,
                'current_beverage': controller.cooking,
                'cup_light': controller.switches.cup_light,
                'energy_save': controller.switches.energy_save,
                'sound_enabled': controller.switches.sounds
            }
            
        except Exception as e:
            self.get_logger().error(f'Error getting status details: {e}')
            return None
        finally:
            # Always try to disconnect the controller
            try:
                await controller.disconnect()
            except Exception as e:
                self.get_logger().warning(f'Error disconnecting controller: {e}')

    def _run_command(self, loop, action, parameter):
        """Run a command in its own event loop"""
        asyncio.set_event_loop(loop)
        try:
            return loop.run_until_complete(self._execute_command(action, parameter))
        finally:
            loop.close()
            
    def _run_status_check(self, loop):
        """Run a status check in its own event loop"""
        asyncio.set_event_loop(loop)
        try:
            return loop.run_until_complete(self._check_status())
        finally:
            loop.close()

    def destroy_node(self):
        """Clean up node resources"""
        if hasattr(self, '_thread_pool'):
            self._thread_pool.shutdown(wait=True)
        super().destroy_node()

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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
