#!/usr/bin/env python3

"""
Coffee Dispenser Node - Controls the Delonghi coffee machine via Bluetooth.
"""

import asyncio
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from coffee_interfaces.srv import DispenseCoffee
from functools import partial
import threading
import time
from bleak import BleakScanner
import sys
import os
import json

# Add the path to the delonghi_controller
sys.path.append('/home/vr-workstation-2/Documents/GitHub/coffee-club/delonghi_controller/src')
from delonghi_controller import DelongiPrimadonna, AvailableBeverage

# Topic for status updates
COFFEE_DISPENSER_STATUS_TOPIC = 'coffee_dispenser/status'
# Service name for coffee dispensing
DISPENSE_COFFEE_SERVICE = 'dispense_coffee'
# Delonghi device name to search for
DELONGHI_DEVICE_NAME = "D1708353"

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
    
    async def send_command(self, message):
        """Simulate sending a command to the machine"""
        print(f"[MOCK] Sending command to coffee machine: {message}")
        return True
    
    async def disconnect(self):
        """Simulate disconnecting from the machine"""
        self.connected = False
        print(f"[MOCK] Disconnected from coffee machine")
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

class CoffeeDispenserNode(Node):
    """
    ROS2 node that provides a service to dispense coffee using the Delonghi coffee machine.
    """
    def __init__(self):
        super().__init__('coffee_dispenser_node')
        self.get_logger().info("Coffee Dispenser node initializing...")
        
        # Create callback groups
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()
        
        # Declare parameters
        self.declare_parameter('use_mock_machine', False)
        self.use_mock = self.get_parameter('use_mock_machine').get_parameter_value().bool_value
        
        self.get_logger().info(f"Mock mode {'enabled' if self.use_mock else 'disabled'}")
        
        # Initialize member variables
        self.coffee_machine = None
        self.device_mac = None
        self.connected = False
        self.is_brewing = False
        self._event_loop = None
        self._scanning_thread = None
        self._connected_event = threading.Event()
        
        # Create status publisher
        self.status_pub = self.create_publisher(
            String, COFFEE_DISPENSER_STATUS_TOPIC, 10)
        
        # Create dispense coffee service
        self.srv = self.create_service(
            DispenseCoffee,
            DISPENSE_COFFEE_SERVICE,
            self.dispense_coffee_callback,
            callback_group=self.service_group
        )
        
        # Status update timer
        self.status_timer = self.create_timer(
            1.0, self.publish_status, callback_group=self.timer_group)
        
        # If using mock, set up the mock machine immediately
        if self.use_mock:
            self.coffee_machine = MockCoffeeMachine()
            self.device_mac = "00:11:22:33:44:55" # Mock MAC address
            self.connected = True
            self.get_logger().info("Mock coffee machine initialized and ready")
        else:
            # Start the BLE scanning in a separate thread
            self._scanning_thread = threading.Thread(target=self._run_ble_scan)
            self._scanning_thread.daemon = True
            self._scanning_thread.start()
        
        self.get_logger().info("Coffee Dispenser node initialized")

    def publish_status(self):
        """Publish current status information"""
        status_msg = String()
        
        # If using mock machine, update status information
        if self.use_mock and self.coffee_machine and isinstance(self.coffee_machine, MockCoffeeMachine):
            # Check if brewing is complete
            self.coffee_machine.check_brewing_status()
            self.is_brewing = self.coffee_machine.is_brewing
            
        if not self.connected:
            status = {"status": "disconnected", "brewing": False, "mock": self.use_mock}
        else:
            status = {
                "status": "connected", 
                "brewing": self.is_brewing,
                "device_mac": self.device_mac,
                "mock": self.use_mock
            }
        
        status_msg.data = json.dumps(status)
        self.status_pub.publish(status_msg)
    
    def _run_ble_scan(self):
        """Run BLE scanning in a separate thread with its own event loop"""
        self._event_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._event_loop)
        
        # Run the BLE scanning
        self._event_loop.run_until_complete(self._scan_for_coffee_machine())
        self._event_loop.close()
    
    async def _scan_for_coffee_machine(self):
        """Scan for the coffee machine and connect to it"""
        self.get_logger().info(f"Scanning for Delonghi coffee machine ({DELONGHI_DEVICE_NAME})...")
        
        # Try to find the device by name
        max_attempts = 5
        for attempt in range(max_attempts):
            try:
                self.get_logger().info(f"Scan attempt {attempt+1}/{max_attempts}")
                
                # Discover devices
                devices = await BleakScanner.discover()
                
                # Look for the Delonghi device
                for device in devices:
                    self.get_logger().debug(f"Found device: {device.name} ({device.address})")
                    
                    if device.name and DELONGHI_DEVICE_NAME in device.name:
                        self.device_mac = device.address
                        self.get_logger().info(f"Found Delonghi coffee machine at {self.device_mac}")
                        
                        # Create coffee machine controller
                        self.coffee_machine = DelongiPrimadonna(self.device_mac)
                        
                        # Try to connect and get device name
                        name = await self.coffee_machine.get_device_name()
                        
                        if name:
                            self.get_logger().info(f"Connected to coffee machine: {name}")
                            self.connected = True
                            self._connected_event.set()
                            return
                
                # If we reach here, device wasn't found
                self.get_logger().warn(f"Delonghi coffee machine not found in scan {attempt+1}")
                await asyncio.sleep(2)
                
            except Exception as e:
                self.get_logger().error(f"Error during BLE scan: {e}")
                await asyncio.sleep(2)
        
        self.get_logger().error(f"Failed to find Delonghi coffee machine after {max_attempts} attempts")
        
        # If real machine not found and mock mode is enabled as fallback
        if not self.get_parameter('use_mock_machine').get_parameter_value().bool_value:
            self.get_logger().warn(f"Coffee machine not found. Consider setting use_mock_machine parameter to true for testing.")
    
    def dispense_coffee_callback(self, request, response):
        """Handle request to dispense coffee"""
        self.get_logger().info(f"Received request to dispense {request.beverage_type}")
        
        # Check if the coffee machine is connected
        if not self.connected:
            response.success = False
            response.message = "Coffee machine not connected"
            self.get_logger().error("Cannot dispense coffee - machine not connected")
            return response
        
        # Check if already brewing
        if self.is_brewing:
            response.success = False
            response.message = "Already brewing coffee"
            self.get_logger().warn("Cannot dispense coffee - already brewing")
            return response
        
        # Set brewing flag
        self.is_brewing = True
        self.publish_status()  # Update status immediately
        
        # For now, we only support espresso
        if request.beverage_type.lower() != "espresso":
            self.get_logger().warn(f"Requested beverage '{request.beverage_type}' not supported, using espresso")
        
        # Dispatch to a separate thread to avoid blocking
        thread = threading.Thread(
            target=self._handle_dispense_coffee,
            args=(response,)
        )
        thread.daemon = True
        thread.start()
        
        # Return a partial response - the thread will handle the rest
        return response
    
    def _handle_dispense_coffee(self, response):
        """Handle coffee dispensing in a separate thread"""
        try:
            # Use the event loop to run the async function
            if self.use_mock:
                # For mock mode, create a new event loop
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                loop.run_until_complete(self._dispense_espresso())
                loop.close()
                
                response.success = True
                response.message = "Espresso dispensed successfully (mock mode)"
                self.get_logger().info("Mock espresso dispensing started")
            elif self._event_loop and self._event_loop.is_running():
                asyncio.run_coroutine_threadsafe(
                    self._dispense_espresso(), self._event_loop
                ).result(timeout=30)  # 30 second timeout
                
                response.success = True
                response.message = "Espresso dispensed successfully"
                self.get_logger().info("Espresso dispensed successfully")
            else:
                # If event loop isn't running, create a new one
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                loop.run_until_complete(self._dispense_espresso())
                loop.close()
                
                response.success = True
                response.message = "Espresso dispensed successfully"
                self.get_logger().info("Espresso dispensed successfully")
                
        except Exception as e:
            response.success = False
            response.message = f"Error dispensing espresso: {str(e)}"
            self.get_logger().error(f"Error dispensing espresso: {e}")
        
        finally:
            # Reset brewing flag (for real machine only, mock machine handles this itself)
            if not self.use_mock:
                self.is_brewing = False
            self.publish_status()  # Update status
    
    async def _dispense_espresso(self):
        """Dispense an espresso from the coffee machine"""
        self.get_logger().info("Starting espresso brewing...")
        
        try:
            # Start brewing espresso
            await self.coffee_machine.beverage_start(AvailableBeverage.ESPRESSO)
            
            # For non-mock mode, wait for brewing to complete
            # In a real implementation, you might want to monitor the machine's status
            if not self.use_mock:
                self.get_logger().info("Waiting for espresso to complete brewing...")
                await asyncio.sleep(25)
                self.get_logger().info("Espresso brewing completed")
            else:
                self.get_logger().info("Mock espresso brewing initiated")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error during espresso brewing: {e}")
            raise

def main(args=None):
    rclpy.init(args=args)
    
    coffee_dispenser_node = CoffeeDispenserNode()
    
    # Use a MultiThreadedExecutor to handle the service and timer in parallel
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(coffee_dispenser_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        coffee_dispenser_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 