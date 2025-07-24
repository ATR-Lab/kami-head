#!/usr/bin/env python3
"""
Micro-ROS Agent Manager for Coffee Ears ESP32

This node manages the Docker container lifecycle for the micro-ROS agent
that communicates with the ESP32 ear motion controller.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import subprocess
import threading
import time
import os
import signal
import sys


class MicroROSManager(Node):
    """ROS2 node that manages the micro-ROS agent Docker container"""
    
    def __init__(self):
        super().__init__('microros_manager')
        
        # Parameters
        self.declare_parameter('device_path', '/dev/ttyUSB1')
        self.declare_parameter('verbosity', 6)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('restart_on_failure', True)
        self.declare_parameter('health_check_interval', 5.0)
        
        self.device_path = self.get_parameter('device_path').value
        self.verbosity = self.get_parameter('verbosity').value
        self.auto_start = self.get_parameter('auto_start').value
        self.restart_on_failure = self.get_parameter('restart_on_failure').value
        self.health_check_interval = self.get_parameter('health_check_interval').value
        
        # State
        self.container_process = None
        self.is_running = False
        self.should_restart = True
        
        # Publishers
        self.status_pub = self.create_publisher(
            Bool, 
            'microros_agent/status', 
            10
        )
        
        self.diagnostics_pub = self.create_publisher(
            String, 
            'microros_agent/diagnostics', 
            10
        )
        
        # Subscribers
        self.control_sub = self.create_subscription(
            Bool,
            'microros_agent/control',
            self.control_callback,
            10
        )
        
        # Timers
        self.health_timer = self.create_timer(
            self.health_check_interval, 
            self.health_check
        )
        
        # Container monitoring thread
        self.monitor_thread = None
        
        self.get_logger().info(f"Micro-ROS Manager initialized")
        self.get_logger().info(f"Device: {self.device_path}")
        self.get_logger().info(f"Verbosity: {self.verbosity}")
        self.get_logger().info(f"Auto-start: {self.auto_start}")
        
        # Auto-start if enabled
        if self.auto_start:
            self.get_logger().info("Auto-starting micro-ROS agent...")
            self.start_microros_agent()
    
    def start_microros_agent(self):
        """Start the micro-ROS agent Docker container"""
        if self.is_running:
            self.get_logger().warn("Micro-ROS agent already running")
            return False
        
        # Check if device exists
        if not os.path.exists(self.device_path):
            self.get_logger().error(f"Device {self.device_path} not found")
            self.publish_diagnostics("error", f"Device {self.device_path} not found")
            return False
        
        try:
            # Build Docker command
            cmd = [
                'sudo', 'docker', 'run',
                '--rm',
                '-v', '/dev:/dev',
                '--privileged',
                '--net=host',
                'microros/micro-ros-agent:jazzy',
                'serial',
                '--dev', self.device_path,
                f'-v{self.verbosity}'
            ]
            
            self.get_logger().info(f"Starting micro-ROS agent: {' '.join(cmd)}")
            
            # Start container
            self.container_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                preexec_fn=os.setsid  # Create new process group
            )
            
            self.is_running = True
            
            # Start monitoring thread
            self.monitor_thread = threading.Thread(
                target=self.monitor_container, 
                daemon=True
            )
            self.monitor_thread.start()
            
            self.get_logger().info("Micro-ROS agent started successfully")
            self.publish_diagnostics("started", "Micro-ROS agent container started")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to start micro-ROS agent: {e}")
            self.publish_diagnostics("error", f"Failed to start: {e}")
            self.is_running = False
            return False
    
    def stop_microros_agent(self):
        """Stop the micro-ROS agent Docker container"""
        if not self.is_running:
            self.get_logger().warn("Micro-ROS agent not running")
            return
        
        self.should_restart = False
        
        if self.container_process:
            try:
                # Send SIGTERM to process group
                os.killpg(os.getpgid(self.container_process.pid), signal.SIGTERM)
                
                # Wait for graceful shutdown
                self.container_process.wait(timeout=10)
                
            except subprocess.TimeoutExpired:
                self.get_logger().warn("Graceful shutdown timed out, forcing kill")
                try:
                    os.killpg(os.getpgid(self.container_process.pid), signal.SIGKILL)
                except:
                    pass
            except Exception as e:
                self.get_logger().error(f"Error stopping container: {e}")
        
        self.is_running = False
        self.container_process = None
        self.get_logger().info("Micro-ROS agent stopped")
        self.publish_diagnostics("stopped", "Micro-ROS agent container stopped")
    
    def monitor_container(self):
        """Monitor the container process and handle restarts"""
        while self.container_process and self.is_running:
            try:
                # Check if process is still running
                if self.container_process.poll() is not None:
                    # Process has ended
                    self.is_running = False
                    exit_code = self.container_process.returncode
                    
                    self.get_logger().warn(f"Micro-ROS agent exited with code {exit_code}")
                    self.publish_diagnostics("exited", f"Container exited with code {exit_code}")
                    
                    if self.should_restart and self.restart_on_failure:
                        self.get_logger().info("Restarting micro-ROS agent in 3 seconds...")
                        time.sleep(3)
                        if self.should_restart:  # Check again after sleep
                            self.start_microros_agent()
                    break
                
                time.sleep(1)
                
            except Exception as e:
                self.get_logger().error(f"Error in container monitor: {e}")
                break
    
    def health_check(self):
        """Periodic health check and status publishing"""
        # Publish status
        status_msg = Bool()
        status_msg.data = self.is_running
        self.status_pub.publish(status_msg)
        
        # Check device availability
        if not os.path.exists(self.device_path):
            if self.is_running:
                self.get_logger().error(f"Device {self.device_path} disappeared, stopping agent")
                self.stop_microros_agent()
            self.publish_diagnostics("error", f"Device {self.device_path} not available")
    
    def control_callback(self, msg):
        """Handle control commands (start/stop)"""
        if msg.data and not self.is_running:
            self.get_logger().info("Received start command")
            self.should_restart = True
            self.start_microros_agent()
        elif not msg.data and self.is_running:
            self.get_logger().info("Received stop command")
            self.stop_microros_agent()
    
    def publish_diagnostics(self, level, message):
        """Publish diagnostic information"""
        diag_msg = String()
        diag_msg.data = f"{level}: {message}"
        self.diagnostics_pub.publish(diag_msg)
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down micro-ROS manager...")
        self.should_restart = False
        self.stop_microros_agent()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    manager = None
    try:
        manager = MicroROSManager()
        
        # Handle shutdown gracefully
        def signal_handler(sig, frame):
            if manager:
                manager.shutdown()
            rclpy.shutdown()
            sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        rclpy.spin(manager)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if manager:
            manager.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main() 