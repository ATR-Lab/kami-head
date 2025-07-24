#!/usr/bin/env python3
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple
import json
import asyncio
from threading import Thread
import os
import logging
from pathlib import Path

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from sui_indexer_msgs.msg import SuiEvent, IndexerStatus
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from ament_index_python.packages import get_package_share_directory

from prisma import Prisma
from prisma.models import Cursor
from sui_py import SuiClient, SuiEvent as SuiPySuiEvent, EventFilter, Page
from sui_py.exceptions import SuiRPCError, SuiValidationError

# Import event handlers
try:
    # Try direct package import (works for installed package)
    from sui_indexer.handlers import handle_escrow_objects, handle_lock_objects
except ImportError:
    try:
        # Try relative import (works when run as module)
        from .handlers import handle_escrow_objects, handle_lock_objects
    except ImportError:
        # Try direct import (works when running from source)
        from handlers import handle_escrow_objects, handle_lock_objects

@dataclass
class EventExecutionResult:
    """Result of executing an event job."""
    cursor: Optional[str]
    has_next_page: bool

@dataclass
class EventTracker:
    """Configuration for tracking a specific event type."""
    # The module that defines the type, with format `package::module`
    type: str
    # Event filter for querying
    filter: Dict[str, Any]
    # Callback function to handle events
    callback: Callable[[List[SuiPySuiEvent], str, Prisma], Any]

class SuiIndexerNode(Node):
    """ROS2 node for indexing Sui blockchain events."""
    
    def __init__(self):
        super().__init__('sui_indexer')
        
        # Declare parameters with proper types
        self.declare_parameter('package_id', '', descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='The Sui package ID to index',
            read_only=True,
            additional_constraints='Required. Must be a valid Sui package ID'
        ))
        
        self.declare_parameter('network', 'testnet', descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Sui network to connect to',
            additional_constraints='Must be one of: testnet, mainnet, devnet'
        ))
        
        self.declare_parameter('polling_interval_ms', 1000, descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Polling interval in milliseconds',
            additional_constraints='Must be > 0'
        ))
        
        self.declare_parameter('default_limit', 50, descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Default limit for event queries',
            additional_constraints='Must be between 1 and 100'
        ))
        
        self.declare_parameter('database_url', '', descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Database URL for the indexer. Use absolute path for production deployments.'
        ))
        
        self.declare_parameter('max_retries', 3, descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Maximum number of retries for failed operations',
            additional_constraints='Must be > 0'
        ))
        
        self.declare_parameter('retry_delay_ms', 1000, descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Base retry delay in milliseconds',
            additional_constraints='Must be > 0'
        ))
        
        # Validate parameters
        self._validate_parameters()
        
        # Log all settings
        self.get_logger().info("Indexer Settings:")
        self.get_logger().info(f"  Network: {self.get_parameter('network').value}")
        self.get_logger().info(f"  Package ID: {self.get_parameter('package_id').value}")
        self.get_logger().info(f"  Polling Interval: {self.get_parameter('polling_interval_ms').value}ms")
        self.get_logger().info(f"  Default Limit: {self.get_parameter('default_limit').value}")
        self.get_logger().info(f"  Database URL: {self.get_parameter('database_url').value}")
        self.get_logger().info(f"  Max Retries: {self.get_parameter('max_retries').value}")
        self.get_logger().info(f"  Retry Delay: {self.get_parameter('retry_delay_ms').value}ms")
        
        # Publishers
        self.event_pub = self.create_publisher(SuiEvent, 'sui_events', 10)
        self.status_pub = self.create_publisher(IndexerStatus, 'indexer_status', 10)
        
        # Initialize clients
        self.db = None  # Prisma client
        self.sui_client = None  # Sui client
        self.running = False
        self.indexer_thread = None
        self.loop = None
        
        # Get database URL from parameter or use default in workspace data directory
        database_url = self.get_parameter('database_url').value
        if not database_url or database_url == 'file:sui_indexer.db':
            # Use workspace data directory for default database location (NOT install directory)
            workspace_root = Path(os.getcwd())  # Should be coffee_ws when launched properly
            data_dir = workspace_root / "data" / "sui_indexer"
            data_dir.mkdir(parents=True, exist_ok=True)
            db_path = data_dir / "sui_indexer.db"
            database_url = f'file:{db_path}'
        
        self.database_url = database_url
        
        self.get_logger().info(f"Working directory: {os.getcwd()}")
        self.get_logger().info(f"Using database URL: {database_url}")
        
        # Set up event trackers
        package_id = self.get_parameter('package_id').value
        self.events_to_track: List[EventTracker] = [
            EventTracker(
                type=f"{package_id}::lock",
                filter=EventFilter.by_module(package_id, "lock"),
                callback=handle_lock_objects
            ),
            EventTracker(
                type=f"{package_id}::shared",
                filter=EventFilter.by_module(package_id, "shared"),
                callback=handle_escrow_objects
            )
        ]
        
        # Start the indexer
        self._start_indexer()
        
    def _validate_parameters(self):
        """Validate all parameters."""
        # Validate package_id (required)
        package_id = self.get_parameter('package_id').value
        if not package_id:
            self.get_logger().error("No package_id provided. This parameter is required.")
            rclpy.shutdown()
            return
            
        # Validate network
        network = self.get_parameter('network').value
        if network not in ['testnet', 'mainnet', 'devnet']:
            self.get_logger().error(f"Invalid network value: {network}. Must be one of: testnet, mainnet, devnet")
            rclpy.shutdown()
            return
            
        # Validate polling_interval_ms
        polling_interval = self.get_parameter('polling_interval_ms').value
        if polling_interval <= 0:
            self.get_logger().error(f"Invalid polling_interval_ms: {polling_interval}. Must be > 0")
            rclpy.shutdown()
            return
            
        # Validate default_limit
        default_limit = self.get_parameter('default_limit').value
        if not (1 <= default_limit <= 100):
            self.get_logger().error(f"Invalid default_limit: {default_limit}. Must be between 1 and 100")
            rclpy.shutdown()
            return
            
        # Validate max_retries
        max_retries = self.get_parameter('max_retries').value
        if max_retries <= 0:
            self.get_logger().error(f"Invalid max_retries: {max_retries}. Must be > 0")
            rclpy.shutdown()
            return
            
        # Validate retry_delay_ms
        retry_delay = self.get_parameter('retry_delay_ms').value
        if retry_delay <= 0:
            self.get_logger().error(f"Invalid retry_delay_ms: {retry_delay}. Must be > 0")
            rclpy.shutdown()
            return
    
    def _start_indexer(self):
        """Start the indexer in a background thread."""
        self.running = True
        self.indexer_thread = Thread(target=self._run_indexer_thread, daemon=True)
        self.indexer_thread.start()
        self.get_logger().info("🚀 Started indexer thread")
    
    def _run_indexer_thread(self):
        """Run the indexer in a separate thread with its own event loop."""
        try:
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.loop.run_until_complete(self._run_indexer())
        except Exception as e:
            self.get_logger().error(f"Fatal error in indexer thread: {e}")
            self._publish_status(False, f"Fatal error: {e}")
        finally:
            if self.loop and not self.loop.is_closed():
                self.loop.close()
    
    async def _run_indexer(self):
        """Main indexer logic running in async context."""
        try:
            await self._initialize_clients()
            await self._setup_listeners()
        except Exception as e:
            self.get_logger().error(f"Error in indexer: {e}")
            self._publish_status(False, f"Indexer error: {e}")
            raise
    
    async def _initialize_clients(self):
        """Initialize database and Sui clients."""
        try:
            self.get_logger().info("Initializing clients...")
            
            # Add workspace-generated Prisma client to Python path
            workspace_root = Path(os.getcwd())
            prisma_client_dir = workspace_root / "data" / "sui_indexer" / "prisma_client"
            if prisma_client_dir.exists():
                import sys
                sys.path.insert(0, str(prisma_client_dir))
            
            # Initialize Prisma with database URL from parameter
            self.db = Prisma(
                datasource={
                    "url": self.database_url
                }
            )
            await self.db.connect()
            self.get_logger().info("✅ Connected to database")
            
            # Initialize Sui client with proper network configuration
            network = self.get_parameter('network').value
            network_urls = {
                'devnet': 'https://fullnode.devnet.sui.io:443',
                'testnet': 'https://fullnode.testnet.sui.io:443',
                'mainnet': 'https://fullnode.mainnet.sui.io:443'
            }
            
            if network not in network_urls:
                raise ValueError(f"Invalid network {network}. Must be one of: devnet, testnet, mainnet")
                
            rpc_url = network_urls[network]
            self.sui_client = SuiClient(rpc_url)
            await self.sui_client.connect()
            self.get_logger().info(f"✅ Connected to Sui network: {network}")
            
            self._publish_status(True, "Clients initialized successfully")
            
        except Exception as e:
            self.get_logger().error(f"Client initialization error: {str(e)}")
            self._publish_status(False, f"Initialization error: {e}")
            raise
    
    async def _setup_listeners(self):
        """Set up all event listeners."""
        self.get_logger().info(f"Setting up listeners for {len(self.events_to_track)} event types")
        
        # Start a task for each event tracker
        tasks = []
        for event_tracker in self.events_to_track:
            cursor = await self._get_latest_cursor(event_tracker)
            task = asyncio.create_task(
                self._run_event_job(event_tracker, cursor),
                name=f"event_job_{event_tracker.type}"
            )
            tasks.append(task)
        
        # Wait for all tasks to complete (they run indefinitely)
        try:
            await asyncio.gather(*tasks)
        except asyncio.CancelledError:
            self.get_logger().info("Event listener tasks cancelled")
            for task in tasks:
                if not task.done():
                    task.cancel()
    
    async def _run_event_job(self, tracker: EventTracker, cursor: Optional[str]) -> None:
        """
        Run an event job continuously.
        
        Args:
            tracker: Event tracker configuration
            cursor: Starting cursor position
        """
        self.get_logger().info(f"Starting event job for {tracker.type}")
        current_cursor = cursor
        retry_count = 0
        max_retries = self.get_parameter('max_retries').value
        retry_delay_ms = self.get_parameter('retry_delay_ms').value
        polling_interval_ms = self.get_parameter('polling_interval_ms').value
        
        while self.running:
            try:
                result = await self._execute_event_job(tracker, current_cursor)
                current_cursor = result.cursor
                retry_count = 0  # Reset retry count on success
                
                # Determine sleep interval
                if result.has_next_page:
                    # No delay if there are more pages to process
                    sleep_ms = 0
                else:
                    # Use polling interval if no more pages
                    sleep_ms = polling_interval_ms
                
                if sleep_ms > 0:
                    await asyncio.sleep(sleep_ms / 1000.0)
                    
            except Exception as e:
                retry_count += 1
                self.get_logger().error(f"Error in event job {tracker.type} (attempt {retry_count}): {e}")
                self._publish_status(False, f"Error in {tracker.type}: {e}")
                
                if retry_count >= max_retries:
                    self.get_logger().error(f"Max retries exceeded for {tracker.type}, stopping job")
                    break
                
                # Exponential backoff
                sleep_ms = retry_delay_ms * (2 ** (retry_count - 1))
                self.get_logger().info(f"Retrying {tracker.type} in {sleep_ms}ms...")
                await asyncio.sleep(sleep_ms / 1000.0)
        
        self.get_logger().info(f"Event job for {tracker.type} stopped")
    
    async def _execute_event_job(
        self, 
        tracker: EventTracker, 
        cursor: Optional[str]
    ) -> EventExecutionResult:
        """
        Execute a single event job iteration.
        
        Args:
            tracker: Event tracker configuration
            cursor: Current cursor position
            
        Returns:
            EventExecutionResult with new cursor and pagination info
        """
        try:
            self.get_logger().debug(f"📡 Querying {tracker.type} events...")
            default_limit = self.get_parameter('default_limit').value
            
            # Query events from the chain using typed Extended API
            events_page: Page[SuiPySuiEvent] = await self.sui_client.extended_api.query_events(
                query=tracker.filter,
                cursor=cursor,
                limit=default_limit,
                descending_order=False
            )
            
            events = events_page.data
            self.get_logger().info(f"📥 {len(events)} events found for {tracker.type}")
            
            if events:
                self.get_logger().info(f"🎉 Processing {len(events)} new events for {tracker.type}")
                
                # Process events with the appropriate handler
                await tracker.callback(events, tracker.type, self.db)
                self.get_logger().info(f"✅ Processed {len(events)} events for {tracker.type}")
                
                # Publish events to ROS2 topic
                self._publish_events(events)
                
                # Save cursor if we processed events
                if events_page.next_cursor:
                    await self._save_latest_cursor(tracker, events_page.next_cursor)
                    
                    # Extract event sequence for status reporting
                    last_seq = 0
                    if isinstance(events_page.next_cursor, dict):
                        try:
                            last_seq = int(events_page.next_cursor.get("eventSeq", 0))
                        except (ValueError, TypeError):
                            last_seq = 0
                    
                    self._publish_status(True, f"Processed {len(events)} events for {tracker.type}", last_seq)
                else:
                    self._publish_status(True, f"Processed {len(events)} events for {tracker.type}")
            
            # Update cursor for next iteration
            next_cursor = events_page.next_cursor
            has_next_page = events_page.has_next_page
            
            return EventExecutionResult(
                cursor=next_cursor,
                has_next_page=has_next_page
            )
            
        except Exception as e:
            self.get_logger().error(f"❌ Error processing {tracker.type}: {e}")
            raise
    
    async def _get_latest_cursor(self, tracker: EventTracker) -> Optional[str]:
        """
        Get the latest cursor for an event tracker from the database.
        
        Args:
            tracker: Event tracker configuration
            
        Returns:
            Latest cursor string or None if not found
        """
        try:
            cursor_record = await self.db.cursor.find_unique(
                where={"id": tracker.type}
            )
            
            if cursor_record:
                # Reconstruct cursor from stored components
                cursor = {
                    "txDigest": cursor_record.txDigest,
                    "eventSeq": cursor_record.eventSeq
                }
                self.get_logger().info(f"Resuming {tracker.type} from cursor: {cursor}")
                return cursor
            else:
                self.get_logger().info(f"No previous cursor found for {tracker.type}, starting from beginning")
                return None
        except Exception as e:
            self.get_logger().error(f"Error getting cursor for {tracker.type}: {e}")
            return None
    
    async def _save_latest_cursor(
        self, 
        tracker: EventTracker, 
        cursor: Any
    ) -> None:
        """
        Save the latest cursor for an event tracker to the database.
        
        Args:
            tracker: Event tracker configuration
            cursor: Cursor object from the API response
        """
        if not cursor:
            return
        
        try:
            # Extract cursor components
            if isinstance(cursor, dict):
                tx_digest = cursor.get("txDigest")
                event_seq = cursor.get("eventSeq")
            else:
                # Handle string cursor or other formats
                self.get_logger().warning(f"Unexpected cursor format for {tracker.type}: {cursor}")
                return
            
            if not tx_digest or not event_seq:
                self.get_logger().warning(f"Invalid cursor data for {tracker.type}: {cursor}")
                return
            
            # Upsert cursor record using Prisma
            await self.db.cursor.upsert(
                where={"id": tracker.type},
                data={
                    "create": {
                        "id": tracker.type,
                        "txDigest": tx_digest,
                        "eventSeq": event_seq
                    },
                    "update": {
                        "txDigest": tx_digest,
                        "eventSeq": event_seq
                    }
                }
            )
            self.get_logger().debug(f"Saved cursor for {tracker.type}: {tx_digest}:{event_seq}")
        except Exception as e:
            self.get_logger().error(f"Failed to save cursor for {tracker.type}: {e}")
            raise
    
    def _publish_events(self, events: List[SuiPySuiEvent]):
        """Publish Sui events to ROS2 topic."""
        for event in events:
            try:
                ros_event = self._sui_event_to_ros_msg(event)
                self.event_pub.publish(ros_event)
            except Exception as e:
                self.get_logger().error(f"Failed to publish event {event.id}: {e}")
    
    def _sui_event_to_ros_msg(self, sui_event: SuiPySuiEvent) -> SuiEvent:
        """Convert SuiPy SuiEvent to ROS2 SuiEvent message."""
        ros_event = SuiEvent()
        
        # Set timestamp to current ROS time
        ros_event.timestamp = self.get_clock().now().to_msg()
        
        # Set event type
        ros_event.event_type = sui_event.type
        
        # Convert parsed JSON to string
        if sui_event.parsed_json:
            ros_event.event_data = json.dumps(sui_event.parsed_json)
        else:
            ros_event.event_data = "{}"
        
        # Set transaction digest and event sequence from dictionary
        ros_event.tx_digest = sui_event.id['txDigest']
        ros_event.event_seq = int(sui_event.id['eventSeq'])
        
        # Parse package_id and module_name from event type
        # Format: package_id::module_name::event_name
        parts = sui_event.type.split("::")
        if len(parts) >= 2:
            ros_event.package_id = parts[0]
            ros_event.module_name = parts[1]
        else:
            ros_event.package_id = ""
            ros_event.module_name = ""
        
        return ros_event
    
    def _publish_status(self, is_running: bool, message: str, last_seq: int = 0):
        """Publish indexer status."""
        try:
            status = IndexerStatus()
            status.timestamp = self.get_clock().now().to_msg()
            status.status = "RUNNING" if is_running else "STOPPED"
            status.message = message
            status.last_processed_seq = int(last_seq)
            self.status_pub.publish(status)
        except Exception as e:
            self.get_logger().error(f"Failed to publish status: {e}")
    
    def destroy_node(self):
        """Clean up node resources."""
        self.get_logger().info("Shutting down indexer...")
        
        # Signal indexer to stop
        self.running = False
        
        # Wait for indexer thread to finish
        if self.indexer_thread and self.indexer_thread.is_alive():
            self.get_logger().info("Waiting for indexer thread to finish...")
            self.indexer_thread.join(timeout=5.0)
            if self.indexer_thread.is_alive():
                self.get_logger().warning("Indexer thread did not finish gracefully")
        
        # Close connections using asyncio if loop exists
        if self.loop and not self.loop.is_closed():
            try:
                # Schedule cleanup and run briefly
                if self.db:
                    self.loop.run_until_complete(self.db.disconnect())
                if self.sui_client:
                    self.loop.run_until_complete(self.sui_client.close())
            except Exception as e:
                self.get_logger().error(f"Error during cleanup: {e}")
        
        self._publish_status(False, "Indexer shut down")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SuiIndexerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
