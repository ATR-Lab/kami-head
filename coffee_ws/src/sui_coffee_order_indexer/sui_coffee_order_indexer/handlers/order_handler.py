"""
Order event handler for the Sui Coffee Order Indexer.

This module handles CoffeeOrderCreated and CoffeeOrderUpdated events emitted by the 
coffee club contract, processing order lifecycle events and triggering coffee machine
operations via ROS2 services when orders reach the "Processing" status.
"""

import logging
from datetime import datetime
from typing import Any, Dict, List, Optional
import threading

import rclpy
from rclpy.node import Node
from prisma import Prisma
from sui_py import SuiEvent as SuiPySuiEvent
from coffee_machine_control_msgs.srv import CoffeeCommand

logger = logging.getLogger(__name__)

# Global ROS2 service client instance (shared across handler calls)
_coffee_service_client = None
_coffee_service_node = None
_ros_initialized = False


class CoffeeOrderCreated:
    """Represents a CoffeeOrderCreated event."""
    
    def __init__(self, data: Dict[str, Any]):
        self.order_id = data["order_id"]
        
        # Extract coffee_type from Move enum object
        coffee_type_data = data["coffee_type"]
        if isinstance(coffee_type_data, dict) and "variant" in coffee_type_data:
            self.coffee_type = coffee_type_data["variant"]  # Extract "Espresso" from {'variant': 'Espresso', 'fields': {}}
        else:
            self.coffee_type = str(coffee_type_data)  # Fallback for simple strings
            
        # Additional fields might be present
        self.cafe_id = data.get("cafe_id")
        self.customer = data.get("customer")


class CoffeeOrderUpdated:
    """Represents a CoffeeOrderUpdated event."""
    
    def __init__(self, data: Dict[str, Any]):
        self.order_id = data["order_id"]
        
        # Extract status from Move enum object
        status_data = data["status"]
        if isinstance(status_data, dict) and "variant" in status_data:
            self.status = status_data["variant"]  # Extract "Processing" from {'variant': 'Processing', 'fields': {}}
        else:
            self.status = str(status_data)  # Fallback for simple strings


def _init_coffee_service_client():
    """Initialize ROS2 service client for coffee machine control."""
    global _coffee_service_client, _coffee_service_node, _ros_initialized
    
    if _coffee_service_client is not None:
        return _coffee_service_client
    
    try:
        if not _ros_initialized:
            if not rclpy.ok():
                rclpy.init()
            _ros_initialized = True
        
        # Create a minimal node for service client
        _coffee_service_node = Node('order_handler_client')
        _coffee_service_client = _coffee_service_node.create_client(CoffeeCommand, 'coffee_command')
        
        # Wait for service to be available
        if not _coffee_service_client.wait_for_service(timeout_sec=1.0):
            logger.warning("Coffee machine control service not available")
            
    except Exception as e:
        logger.error(f"Failed to initialize coffee service client: {e}")
        
    return _coffee_service_client


async def _trigger_coffee_machine(coffee_type: str) -> bool:
    """
    Trigger coffee machine via ROS2 service call.
    
    Args:
        coffee_type: Type of coffee to make (espresso, americano, etc.)
        
    Returns:
        bool: True if successful, False otherwise
    """
    client = _init_coffee_service_client()
    if not client:
        logger.error("Coffee service client not available")
        return False
    
    try:
        # Map coffee types to machine parameters
        coffee_type_map = {
            'espresso': 'espresso',
            'americano': 'americano',
            'doppio': 'doppio',
            'long': 'long',
            'coffee': 'coffee',
            'hotwater': 'hot_water'
        }
        
        machine_parameter = coffee_type_map.get(coffee_type.lower(), coffee_type.lower())
        
        # Create service request
        request = CoffeeCommand.Request()
        request.action = 'make'
        request.parameter = machine_parameter
        
        logger.info(f"☕ Triggering coffee machine: {machine_parameter}")
        
        # Call service (this is synchronous, but quick)
        future = client.call_async(request)
        
        # Spin briefly to get response
        def spin_until_future_complete():
            rclpy.spin_until_future_complete(_coffee_service_node, future, timeout_sec=5.0)
        
        # Run in thread to avoid blocking the asyncio loop
        thread = threading.Thread(target=spin_until_future_complete)
        thread.start()
        thread.join(timeout=10.0)  # Overall timeout
        
        if future.done():
            response = future.result()
            if response.success:
                logger.info(f"✅ Coffee machine triggered successfully: {response.message}")
                return True
            else:
                logger.error(f"❌ Coffee machine error: {response.message}")
                return False
        else:
            logger.error("❌ Coffee machine service call timed out")
            return False
            
    except Exception as e:
        logger.error(f"❌ Error triggering coffee machine: {e}")
        return False


async def handle_order_events(events: List[SuiPySuiEvent], event_type: str, db: Prisma) -> None:
    """
    Handle order events from the coffee club contract.
    
    Args:
        events: List of SuiEvent objects to process
        event_type: Type identifier for logging
        db: Prisma database connection
    """
    logger.info(f"☕ ORDER HANDLER: Processing {len(events)} order events of type {event_type}")
    
    for i, event in enumerate(events):
        logger.debug(f"🔍 Processing order event {i+1}/{len(events)}: {event.type}")
        
        # Validate event origin
        if not event.type.startswith(event_type):
            logger.error(f"Invalid event module origin: {event.type} does not start with {event_type}")
            raise ValueError(f"Invalid event module origin: {event.type}")
        
        # Parse the event data
        if not event.parsed_json:
            logger.warning(f"Event {event.id} has no parsed JSON data, skipping")
            continue
        
        data = event.parsed_json
        logger.debug(f"📊 Order event data: {data}")
        
        try:
            if "CoffeeOrderCreated" in event.type:
                await _handle_order_created(CoffeeOrderCreated(data), db)
            elif "CoffeeOrderUpdated" in event.type:
                await _handle_order_updated(CoffeeOrderUpdated(data), db)
            else:
                logger.info(f"🚫 Skipping unknown order event: {event.type}")
                continue
                
        except KeyError as e:
            logger.error(f"❌ Missing required field in order event {event.id}: {e}")
            continue
        except Exception as e:
            logger.error(f"❌ Error processing order event {event.id}: {e}")
            raise
    
    logger.info(f"🎉 Successfully processed {len(events)} order events")


async def _handle_order_created(order_created: CoffeeOrderCreated, db: Prisma) -> None:
    """Process a newly created coffee order."""
    logger.info(f"☕ Creating new order {order_created.order_id}")
    
    await db.coffeeorder.upsert(
        where={"objectId": order_created.order_id},
        data={
            "create": {
                "objectId": order_created.order_id,
                "status": "Created",
                "coffeeType": order_created.coffee_type,
                "createdAt": datetime.now(),
            },
            "update": {
                # Don't update existing orders on creation events
            }
        }
    )
    
    logger.info(f"✅ Order {order_created.order_id} created with coffee type: {order_created.coffee_type}")


async def _handle_order_updated(order_updated: CoffeeOrderUpdated, db: Prisma) -> None:
    """Process an updated coffee order."""
    order_id = order_updated.order_id
    new_status = order_updated.status
    logger.info(f"🔄 Processing order update for {order_id} to status {new_status}")
    
    # Get current order from database to check for duplicates and get coffee type
    order = await db.coffeeorder.find_unique(where={"objectId": order_id})
    
    if not order:
        logger.error(f"❌ Order {order_id} not found in database")
        return
    
    # Check if the order is already at this status to avoid duplicates
    if order.status == new_status:
        logger.info(f"⚠️ Order {order_id} already has status {new_status}, skipping")
        return
    
    # Update order status in database (no upsert needed - order exists!)
    await db.coffeeorder.update(
        where={"objectId": order_id},
        data={
            "status": new_status,
            "updatedAt": datetime.now(),
        }
    )
    
    logger.info(f"📊 Updated order {order_id} status to {new_status}")
    
    # If status is "Processing", trigger coffee machine using stored coffee type
    if new_status == "Processing" and order.coffeeType:
        logger.info(f"🚀 Order {order_id} is being processed - triggering coffee machine")
        success = await _trigger_coffee_machine(order.coffeeType)
        if success:
            logger.info(f"✅ Coffee machine triggered for order {order_id}")
        else:
            logger.error(f"❌ Failed to trigger coffee machine for order {order_id}")
    elif new_status == "Processing" and not order.coffeeType:
        logger.warning(f"⚠️  Order {order_id} has no coffee type - cannot trigger machine")
    
    logger.info(f"✅ Order {order_id} updated to status: {new_status}") 