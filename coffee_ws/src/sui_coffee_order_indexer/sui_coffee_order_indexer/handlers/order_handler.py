"""
Order event handler for the Sui Coffee Order Indexer.

This module handles CoffeeOrderCreated and CoffeeOrderUpdated events emitted by the 
coffee club contract, processing order lifecycle events and storing them in the database.

Coffee machine integration is handled by separate controller nodes that subscribe to 
the indexer's published events, following proper ROS2 architectural patterns.
"""

import logging
from datetime import datetime
from typing import Any, Dict, List, Optional

from prisma import Prisma
from sui_py import SuiEvent as SuiPySuiEvent

logger = logging.getLogger(__name__)


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
    
    # PHASE 1 ARCHITECTURAL CHANGE: Removed coffee machine service calls
    # Coffee machine integration is now handled by separate controller nodes
    # that subscribe to the indexer's published ROS2 events.
    #
    # This eliminates threading conflicts and follows proper ROS2 patterns:
    # Indexer (blockchain events) → ROS2 Topics → Coffee Controller → Coffee Machine
    
    if new_status == "Processing" and order.coffeeType:
        logger.info(f"☕ Order {order_id} ({order.coffeeType}) ready for coffee machine processing")
        logger.info(f"📢 Event will be published to ROS2 topics for coffee controller to handle")
    elif new_status == "Processing" and not order.coffeeType:
        logger.warning(f"⚠️  Order {order_id} has no coffee type specified")
    
    logger.info(f"✅ Order {order_id} updated to status: {new_status}") 