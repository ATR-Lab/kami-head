"""
Cafe event handler for the Sui Coffee Order Indexer.

This module handles CafeCreated events emitted by the coffee club contract.
"""

import logging
from datetime import datetime
from typing import Any, Dict, List

from prisma import Prisma
from sui_py import SuiEvent as SuiPySuiEvent

logger = logging.getLogger(__name__)


class CafeCreated:
    """Represents a CafeCreated event."""
    
    def __init__(self, data: Dict[str, Any]):
        self.cafe_id = data["cafe_id"]
        self.creator = data["creator"]
        # Optional fields that might be present
        self.name = data.get("name")
        self.location = data.get("location")
        self.description = data.get("description")


async def handle_cafe_events(events: List[SuiPySuiEvent], event_type: str, db: Prisma) -> None:
    """
    Handle cafe events from the coffee club contract.
    
    Args:
        events: List of SuiEvent objects to process
        event_type: Type identifier for logging
        db: Prisma database connection
    """
    logger.info(f"🏪 CAFE HANDLER: Processing {len(events)} cafe events of type {event_type}")
    
    for i, event in enumerate(events):
        logger.debug(f"🔍 Processing cafe event {i+1}/{len(events)}: {event.type}")
        
        # Validate event origin
        if not event.type.startswith(event_type):
            logger.error(f"Invalid event module origin: {event.type} does not start with {event_type}")
            raise ValueError(f"Invalid event module origin: {event.type}")
        
        # Only process CafeCreated events
        if not event.type.endswith("::CafeCreated") and "CafeCreated" not in event.type:
            logger.info(f"🚫 Skipping non-CafeCreated event: {event.type}")
            continue
        
        # Parse the event data
        if not event.parsed_json:
            logger.warning(f"Event {event.id} has no parsed JSON data, skipping")
            continue
        
        data = event.parsed_json
        logger.debug(f"📊 Cafe event data: {data}")
        
        try:
            cafe_created = CafeCreated(data)
            logger.info(f"✨ Processing cafe creation for cafe {cafe_created.cafe_id}")
            
            # Upsert cafe in database
            await db.cafe.upsert(
                where={"objectId": cafe_created.cafe_id},
                data={
                    "create": {
                        "objectId": cafe_created.cafe_id,
                        "creator": cafe_created.creator,
                        "name": cafe_created.name,
                        "location": cafe_created.location,
                        "description": cafe_created.description,
                        "status": "active",  # Default status
                        "createdAt": datetime.now(),
                    },
                    "update": {
                        "creator": cafe_created.creator,
                        "name": cafe_created.name,
                        "location": cafe_created.location,
                        "description": cafe_created.description,
                        "updatedAt": datetime.now(),
                    }
                }
            )
            
            logger.info(f"✅ Successfully processed cafe {cafe_created.cafe_id}")
            
        except KeyError as e:
            logger.error(f"❌ Missing required field in cafe event {event.id}: {e}")
            continue
        except Exception as e:
            logger.error(f"❌ Error processing cafe creation for event {event.id}: {e}")
            raise
    
    logger.info(f"🎉 Successfully processed {len(events)} cafe events") 