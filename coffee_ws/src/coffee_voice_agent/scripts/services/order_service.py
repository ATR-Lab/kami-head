"""Order notification service using WebSocket server for receiving coffee orders"""

import asyncio
import json
import logging
import threading
from typing import Callable, Optional, Dict, Any

import websockets
import websockets.server
from config.settings import WEBSOCKET_HOST, WEBSOCKET_PORT

logger = logging.getLogger(__name__)


class OrderNotificationService:
    """WebSocket server for receiving order notifications from external systems
    
    This service extracts the working WebSocket server logic and provides a clean
    callback interface for handling order notifications.
    """
    
    def __init__(self, on_order_received: Optional[Callable] = None):
        # Configuration
        self.host = WEBSOCKET_HOST
        self.port = WEBSOCKET_PORT
        
        # Callback for when orders are received
        self.on_order_received = on_order_received
        
        # Threading control (same pattern as original)
        self.websocket_thread = None
        self.websocket_active = False
        self.event_loop = None
        
        logger.info(f"OrderNotificationService initialized - Server: {self.host}:{self.port}")
    
    async def start(self):
        """Start WebSocket server for receiving order notifications (same logic as original)"""
        try:
            self.websocket_active = True
            self.event_loop = asyncio.get_event_loop()
            
            # Start WebSocket server in separate thread (same as original)
            self.websocket_thread = threading.Thread(
                target=self._websocket_server_loop,
                daemon=True
            )
            self.websocket_thread.start()
            
            logger.info(f"WebSocket server started on {self.host}:{self.port} - listening for order notifications")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start WebSocket server: {e}")
            return False
    
    def _websocket_server_loop(self):
        """WebSocket server loop running in separate thread (same logic as original)"""
        try:
            # Create new event loop for this thread (same as original)
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            # Start WebSocket server (same as original)
            async def server_main():
                async with websockets.server.serve(
                    self._handle_websocket_message,
                    self.host,
                    self.port
                ):
                    logger.info(f"🌐 WebSocket server listening on ws://{self.host}:{self.port}")
                    # Keep server running (same as original)
                    while self.websocket_active:
                        await asyncio.sleep(1)
            
            loop.run_until_complete(server_main())
            
        except Exception as e:
            logger.error(f"WebSocket server error: {e}")
        finally:
            loop.close()
    
    async def _handle_websocket_message(self, websocket, path):
        """Handle incoming WebSocket messages from indexer (same logic as original)"""
        client_info = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        logger.info(f"🌐 WebSocket client connected: {client_info}")
        
        try:
            async for message in websocket:
                try:
                    # Parse incoming message (same logic as original)
                    data = json.loads(message)
                    logger.info(f"📨 Received WebSocket message: {data}")
                    
                    # Extract order information (same logic as original)
                    order_info = self._extract_order_info(data)
                    
                    # Trigger callback if provided
                    if self.on_order_received:
                        # Use thread-safe method to trigger callback (same as original)
                        asyncio.run_coroutine_threadsafe(
                            self.on_order_received(order_info),
                            self.event_loop
                        )
                    
                    logger.info(f"✅ Processed order notification: {order_info['coffee_type']} for order {order_info['order_id']}")
                    
                    # Send confirmation back to indexer (same as original)
                    response = {
                        "status": "success",
                        "message": f"Order notification received: {order_info['coffee_type']}"
                    }
                    await websocket.send(json.dumps(response))
                    
                except json.JSONDecodeError as e:
                    logger.error(f"❌ Invalid JSON in WebSocket message: {e}")
                    error_response = {"status": "error", "message": "Invalid JSON format"}
                    await websocket.send(json.dumps(error_response))
                    
                except Exception as e:
                    logger.error(f"❌ Error processing WebSocket message: {e}")
                    error_response = {"status": "error", "message": str(e)}
                    await websocket.send(json.dumps(error_response))
                    
        except websockets.exceptions.ConnectionClosed:
            logger.info(f"🌐 WebSocket client disconnected: {client_info}")
        except Exception as e:
            logger.error(f"❌ WebSocket connection error: {e}")
    
    def _extract_order_info(self, data: Dict[str, Any]) -> Dict[str, str]:
        """Extract order information from WebSocket message (same logic as original)"""
        order_type = data.get("type", "NEW_COFFEE_REQUEST")
        order_id = data.get("order_id", "unknown")
        coffee_type = data.get("coffee_type", "coffee")
        priority = data.get("priority", "normal")
        
        # Format content for voice announcement (same logic as original)
        content = f"{coffee_type} (Order {order_id[:8]})"
        
        return {
            "type": order_type,
            "order_id": order_id,
            "coffee_type": coffee_type,
            "priority": priority,
            "content": content
        }
    
    def stop(self):
        """Stop WebSocket server (same cleanup logic as original)"""
        self.websocket_active = False
        
        if self.websocket_thread and self.websocket_thread.is_alive():
            self.websocket_thread.join(timeout=2.0)
        
        logger.info("WebSocket server stopped")
    
    def is_active(self) -> bool:
        """Check if WebSocket server is active"""
        return self.websocket_active
    
    def set_callback(self, callback: Callable):
        """Set or update the order received callback"""
        self.on_order_received = callback
        logger.info("Order notification callback updated")
    
    def __repr__(self):
        return f"OrderNotificationService(host={self.host}, port={self.port}, active={self.websocket_active})" 