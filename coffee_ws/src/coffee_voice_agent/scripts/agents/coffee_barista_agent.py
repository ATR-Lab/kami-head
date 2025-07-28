"""Coffee Barista Agent extracted from the original monolithic implementation

This module contains the CoffeeBaristaAgent class with all the I/O services and TTS processing.
It uses the extracted StateManager for conversation state management.
"""

import asyncio
import json
import logging
import os
import threading
from datetime import datetime

import pvporcupine
import websockets
import websockets.server
from pvrecorder import PvRecorder

from livekit.agents import Agent, function_tool

from config.instructions import BARISTA_INSTRUCTIONS
from config.settings import WEBSOCKET_HOST, WEBSOCKET_PORT, VALID_EMOTIONS
from state.state_manager import StateManager, AgentState
from tools.coffee_tools import (
    get_current_time_impl, get_current_date_impl, get_coffee_menu_impl,
    get_ordering_instructions_impl, recommend_drink_impl
)

logger = logging.getLogger(__name__)


class CoffeeBaristaAgent(Agent):
    """Coffee Barista Agent for blockchain conference"""
    
    def __init__(self):
        # Initialize with instructions and programmatically registered tools
        super().__init__(
            instructions=BARISTA_INSTRUCTIONS,
            tools=[
                function_tool(
                    get_current_time_impl,
                    name="get_current_time",
                    description="Get the current time."
                ),
                function_tool(
                    get_current_date_impl,
                    name="get_current_date",
                    description="Get today's date."
                ),
                function_tool(
                    get_coffee_menu_impl,
                    name="get_coffee_menu",
                    description="Get the Sui Hub coffee menu."
                ),
                function_tool(
                    get_ordering_instructions_impl,
                    name="get_ordering_instructions",
                    description="Get instructions on how to order coffee through the Slush wallet and Coffee Hub website."
                ),
                function_tool(
                    recommend_drink_impl,
                    name="recommend_drink",
                    description="Recommend a drink based on user preference."
                ),
            ]
        )
        
        # State management
        self.state_manager = StateManager(self)
        
        # Wake word detection setup
        self.porcupine_access_key = os.getenv("PORCUPINE_ACCESS_KEY")
        self.porcupine = None
        self.recorder = None
        self.wake_word_thread = None
        self.wake_word_active = False
        self.wake_word_paused = False
        self.event_loop = None
        
        # WebSocket server setup
        self.websocket_server = None
        self.websocket_thread = None
        self.websocket_active = False
        self.connected_clients = set()  # Track connected WebSocket clients for event broadcasting
        
    async def tts_node(self, text, model_settings=None):
        """Override TTS node to process delimiter-based responses (emotion:text) with minimal buffering"""
        
        # Initialize text tracking for TTS events
        self.state_manager.current_speech_preview = ""
        self.state_manager.current_speech_full_text = ""
        preview_set = False
        
        # Process text stream with minimal buffering for emotion extraction
        async def process_text_stream():
            nonlocal preview_set
            first_chunk_buffer = ""
            emotion_extracted = False
            emotion_check_limit = 50  # Only check first 50 characters for emotion delimiter
            chunks_processed = 0
            
            async for text_chunk in text:
                if not text_chunk:
                    continue

                chunks_processed += 1
                
                # Only buffer and check for emotion in the very first chunk(s)
                if not emotion_extracted and len(first_chunk_buffer) < emotion_check_limit:
                    first_chunk_buffer += text_chunk
                    
                    # Check if we have delimiter in the buffered portion
                    if ":" in first_chunk_buffer:
                        logger.info("🔍 DEBUG: Found delimiter in first chunk(s)! Extracting emotion...")
                        logger.info(f"🔍 DEBUG: Full buffer for splitting: '{first_chunk_buffer}'")
                        
                        # Split on first colon
                        parts = first_chunk_buffer.split(":", 1)
                        emotion = parts[0].strip()
                        text_after_delimiter = parts[1] if len(parts) > 1 else ""
                        
                        # Validate and process the emotion
                        if emotion in VALID_EMOTIONS:
                            if emotion != self.state_manager.current_emotion:
                                logger.info(f"🎭 Emotion transition: {self.state_manager.current_emotion} → {emotion}")
                                self.state_manager.log_animated_eyes(emotion)
                                self.state_manager.current_emotion = emotion
                            
                            logger.info(f"🎭 Agent speaking with emotion: {emotion}")
                        else:
                            logger.warning(f"Invalid emotion '{emotion}', keeping current emotion")
                        
                        # Mark emotion as extracted
                        emotion_extracted = True
                        
                        # Immediately yield the text part (no more buffering)
                        if text_after_delimiter.strip():
                            logger.info(f"💬 TTS streaming text immediately: {text_after_delimiter[:30]}{'...' if len(text_after_delimiter) > 30 else ''}")
                            # Accumulate text and set preview
                            self.state_manager.current_speech_full_text += text_after_delimiter
                            if not preview_set:
                                self.state_manager.current_speech_preview = text_after_delimiter[:50] + "..." if len(text_after_delimiter) > 50 else text_after_delimiter
                                preview_set = True
                            yield text_after_delimiter
                        else:
                            logger.warning("🔍 DEBUG: text_after_delimiter is empty or whitespace - nothing to yield!")
                        
                    elif len(first_chunk_buffer) >= emotion_check_limit:
                        # Reached limit without finding delimiter - give up and stream everything
                        logger.info("🔍 DEBUG: No delimiter found within limit, streaming everything with default emotion")
                        
                        # Use default emotion
                        emotion = "friendly"
                        if emotion != self.state_manager.current_emotion:
                            logger.info(f"🎭 Using fallback emotion: {emotion}")
                            self.state_manager.log_animated_eyes(emotion)
                            self.state_manager.current_emotion = emotion
                        
                        emotion_extracted = True
                        
                        # Yield the buffered content immediately
                        logger.info(f"💬 TTS fallback streaming: {first_chunk_buffer[:30]}{'...' if len(first_chunk_buffer) > 30 else ''}")
                        # Accumulate text and set preview
                        self.state_manager.current_speech_full_text += first_chunk_buffer
                        if not preview_set:
                            self.state_manager.current_speech_preview = first_chunk_buffer[:50] + "..." if len(first_chunk_buffer) > 50 else first_chunk_buffer
                            preview_set = True
                        yield first_chunk_buffer
                    
                    # If we haven't extracted emotion yet and haven't hit limit, continue buffering
                    # (don't yield anything yet)
                    
                else:
                    # Either emotion already extracted, or we're past the check limit
                    # Stream everything immediately
                    # Accumulate text and set preview if not set
                    self.state_manager.current_speech_full_text += text_chunk
                    if not preview_set:
                        self.state_manager.current_speech_preview = text_chunk[:50] + "..." if len(text_chunk) > 50 else text_chunk
                        preview_set = True
                    yield text_chunk
        
        # Process the text stream and pass clean text to default TTS
        processed_text = process_text_stream()
        
        # Use default TTS implementation with processed text
        async for audio_frame in Agent.default.tts_node(self, processed_text, model_settings):
            yield audio_frame
        
        logger.info("🔍 DEBUG: tts_node processing complete")

    async def start_wake_word_detection(self, room):
        """Start wake word detection in a separate thread"""
        if not self.porcupine_access_key:
            logger.info("No Porcupine access key found, skipping wake word detection")
            return
            
        try:
            # Initialize Porcupine with "hey barista" wake word
            self.porcupine = pvporcupine.create(
                access_key=self.porcupine_access_key,
                keywords=["hey barista"]
            )
            
            # Initialize recorder
            self.recorder = PvRecorder(
                device_index=-1,  # default device
                frame_length=self.porcupine.frame_length
            )
            
            self.wake_word_active = True
            self.event_loop = asyncio.get_event_loop()
            
            # Start wake word detection in separate thread
            self.wake_word_thread = threading.Thread(
                target=self._wake_word_detection_loop,
                args=(room,),
                daemon=True
            )
            self.wake_word_thread.start()
            
            logger.info("Wake word detection started - listening for 'hey barista'")
            
        except Exception as e:
            logger.error(f"Failed to start wake word detection: {e}")
    
    def _wake_word_detection_loop(self, room):
        """Wake word detection loop running in separate thread"""
        try:
            self.recorder.start()
            
            while self.wake_word_active:
                if self.wake_word_paused:
                    # Sleep briefly when paused to avoid busy waiting
                    threading.Event().wait(0.1)
                    continue
                    
                pcm = self.recorder.read()
                result = self.porcupine.process(pcm)
                
                if result >= 0:  # Wake word detected
                    logger.info("Wake word 'hey barista' detected!")
                    
                    # Use thread-safe method to trigger conversation
                    asyncio.run_coroutine_threadsafe(
                        self.activate_conversation(room), 
                        self.event_loop
                    )
                    
        except Exception as e:
            logger.error(f"Wake word detection error: {e}")
        finally:
            if self.recorder:
                self.recorder.stop()
    
    async def activate_conversation(self, room):
        """Activate conversation when wake word is detected"""
        logger.info("🔍 DEBUG: activate_conversation called")
        
        if self.wake_word_paused:
            logger.info("🔍 DEBUG: Conversation already active, ignoring wake word")
            return
            
        self.wake_word_paused = True  # Pause wake word detection during conversation
        
        logger.info("🔍 DEBUG: Activating conversation mode")
        
        try:
            # Transition to connecting state
            await self.state_manager.transition_to_state(AgentState.CONNECTING)
            
            # Create new session
            session = await self.state_manager.create_session(self)
            
            # Transition to active state
            await self.state_manager.transition_to_state(AgentState.ACTIVE)
            
            # Get random greeting from pool
            greeting = self.state_manager.get_random_greeting()
            
            logger.info("🔍 DEBUG: About to call process_emotional_response and say_with_emotion (MANUAL TTS)")
            # Process the emotional response
            emotion, text = self.state_manager.process_emotional_response(greeting)
            await self.state_manager.say_with_emotion(text, emotion)
            logger.info("🔍 DEBUG: Manual TTS call completed")
                
        except Exception as e:
            logger.error(f"Error activating conversation: {e}")
            # Return to dormant state on error
            await self.state_manager.transition_to_state(AgentState.DORMANT)
            self.wake_word_paused = False

    def stop_wake_word_detection(self):
        """Stop wake word detection"""
        self.wake_word_active = False
        self.wake_word_paused = False
        
        if self.wake_word_thread and self.wake_word_thread.is_alive():
            self.wake_word_thread.join(timeout=2.0)
            
        if self.recorder:
            try:
                self.recorder.stop()
                self.recorder.delete()
            except:
                pass
                
        if self.porcupine:
            try:
                self.porcupine.delete()
            except:
                pass
        
        logger.info("Wake word detection stopped")

    async def start_websocket_server(self):
        """Start WebSocket server for receiving order notifications"""
        try:
            self.websocket_active = True
            self.event_loop = asyncio.get_event_loop()
            
            # Start WebSocket server in separate thread
            self.websocket_thread = threading.Thread(
                target=self._websocket_server_loop,
                daemon=True
            )
            self.websocket_thread.start()
            
            logger.info(f"WebSocket server started on {WEBSOCKET_HOST}:{WEBSOCKET_PORT} - listening for order notifications")
            
        except Exception as e:
            logger.error(f"Failed to start WebSocket server: {e}")

    def _websocket_server_loop(self):
        """WebSocket server loop running in separate thread"""
        try:
            # Create new event loop for this thread
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            # Start WebSocket server
            async def server_main():
                async with websockets.server.serve(
                    self._handle_websocket_message,
                    WEBSOCKET_HOST,
                    WEBSOCKET_PORT
                ):
                    logger.info(f"🌐 WebSocket server listening on ws://{WEBSOCKET_HOST}:{WEBSOCKET_PORT}")
                    # Keep server running
                    while self.websocket_active:
                        await asyncio.sleep(1)
            
            loop.run_until_complete(server_main())
            
        except Exception as e:
            logger.error(f"WebSocket server error: {e}")
        finally:
            loop.close()

    async def _handle_websocket_message(self, websocket, path):
        """Handle incoming WebSocket messages from indexer"""
        client_info = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        logger.info(f"🌐 WebSocket client connected: {client_info}")
        
        # Add client to connected clients set for event broadcasting
        self.connected_clients.add(websocket)
        
        # Send startup event to test bidirectional communication
        startup_event = {
            "timestamp": datetime.now().isoformat(),
            "message": "Voice agent ready for events",
            "version": "refactored"
        }
        await self._send_websocket_event("STARTUP", startup_event)
        
        try:
            async for message in websocket:
                try:
                    # Parse incoming message
                    data = json.loads(message)
                    logger.info(f"📨 Received WebSocket message: {data}")
                    
                    # Extract order information
                    order_type = data.get("type", "NEW_COFFEE_REQUEST")
                    order_id = data.get("order_id", "unknown")
                    coffee_type = data.get("coffee_type", "coffee")
                    priority = data.get("priority", "normal")
                    
                    # Format content for voice announcement
                    content = f"{coffee_type} (Order {order_id[:8]})"
                    
                    # Queue virtual request using thread-safe method
                    # Use run_coroutine_threadsafe for async function calls
                    asyncio.run_coroutine_threadsafe(
                        self.state_manager.queue_virtual_request(order_type, content, priority),
                        self.event_loop
                    )
                    
                    logger.info(f"✅ Queued order notification: {coffee_type} for order {order_id}")
                    
                    # Send confirmation back to indexer
                    response = {
                        "type": "ACKNOWLEDGMENT",
                        "status": "success",
                        "message": f"Order notification received: {coffee_type}"
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
        finally:
            # Remove client from connected clients set
            self.connected_clients.discard(websocket)

    def stop_websocket_server(self):
        """Stop WebSocket server"""
        self.websocket_active = False
        
        if self.websocket_thread and self.websocket_thread.is_alive():
            self.websocket_thread.join(timeout=2.0)
        
        logger.info("WebSocket server stopped")

    async def _send_websocket_event(self, event_type: str, event_data: dict):
        """Send event to all connected WebSocket clients (like the ROS2 bridge)"""
        if not self.connected_clients:
            logger.debug(f"No WebSocket clients connected - skipping {event_type} event")
            return
            
        # Format event message
        message = {
            "type": event_type,
            "data": event_data
        }
        
        # Send to all connected clients
        disconnected_clients = set()
        for client in self.connected_clients.copy():
            try:
                await client.send(json.dumps(message))
                logger.debug(f"Sent {event_type} event to WebSocket client")
            except Exception as e:
                logger.error(f"Failed to send {event_type} event to client: {e}")
                disconnected_clients.add(client)
        
        # Remove disconnected clients
        self.connected_clients -= disconnected_clients 