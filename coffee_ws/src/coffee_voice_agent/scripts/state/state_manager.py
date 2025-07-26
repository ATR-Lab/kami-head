"""State management for the coffee barista agent

This module contains the StateManager class extracted from the original monolithic implementation.
It handles conversation state transitions, timeout management, and virtual request processing.
"""

import asyncio
import logging
import json
import os
from datetime import datetime
from enum import Enum

from livekit.agents import AgentSession
from livekit.plugins import openai, silero

from config.settings import (
    USER_RESPONSE_TIMEOUT, FINAL_TIMEOUT, MAX_CONVERSATION_TIME,
    VALID_EMOTIONS
)

logger = logging.getLogger(__name__)


class AgentState(Enum):
    DORMANT = "dormant"           # Only wake word detection active
    CONNECTING = "connecting"     # Creating LiveKit session  
    ACTIVE = "active"            # Conversation mode
    SPEAKING = "speaking"        # Agent is speaking
    DISCONNECTING = "disconnecting" # Cleaning up session


class StateManager:
    def __init__(self, agent=None):
        self.current_state = AgentState.DORMANT
        self.state_lock = asyncio.Lock()
        self.session = None
        self.conversation_timer = None  # Max conversation time limit
        self.user_response_timer = None  # Timer for waiting for user response
        self.ctx = None
        self.agent = agent
        self.current_emotion = "waiting"  # Track current emotional state
        self.emotion_history = []  # Log emotional journey
        self.ending_conversation = False  # Flag to prevent timer conflicts during goodbye
        self.virtual_request_queue = []  # Queue for virtual coffee requests
        self.announcing_virtual_request = False  # Flag to prevent conflicts during announcements
        self.recent_greetings = []  # Track recent greetings to avoid repetition
        self.interaction_count = 0  # Track number of interactions for familiarity
        
        # Phase 4: Mutual exclusion for virtual request processing
        self.virtual_request_processing_lock = asyncio.Lock()
        self.batch_timer = None  # Timer for batching rapid requests
        self.batch_window_duration = 2.0  # seconds to collect requests into a batch

    async def transition_to_state(self, new_state: AgentState):
        """Handle state transitions with proper cleanup"""
        async with self.state_lock:
            if self.current_state == new_state:
                return
                
            logger.info(f"State transition: {self.current_state.value} → {new_state.value}")
            await self._exit_current_state()
            self.current_state = new_state
            await self._enter_new_state()

    async def _exit_current_state(self):
        """Clean up current state"""
        if self.current_state == AgentState.ACTIVE:
            # Cancel conversation timers
            if self.conversation_timer:
                self.conversation_timer.cancel()
                self.conversation_timer = None
            if self.user_response_timer:
                self.user_response_timer.cancel()
                self.user_response_timer = None
            # Reset conversation ending flag
            self.ending_conversation = False
        
        # Cancel batch timer if transitioning away from DORMANT
        if self.current_state == AgentState.DORMANT and self.batch_timer:
            self.batch_timer.cancel()
            self.batch_timer = None
            logger.info("🔍 Cancelled batch collection timer during state transition")

    async def _enter_new_state(self):
        """Initialize new state"""
        if self.current_state == AgentState.ACTIVE:
            # Start max conversation timer (absolute limit)
            self.conversation_timer = asyncio.create_task(self._max_conversation_timeout())
        elif self.current_state == AgentState.DORMANT:
            # Resume wake word detection when returning to dormant
            if self.agent:
                self.agent.wake_word_paused = False
                logger.info("Resumed wake word detection")

    async def _max_conversation_timeout(self):
        """Handle maximum conversation time limit"""
        try:
            await asyncio.sleep(MAX_CONVERSATION_TIME)  # 5 minute absolute limit
            if self.session and self.current_state == AgentState.ACTIVE:
                logger.info("Maximum conversation time reached - ending session")
                
                # Set ending flag to prevent timer conflicts
                self.ending_conversation = True
                
                # Timeout message with sleepy emotion using delimiter format
                timeout_response = "sleepy:We've been chatting for a while! I'm getting a bit sleepy. Thanks for the conversation. Say 'hey barista' if you need me again."
                emotion, text = self.process_emotional_response(timeout_response)
                await self.say_with_emotion(text, emotion)
                
                await asyncio.sleep(2)
                await self.end_conversation()
        except asyncio.CancelledError:
            pass

    async def _wait_for_user_response(self):
        """Wait for user response after agent speaks"""
        try:
            # Wait for user to respond, but check for virtual requests during pause
            pause_duration = 0
            check_interval = 2  # Check every 2 seconds
            
            while pause_duration < USER_RESPONSE_TIMEOUT:
                await asyncio.sleep(check_interval)
                pause_duration += check_interval
                
                # Check for virtual requests during the pause
                if self.virtual_request_queue and not self.announcing_virtual_request:
                    # Process virtual request during conversation pause
                    await self._process_virtual_request_during_conversation()
                    # Reset pause duration after announcement
                    pause_duration = 0
            
            if self.session and self.current_state == AgentState.ACTIVE:
                # Polite prompt with curious emotion

                prompt = "curious:Is there anything else I can help you with?"
                emotion, text = self.process_emotional_response(prompt)
                await self.say_with_emotion(text, emotion)
                
                # Wait a bit more
                await asyncio.sleep(FINAL_TIMEOUT)
                
                if self.session and self.current_state == AgentState.ACTIVE:
                    # Set ending flag to prevent timer conflicts
                    self.ending_conversation = True
                    
                    # End conversation with friendly emotion using delimiter format
                    goodbye_response = "friendly:Thanks for chatting! Say 'hey barista' if you need me again."
                    emotion, text = self.process_emotional_response(goodbye_response)
                    await self.say_with_emotion(text, emotion)
                    
                    await asyncio.sleep(2)
                    await self.end_conversation()
                
        except asyncio.CancelledError:
            pass  # User spoke, timer cancelled

    async def create_session(self, agent) -> AgentSession:
        """Create new session when wake word detected"""
        if self.session:
            await self.destroy_session()
            
        self.session = AgentSession(
            stt=openai.STT(model="whisper-1"),
            llm=openai.LLM(
                model="gpt-4o-mini",
                temperature=float(os.getenv("VOICE_AGENT_TEMPERATURE", "0.7"))
            ),
            tts=openai.TTS(
                model="tts-1",
                voice=os.getenv("VOICE_AGENT_VOICE", "nova")
            ),
            vad=silero.VAD.load(),
        )
        
        # Set up session event handlers using correct LiveKit 1.0+ events
        @self.session.on("conversation_item_added")
        def on_conversation_item_added(event):
            """Handle conversation items being added - detect user goodbye and manage conversation flow"""
            logger.info("🔍 DEBUG: conversation_item_added event fired!")
            logger.info(f"🔍 DEBUG: event type: {type(event)}")
            logger.info(f"🔍 DEBUG: item role: {event.item.role}")
            logger.info(f"🔍 DEBUG: item text: {event.item.text_content}")
            
            async def handle_conversation_item():
                try:
                    # Handle user messages for goodbye detection
                    if event.item.role == "user":
                        # Cancel user response timer when user speaks
                        if self.user_response_timer:
                            self.user_response_timer.cancel()
                            self.user_response_timer = None
                        
                        # Check for goodbye in user text
                        user_text = event.item.text_content or ""
                        text_lower = user_text.lower()
                        logger.info(f"🔍 DEBUG: User said: '{user_text}'")
                        
                        goodbye_words = ['goodbye', 'thanks', 'that\'s all', 'see you', 'bye']
                        
                        if any(word in text_lower for word in goodbye_words):
                            logger.info("🔍 DEBUG: User indicated conversation ending - goodbye detected!")
                            # Set ending flag to prevent timer conflicts
                            self.ending_conversation = True
                            
                            # Let agent say goodbye before ending conversation
                            goodbye_response = "friendly:Thanks for chatting! Say 'hey barista' if you need me again."
                            emotion, text = self.process_emotional_response(goodbye_response)
                            await self.say_with_emotion(text, emotion)
                            
                            # Wait for goodbye to finish, then end conversation
                            await asyncio.sleep(3)  # Give time for TTS to complete
                            await self.end_conversation()
                        else:
                            logger.info(f"🔍 DEBUG: No goodbye detected in: '{user_text}'")
                    
                    # Handle agent messages for timer management
                    elif event.item.role == "assistant":
                        logger.info("🔍 DEBUG: Agent message added to conversation")
                        
                        # Only start new timer if we're not ending the conversation
                        if not self.ending_conversation:
                            # Start timer to wait for user response after agent speaks
                            if self.user_response_timer:
                                self.user_response_timer.cancel()
                            
                            self.user_response_timer = asyncio.create_task(self._wait_for_user_response())
                            logger.info("🔍 DEBUG: Started user response timer")
                        else:
                            logger.info("🔍 DEBUG: Skipping user response timer - conversation ending")
                            
                except Exception as e:
                    logger.error(f"🔍 DEBUG: Exception in conversation_item_added handler: {e}")
                    import traceback
                    logger.error(f"🔍 DEBUG: Traceback: {traceback.format_exc()}")
                    
            asyncio.create_task(handle_conversation_item())
        
        @self.session.on("user_state_changed")
        def on_user_state_changed(event):
            """Handle user state changes (speaking/listening)"""
            logger.info(f"🔍 DEBUG: user_state_changed: {event.old_state} → {event.new_state}")
            
            if event.new_state == "speaking":
                logger.info("🔍 DEBUG: User started speaking")
            elif event.new_state == "listening":
                logger.info("🔍 DEBUG: User stopped speaking")
        
        @self.session.on("agent_state_changed")
        def on_agent_state_changed(event):
            """Handle agent state changes (initializing/listening/thinking/speaking)"""
            logger.info(f"🔍 DEBUG: agent_state_changed: {event.old_state} → {event.new_state}")
            
            if event.new_state == "speaking":
                logger.info("🔍 DEBUG: Agent started speaking")
            elif event.new_state == "listening":
                logger.info("🔍 DEBUG: Agent is listening")
            elif event.new_state == "thinking":
                logger.info("🔍 DEBUG: Agent is thinking")
        
        @self.session.on("close")
        def on_session_close(event):
            """Handle session close events"""
            logger.info("🔍 DEBUG: session close event fired!")
            if event.error:
                logger.error(f"🔍 DEBUG: Session closed with error: {event.error}")
            else:
                logger.info("🔍 DEBUG: Session closed normally")
        
        await self.session.start(agent=agent, room=self.ctx.room)
        
        # Debug session after creation
        logger.info(f"🔍 DEBUG: Session created: {self.session}")
        logger.info(f"🔍 DEBUG: Session type: {type(self.session)}")
        
        return self.session

    async def destroy_session(self):
        """Clean up session when conversation ends"""
        if self.session:
            try:
                await self.session.aclose()
            except Exception as e:
                logger.error(f"Error closing session: {e}")
            finally:
                self.session = None

    async def end_conversation(self):
        """End the current conversation and return to dormant state"""
        logger.info("🔍 DEBUG: end_conversation called - cleaning up and transitioning to dormant")
        await self.destroy_session()
        await self.transition_to_state(AgentState.DORMANT)
        
        # Process any queued virtual requests when conversation ends
        await self._process_queued_virtual_requests()
        
        logger.info("🔍 DEBUG: end_conversation completed - now in dormant state")

    async def queue_virtual_request(self, request_type: str, content: str, priority: str = "normal"):
        """Add a virtual request to the queue with batching support"""
        request = {
            "type": request_type,
            "content": content,
            "priority": priority,
            "timestamp": datetime.now()
        }
        
        # Insert based on priority (urgent requests go to front)
        if priority == "urgent":
            self.virtual_request_queue.insert(0, request)
        else:
            self.virtual_request_queue.append(request)
            
        logger.info(f"📋 Queued virtual request: {request_type} - {content} (priority: {priority}) [Queue size: {len(self.virtual_request_queue)}]")
        
        # If agent is in DORMANT state, trigger batch processing
        if self.current_state == AgentState.DORMANT:
            await self._trigger_batch_processing()
        else:
            logger.info(f"🔍 Agent in {self.current_state.value} state - request will be processed during conversation")

    async def _trigger_batch_processing(self):
        """Trigger batch processing with mutual exclusion and batching logic"""
        # Check if we're already processing (mutual exclusion)
        if self.virtual_request_processing_lock.locked():
            logger.info("🔍 Virtual request processing already in progress - request will be included in current/next batch")
            return
        
        # Check if there are urgent requests - process immediately
        has_urgent = any(req.get("priority") == "urgent" for req in self.virtual_request_queue)
        
        if has_urgent:
            logger.info("🔍 Urgent request detected - processing batch immediately")
            # Cancel existing batch timer if running
            if self.batch_timer:
                self.batch_timer.cancel()
                self.batch_timer = None
            # Process immediately
            await self._process_batch_with_lock()
        else:
            # If no batch timer is running, start one for normal priority requests
            if self.batch_timer is None:
                logger.info("🔍 Starting batch collection timer for normal priority requests")
                self.batch_timer = asyncio.create_task(self._batch_collection_timer())

    async def _batch_collection_timer(self):
        """Collect requests for a time window, then process them as a batch"""
        try:
            # Wait for batch collection window
            await asyncio.sleep(self.batch_window_duration)
            
            # Process collected batch
            await self._process_batch_with_lock()
            
        except asyncio.CancelledError:
            logger.info("🔍 Batch collection timer cancelled")
        finally:
            self.batch_timer = None

    async def _process_batch_with_lock(self):
        """Process all queued virtual requests as a single batch with mutual exclusion"""
        async with self.virtual_request_processing_lock:
            if not self.virtual_request_queue:
                logger.info("🔍 No virtual requests to process in batch")
                return
            
            if self.current_state != AgentState.DORMANT:
                logger.info("🔍 Agent no longer in DORMANT state - skipping batch processing")
                return
            
            logger.info(f"🔍 Processing batch of {len(self.virtual_request_queue)} virtual requests")
            
            # Create single session for entire batch
            temp_session = None
            try:
                if not self.session:
                    temp_session = await self.create_session(self.agent)
                    logger.info("🔍 Created temporary session for batch processing")
                
                # Process all requests in the batch (back to front for stable indices)
                batch_size = len(self.virtual_request_queue)
                for i in range(batch_size - 1, -1, -1):
                    try:
                        # Get request and remove by index (avoids race condition)
                        request = self.virtual_request_queue[i]
                        del self.virtual_request_queue[i]
                        
                        # Announce the virtual request
                        announcement = self._format_virtual_request_announcement(request)
                        logger.info(f"🔍 DEBUG: BATCH - Raw announcement: '{announcement}'")
                        emotion, text = self.process_emotional_response(announcement)
                        logger.info(f"🔍 DEBUG: BATCH - Processed emotion: '{emotion}', text: '{text}'")
                        await self.say_with_emotion(text, emotion)
                        
                        logger.info(f"📢 Announced batch request {batch_size-i}/{batch_size}: {request['type']}")
                        
                        # Small delay between announcements in the same batch
                        if i > 0:
                            await asyncio.sleep(1.0)
                        
                    except Exception as e:
                        logger.error(f"Error processing batch request {request['type']}: {e}")
                
                # Final pause before cleaning up session
                await asyncio.sleep(1.0)
                
            except Exception as e:
                logger.error(f"Error in batch processing: {e}")
            finally:
                # Clean up temporary session
                if temp_session and self.session:
                    await self.destroy_session()
                    logger.info("🔍 Cleaned up temporary batch processing session")
                
                logger.info(f"🔍 Batch processing completed - {len(self.virtual_request_queue)} requests remaining in queue")

    async def _process_virtual_request_during_conversation(self):
        """Process a virtual request during conversation pause"""
        if not self.virtual_request_queue or self.announcing_virtual_request:
            return
            
        self.announcing_virtual_request = True
        
        try:
            request = self.virtual_request_queue.pop(0)
            
            # Brief polite interruption
            # excuse_msg = "excuse:Oh, excuse me one moment..."
            # emotion, text = self.process_emotional_response(excuse_msg)
            # await self.say_with_emotion(text, emotion)
            
            # await asyncio.sleep(5)
            
            # Announce the virtual request
            announcement = self._format_virtual_request_announcement(request)
            emotion, text = self.process_emotional_response(announcement)
            await self.say_with_emotion(text, emotion)
            
            await asyncio.sleep(1)
            
            # # Resume conversation
            # resume_msg = "friendly:Now, where were we?"
            # emotion, text = self.process_emotional_response(resume_msg)
            # await self.say_with_emotion(text, emotion)
            
            logger.info(f"📢 Announced virtual request during conversation: {request['type']}")
            
        except Exception as e:
            logger.error(f"Error processing virtual request during conversation: {e}")
        finally:
            self.announcing_virtual_request = False

    async def _process_queued_virtual_requests(self):
        """Legacy method - now delegates to new batch processing system"""
        logger.info("🔍 Legacy _process_queued_virtual_requests called - delegating to batch processing")
        await self._process_batch_with_lock()

    def _format_virtual_request_announcement(self, request: dict) -> str:
        """Format virtual request as emotional announcement"""
        from utils.announcement_data import format_virtual_request_announcement
        
        return format_virtual_request_announcement(request)

    def process_emotional_response(self, llm_response: str) -> tuple[str, str]:
        """Process LLM response and extract emotion + text from delimiter format (emotion:text)"""
        try:
            # Check if response uses delimiter format (emotion:text)
            if ":" in llm_response:
                # Split on first colon
                parts = llm_response.split(":", 1)
                emotion = parts[0].strip()
                text = parts[1].strip() if len(parts) > 1 else ""
                
                logger.info(f"🔍 DEBUG: Delimiter format detected - emotion: '{emotion}', text: '{text[:50]}{'...' if len(text) > 50 else ''}'")
                
            else:
                # Try to parse as JSON (legacy format)
                try:
                    response_data = json.loads(llm_response)
                    emotion = response_data.get("emotion", "friendly")
                    text = response_data.get("text", "")
                    logger.info("🔍 DEBUG: JSON format detected (legacy)")
                except json.JSONDecodeError:
                    # Fallback: treat entire response as text with default emotion
                    logger.warning("No delimiter or JSON format found, using fallback")
                    emotion = "friendly"
                    text = llm_response
            
            # Validate emotion is in our supported set
            if emotion not in VALID_EMOTIONS:
                logger.warning(f"Unknown emotion '{emotion}', defaulting to 'friendly'")
                emotion = "friendly"
            
            # Log emotion transition
            if emotion != self.current_emotion:
                logger.info(f"🎭 Emotion transition: {self.current_emotion} → {emotion}")
                self.log_animated_eyes(emotion)
                self.current_emotion = emotion
                
                # Store in emotion history
                self.emotion_history.append({
                    'timestamp': datetime.now(),
                    'emotion': emotion,
                    'text_preview': text[:50] + "..." if len(text) > 50 else text
                })
            
            return emotion, text
            
        except Exception as e:
            logger.error(f"Error processing emotional response: {e}")
            return "friendly", llm_response

    def log_animated_eyes(self, emotion: str):
        """Log how this emotion would appear as animated eyes"""
        from utils.animation_data import log_animated_eyes
        
        log_animated_eyes(emotion)

    async def say_with_emotion(self, text: str, emotion: str = None):
        """Speak text and log emotional context"""
        logger.info("🔍 DEBUG: say_with_emotion called - MANUAL TTS PATHWAY")
        logger.info(f"🔍 DEBUG: say_with_emotion text: '{text[:100]}{'...' if len(text) > 100 else ''}'")
        logger.info(f"🔍 DEBUG: say_with_emotion emotion: {emotion}")
        
        if self.session:
            # Send TTS_STARTED event
            await self._send_tts_event("started", text, emotion or self.current_emotion, "manual")
            
            logger.info("🔍 DEBUG: Calling session.say() directly (bypasses llm_node)")
            handle = await self.session.say(text)
            
            # Wait for TTS completion
            await handle.wait_for_playout()
            
            # Send TTS_FINISHED event
            await self._send_tts_event("finished", text, emotion or self.current_emotion, "manual")
            
            if emotion:
                logger.info(f"🎭 Speaking with emotion: {emotion}")
                logger.info(f"💬 Text: {text[:100]}{'...' if len(text) > 100 else ''}")
        else:
            logger.error("🔍 DEBUG: No session available for speaking")

    def get_random_greeting(self) -> str:
        """Get a random greeting from the greeting pool"""
        from utils.greeting_data import get_random_greeting
        
        return get_random_greeting()

    async def _send_tts_event(self, event: str, text: str, emotion: str, source: str):
        """Send TTS event through agent's WebSocket connection"""
        if self.agent and hasattr(self.agent, '_send_websocket_event'):
            event_data = {
                "event": event,
                "text": text[:100] + "..." if len(text) > 100 else text,  # Truncate long text
                "emotion": emotion,
                "source": source,
                "timestamp": datetime.now().isoformat()
            }
            await self.agent._send_websocket_event("TTS_EVENT", event_data)
        else:
            logger.debug(f"Cannot send TTS {event} event - no agent WebSocket connection") 