#!/usr/bin/env python3

"""
LLM Client abstraction layer for the language model processor node.
Provides a consistent interface for different LLM providers.
"""

import os
import abc
import logging
from concurrent.futures import ThreadPoolExecutor
from typing import List, Dict, Any, Optional, Union

# OpenAI imports
try:
    from openai import OpenAI
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False

# Atoma SDK imports
try:
    from atoma_sdk import AtomaSDK
    from atoma_sdk.models import ChatCompletionMessage
    from atoma_sdk.types import UNSET
    ATOMA_AVAILABLE = True
except ImportError:
    ATOMA_AVAILABLE = False


class LLMClient(abc.ABC):
    """
    Abstract base class for LLM clients.
    """
    
    def __init__(self, api_key: str, model: str, logger: Optional[logging.Logger] = None):
        """
        Initialize the LLM client.
        
        Args:
            api_key: API key for the LLM provider
            model: Model name to use
            logger: Logger instance
        """
        self.api_key = api_key
        self.model = model
        self.logger = logger or logging.getLogger(__name__)
        self.client = None
    
    @abc.abstractmethod
    def initialize(self) -> bool:
        """
        Initialize the client with the API key and model.
        
        Returns:
            bool: True if initialization was successful, False otherwise
        """
        pass
    
    @abc.abstractmethod
    def generate_response(self, 
                          conversation_history: List[Dict[str, str]], 
                          functions: List[Dict[str, Any]],
                          temperature: float = 0.7,
                          max_tokens: int = 200) -> Dict[str, Any]:
        """
        Generate a response from the LLM.
        
        Args:
            conversation_history: List of conversation history dictionaries with 'role' and 'content' keys
            functions: List of function definitions for tool calling
            temperature: Temperature parameter for generation
            max_tokens: Maximum number of tokens to generate
            
        Returns:
            Dict containing response text and any tool calls
        """
        pass
    

class OpenAIClient(LLMClient):
    """
    OpenAI API client implementation.
    """
    
    def initialize(self) -> bool:
        """
        Initialize the OpenAI client.
        
        Returns:
            bool: True if initialization was successful, False otherwise
        """
        if not OPENAI_AVAILABLE:
            self.logger.error("OpenAI package not available. Please install it with 'pip install openai'")
            return False
        
        try:
            self.client = OpenAI(api_key=self.api_key)
            self.logger.info(f"OpenAI client initialized with model: {self.model}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to initialize OpenAI client: {str(e)}")
            return False
    
    def generate_response(self, 
                          conversation_history: List[Dict[str, str]], 
                          functions: List[Dict[str, Any]],
                          temperature: float = 0.7,
                          max_tokens: int = 200) -> Dict[str, Any]:
        """
        Generate a response using the OpenAI API.
        
        Args:
            conversation_history: List of conversation history dictionaries with 'role' and 'content' keys
            functions: List of function definitions for tool calling
            temperature: Temperature parameter for generation
            max_tokens: Maximum number of tokens to generate
            
        Returns:
            Dict containing response text and any tool calls
        """
        if not self.client:
            self.logger.error("OpenAI client not initialized")
            return {"text": "", "tool_calls": [], "error": "Client not initialized"}
        
        try:
            with ThreadPoolExecutor(max_workers=1) as executor:
                future = executor.submit(
                    self._make_api_call,
                    conversation_history,
                    functions,
                    temperature,
                    max_tokens
                )
                api_response = future.result(timeout=30)  # 30 second timeout
            
            # Get message from response
            message = api_response.choices[0].message
            response_text = message.content or ""
            
            # Extract tool calls if any
            tool_calls = []
            if hasattr(message, 'tool_calls') and message.tool_calls:
                self.logger.info("Function call detected in response")
                for tool_call in message.tool_calls:
                    tool_calls.append({
                        "id": tool_call.id,
                        "name": tool_call.function.name,
                        "arguments": tool_call.function.arguments
                    })
            
            return {
                "text": response_text,
                "tool_calls": tool_calls,
                "error": ""
            }
                
        except Exception as api_error:
            self.logger.error(f"OpenAI API request failed with error: {str(api_error)}")
            return {"text": "", "tool_calls": [], "error": str(api_error)}
    
    def _make_api_call(self, 
                      conversation_history: List[Dict[str, str]], 
                      functions: List[Dict[str, Any]],
                      temperature: float,
                      max_tokens: int):
        """
        Make the actual API call to OpenAI.
        
        Args:
            conversation_history: List of conversation history
            functions: List of function definitions
            temperature: Temperature parameter
            max_tokens: Maximum tokens to generate
            
        Returns:
            API response
        """
        self.logger.info(f"Making OpenAI API request with model: {self.model}")
        return self.client.chat.completions.create(
            model=self.model,
            messages=conversation_history,
            temperature=temperature,
            max_tokens=max_tokens,
            tools=functions,
            tool_choice="auto"
        )


class AtomaClient(LLMClient):
    """
    Atoma API client implementation.
    """
    
    def initialize(self) -> bool:
        """
        Initialize the Atoma client.
        
        Returns:
            bool: True if initialization was successful, False otherwise
        """
        if not ATOMA_AVAILABLE:
            self.logger.error("Atoma SDK not available. Please install it with 'pip install atoma-sdk'")
            return False
        
        try:
            self.client = AtomaSDK(bearer_auth=self.api_key)
            self.logger.info(f"Atoma client initialized with model: {self.model}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to initialize Atoma client: {str(e)}")
            return False
    
    def generate_response(self, 
                          conversation_history: List[Dict[str, str]], 
                          functions: List[Dict[str, Any]],
                          temperature: float = 0.7,
                          max_tokens: int = 200) -> Dict[str, Any]:
        """
        Generate a response using the Atoma API.
        
        Args:
            conversation_history: List of conversation history dictionaries with 'role' and 'content' keys
            functions: List of function definitions for tool calling
            temperature: Temperature parameter for generation
            max_tokens: Maximum number of tokens to generate
            
        Returns:
            Dict containing response text and any tool calls
        """
        if not self.client:
            self.logger.error("Atoma client not initialized")
            return {"text": "", "tool_calls": [], "error": "Client not initialized"}
        
        try:
            with ThreadPoolExecutor(max_workers=1) as executor:
                future = executor.submit(
                    self._make_api_call,
                    conversation_history,
                    functions,
                    temperature,
                    max_tokens
                )
                result = future.result(timeout=30)  # 30 second timeout
            
            return result
                
        except Exception as api_error:
            self.logger.error(f"Atoma API request failed with error: {str(api_error)}")
            return {"text": "", "tool_calls": [], "error": str(api_error)}
    
    def _make_api_call(self, 
                      conversation_history: List[Dict[str, str]], 
                      functions: List[Dict[str, Any]],
                      temperature: float,
                      max_tokens: int) -> Dict[str, Any]:
        """
        Make the actual API call to Atoma with streaming for tool calls.
        
        Args:
            conversation_history: List of conversation history
            functions: List of function definitions
            temperature: Temperature parameter
            max_tokens: Maximum tokens to generate
            
        Returns:
            Dict containing response text and any tool calls
        """
        self.logger.info(f"Making Atoma API request with model: {self.model}")
        
        # Convert conversation history to Atoma format
        atoma_messages = []
        for msg in conversation_history:
            atoma_messages.append(ChatCompletionMessage(
                role=msg["role"],
                content=msg["content"]
            ))
        
        # Create a streaming request to get tool calls
        stream = self.client.chat.create_stream(
            model=self.model,
            messages=atoma_messages,
            temperature=temperature,
            max_tokens=max_tokens,
            tools=functions,
            tool_choice="auto"
        )
        
        # Process the streaming response to extract tool calls and content
        response_text = ""
        tool_call_id = None
        tool_name = None
        tool_arguments = ""
        has_tool_calls = False
        
        # Use the stream to parse out the response content and any tool calls
        for chunk in stream:
            # Extract content from the chunk
            if hasattr(chunk, 'choices') and chunk.choices:
                choice = chunk.choices[0]
                if hasattr(choice, 'delta') and choice.delta:
                    delta = choice.delta
                    
                    # Extract content
                    if delta.content not in [None, "", UNSET]:
                        response_text += delta.content
                    
                    # Extract tool calls
                    if hasattr(delta, 'tool_calls') and delta.tool_calls and delta.tool_calls not in [None, "", UNSET]:
                        has_tool_calls = True
                        tool_call = delta.tool_calls[0]
                        
                        # Extract tool call ID
                        if hasattr(tool_call, 'id') and tool_call.id:
                            tool_call_id = tool_call.id
                        
                        # Extract tool name
                        if hasattr(tool_call, 'function') and hasattr(tool_call.function, 'name') and tool_call.function.name:
                            tool_name = tool_call.function.name
                        
                        # Extract tool arguments
                        if hasattr(tool_call, 'function') and hasattr(tool_call.function, 'arguments') and tool_call.function.arguments:
                            tool_arguments += tool_call.function.arguments
        
        # Prepare the tool calls list
        tool_calls = []
        if has_tool_calls and tool_name:
            tool_calls.append({
                "id": tool_call_id,
                "name": tool_name,
                "arguments": tool_arguments
            })
        
        self.logger.info("Atoma API request successful")
        return {
            "text": response_text.strip(),
            "tool_calls": tool_calls,
            "error": ""
        }


def create_llm_client(provider: str, api_key: str, model: str, logger: Optional[logging.Logger] = None) -> Optional[LLMClient]:
    """
    Factory function to create an LLM client based on the provider.
    
    Args:
        provider: LLM provider name ('openai' or 'atoma')
        api_key: API key for the provider
        model: Model name to use
        logger: Logger instance
        
    Returns:
        LLMClient instance or None if provider is not supported
    """
    if provider == 'openai':
        client = OpenAIClient(api_key, model, logger)
    elif provider == 'atoma':
        client = AtomaClient(api_key, model, logger)
    else:
        if logger:
            logger.error(f"Unsupported LLM provider: {provider}")
        return None
    
    # Initialize the client
    success = client.initialize()
    if not success:
        if logger:
            logger.error(f"Failed to initialize {provider} client")
        return None
    
    return client
