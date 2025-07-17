#!/usr/bin/env python3

"""
Intent classification using Ollama LLM.

This module handles classification of user intents from text prompts
using local LLM models through Ollama.
"""

import re
import logging
import time

from coffee_speech_processing.constants import (INTENT_MAPPING_STRING_TO_BYTE, DEFAULT_INTENT)

# Import for Ollama
try:
    import ollama
    OLLAMA_AVAILABLE = True
except ImportError:
    OLLAMA_AVAILABLE = False

logger = logging.getLogger(__name__)


class IntentClassifier:
    """
    Classifies user intents using a local LLM through Ollama.
    
    This class handles initialization of the LLM, intent classification,
    and fallback mechanisms when classification fails.
    """
    
    def __init__(self, model='gemma3:1b', timeout=3.0, max_retries=1):
        """
        Initialize the intent classifier.
        
        Args:
            model (str): Ollama LLM model for intent classification
            timeout (float): Timeout in seconds for LLM classification
            max_retries (int): Number of retries for LLM classification before falling back
        """
        if not OLLAMA_AVAILABLE:
            raise ImportError("Ollama is not available. Install it with 'pip install ollama'")
        
        self.model = model
        self.timeout = timeout
        self.max_retries = max_retries
        self.is_ready = False
        
        # System prompt for intent classification
        self.system_prompt = (
            "You are a classifier model designed strictly to classify the intent of the user's input. "
            "Here is the list of intent labels with their corresponding numbers: "
            f"{INTENT_MAPPING_STRING_TO_BYTE}"
            "Your task is to output ONLY the number that represents the intent. "
            "Do NOT include any words, punctuation, or explanation. "
            "Output format: A single number (e.g., 3). "
            f"If unsure, always default to '{DEFAULT_INTENT}' (Fallback) (e.g., {DEFAULT_INTENT}). "
            "Only respond with the number."
        )
        
        # Initialize the LLM
        self.initialize()
    
    def initialize(self):
        """
        Initialize the LLM classifier.
        
        Returns:
            bool: True if initialized successfully, False otherwise
        """
        try:
            # Check if the Ollama model is available
            if not self.ensure_model_available():
                logger.error("Failed to ensure Ollama model availability, LLM classification disabled")
                return False
            
            logger.info(f"LLM classifier initialized with model {self.model}")
            self.is_ready = True
            return True
        except Exception as e:
            logger.error(f"Error initializing LLM: {str(e)}")
            return False
    
    def ensure_model_available(self):
        """
        Check if the Ollama model is available, and pull it if not.
        
        Returns:
            bool: True if model is available, False otherwise
        """
        logger.info(f'Checking if Ollama model {self.model} is available...')
        try:
            # Check if model is available by listing models
            models = ollama.list()
            model_names = [model['model'] for model in models.get('models', [])]
            
            if self.model in model_names:
                logger.info(f'Ollama model {self.model} is available')
                return True
            
            # Model not found, we need to pull it
            logger.info(f'Ollama model {self.model} not found, pulling...')
            ollama.pull(self.model)
            logger.info(f'Successfully pulled Ollama model {self.model}')
            return True
        except Exception as e:
            logger.error(f'Failed to ensure Ollama model availability: {str(e)}')
            return False
    
    def classify(self, prompt_text):
        """
        Classify user intent with the LLM.
        
        Args:
            prompt_text (str): Text prompt to classify
            
        Returns:
            tuple: (intent_code, success_flag)
                intent_code (bytes): Intent code as bytes (single byte)
                success_flag (bool): Whether classification was successful
        """
        if not self.is_ready:
            logger.warning("LLM not ready, using fallback classification")
            return bytes([5]), False  # Fallback intent as bytes
        
        # Number of retries
        retries = 0
        
        while retries <= self.max_retries:
            try:
                logger.info(f"Classifying intent with LLM, attempt {retries+1}/{self.max_retries+1}")
                
                # Call Ollama API with timeout
                response = ollama.chat(
                    model=self.model,
                    messages=[
                        {"role": "system", "content": self.system_prompt},
                        {"role": "user", "content": "Yes, absolutely!"},
                        {"role": "assistant", "content": "1"},
                        {"role": "user", "content": "No, I don't think so."},
                        {"role": "assistant", "content": "2"},
                        {"role": "user", "content": "Buddy, what's the color of my shirt?"},
                        {"role": "assistant", "content": "4"},
                        {"role": "user", "content": "Buddy, my day has been stressful so far."},
                        {"role": "assistant", "content": "5"},
                        {"role": "user", "content": prompt_text}
                    ],
                    stream=False,
                    options={"timeout": self.timeout}
                )
                
                # Extract response
                response_text = response['message']['content'].strip()
                logger.info(f"LLM response: {response_text}")
                
                # Parse the response to extract just the intent number
                intent_match = re.search(r'\b(\d+)\b', response_text)
                if intent_match:
                    intent_number = int(intent_match.group(1))
                    # Ensure intent is within valid range
                    if 0 <= intent_number <= 5:
                        logger.info(f"Intent classified as {intent_number}")
                        return bytes([intent_number]), True  # Convert to bytes
                    else:
                        logger.warning(f"Invalid intent number {intent_number}, must be between 0-5")
                else:
                    logger.warning(f"Failed to extract intent number from response: {response_text}")
                
                # Increment retry counter
                retries += 1
                
            except Exception as e:
                logger.error(f"Error during intent classification: {str(e)}")
                retries += 1
                time.sleep(0.1)  # Add small delay before retry
        
        # If we got here, all attempts failed
        logger.warning("All classification attempts failed, using fallback classification")
        return bytes([DEFAULT_INTENT]), False  # None intent as bytes
        
    def get_fallback_intent(self):
        """
        Get the fallback intent when classification fails.
        
        Returns:
            bytes: Fallback intent code as bytes
        """
        return bytes([DEFAULT_INTENT])  # None intent
    
    def get_intent_name(self, intent_code):
        """
        Get the intent name from the intent code.
        
        Args:
            intent_code (bytes): Intent code as bytes
            
        Returns:
            str: Intent name or 'None' if not found
        """
        if not intent_code or len(intent_code) == 0:
            return "None"
        
        # Convert bytes to int
        intent_number = intent_code[0]
        
        # Find the intent name by number
        for name, number in INTENT_MAPPING_STRING_TO_BYTE.items():
            if number == intent_number:
                return name
        
        return "None" 