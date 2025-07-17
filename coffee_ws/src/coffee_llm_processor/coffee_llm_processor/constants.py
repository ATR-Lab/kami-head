#!/usr/bin/env python3
"""
Constants for Coffee LLM Processor package.

This module contains topic names and service names used by the LLM processor
for the Coffee Buddy robot system. These constants were previously defined 
in the shared_configs package but are now local to maintain package independence.
"""

# Service endpoints
LLM_CHAT_SERVICE = "/coffee/llm/chat"  # Main conversational AI service

# Topic endpoints  
LLM_STATUS_TOPIC = "/coffee/llm/status"  # Health and status monitoring 