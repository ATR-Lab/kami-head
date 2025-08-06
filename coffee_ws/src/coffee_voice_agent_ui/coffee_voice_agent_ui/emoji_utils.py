#!/usr/bin/env python3
"""
Emoji Utilities - Platform-specific emoji handling

Provides cross-platform emoji support with automatic fallbacks for macOS
to prevent Qt rendering crashes in external terminal contexts.
"""

import sys


class EmojiManager:
    """Centralized emoji management with platform-specific fallbacks"""
    
    def __init__(self):
        self.is_macos = sys.platform == "darwin"
        
        # Emoji mappings: (ubuntu_emoji, macos_fallback)
        self.emoji_map = {
            # Main UI section emojis
            'agent_status': ('🤖', '[AI]'),
            'admin_override': ('⚙️', '[ADMIN]'),
            'analytics': ('📊', '[DATA]'),
            'conversation': ('💬', '[CHAT]'),
            'emotion_center': ('🎭', '[EMO]'),
            'tool_activity': ('🔧', '[TOOL]'),
            'virtual_requests': ('☕', '[REQ]'),
            
            # Status indicators
            'connected': ('✅', '[✓]'),
            'disconnected': ('❌', '[X]'),
            'online': ('✅', '[ON]'),
            'offline': ('❌', '[OFF]'),
            
            # Activity/Action emojis
            'session_performance': ('📈', '[PERF]'),
            'popular_interactions': ('🎯', '[POP]'),
            'emotion_trends': ('🎭', '[TREND]'),
            'system_metrics': ('⚙️', '[SYS]'),
            'active_tools': ('⚡', '[ACTIVE]'),
            'total_today': ('📈', '[TOTAL]'),
            'recent_activity': ('📋', '[RECENT]'),
            'usage_statistics': ('📊', '[STATS]'),
            'conversation_time': ('⏰', '[TIME]'),
            'user_timeout': ('⏱️', '[TIMEOUT]'),
            'lock_scroll': ('🔒', '[LOCK]'),
            'auto_scroll': ('📜', '[SCROLL]'),
            'clear': ('🗑️', '[CLEAR]'),
            'vip_status': ('📊', '[VIP]'),
            'vip_history': ('📋', '[HIST]'),
            'test_request': ('☕', '[TEST]'),
            
            # User/Agent/Tool indicators
            'user': ('👤', '[USER]'),
            'agent': ('🤖', '[AI]'),
            'tool': ('🔧', '[TOOL]'),
            
            # Emotion emojis (extensive mapping for EmotionDisplayWidget)
            'friendly': ('😊', 'friendly'),
            'excited': ('🤩', 'excited!'),
            'curious': ('🤔', 'curious?'),
            'sleepy': ('😴', 'sleepy'),
            'waiting': ('😌', 'waiting'),
            'excuse': ('😅', 'excuse me'),
            'helpful': ('🤝', 'helpful'),
            'empathetic': ('🥺', 'caring'),
            'confused': ('😕', 'confused'),
            'proud': ('😊', 'proud'),
            'playful': ('😄', 'playful'),
            'focused': ('🧐', 'focused'),
            'surprised': ('😮', 'surprised!'),
            'enthusiastic': ('🎉', 'enthusiastic!'),
            'warm': ('🤗', 'warm'),
            'professional': ('👔', 'professional'),
            'cheerful': ('😁', 'cheerful'),
        }
    
    def get(self, key, default=''):
        """Get emoji for the given key, with platform-specific fallback"""
        if key not in self.emoji_map:
            return default
        
        emoji, fallback = self.emoji_map[key]
        return fallback if self.is_macos else emoji
    
    def format_title(self, emoji_key, title):
        """Format a title with appropriate emoji/fallback"""
        emoji = self.get(emoji_key)
        if emoji:
            return f"{emoji} {title}"
        return title
    
    def format_status(self, emoji_key, status_text):
        """Format a status message with appropriate emoji/fallback"""
        emoji = self.get(emoji_key)
        if emoji:
            return f"{emoji} {status_text}"
        return status_text
    
    def get_connection_status(self, connected):
        """Get connection status with appropriate emoji/fallback"""
        if connected:
            return self.format_status('connected', 'Connected')
        else:
            return self.format_status('disconnected', 'Disconnected')
    
    def debug_info(self):
        """Get debug information about emoji handling"""
        return {
            'platform': sys.platform,
            'is_macos': self.is_macos,
            'emoji_mode': 'fallback' if self.is_macos else 'emoji',
            'total_mappings': len(self.emoji_map)
        }


# Global emoji manager instance
emoji_manager = EmojiManager()


# Convenience functions for easy access
def get_emoji(key, default=''):
    """Get platform-appropriate emoji or fallback"""
    return emoji_manager.get(key, default)


def format_title(emoji_key, title):
    """Format title with platform-appropriate emoji"""
    return emoji_manager.format_title(emoji_key, title)


def format_status(emoji_key, status_text):
    """Format status with platform-appropriate emoji"""
    return emoji_manager.format_status(emoji_key, status_text)


def get_connection_status(connected):
    """Get formatted connection status"""
    return emoji_manager.get_connection_status(connected) 