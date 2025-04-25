#!/usr/bin/env python3

"""
Memory management utilities for GPU and resource cleanup.

This module contains functions for monitoring and managing GPU memory usage,
performing garbage collection, and other memory-related utilities.
"""

import gc
import logging

# Conditionally import torch for GPU memory management
try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

logger = logging.getLogger(__name__)


class MemoryManager:
    """
    Manages memory resources and provides utilities for monitoring and cleanup.
    
    This class handles GPU memory monitoring and cleanup operations to ensure
    efficient resource usage, especially for GPU-accelerated applications.
    """

    def __init__(self, using_gpu=False, monitoring_enabled=True, cleanup_interval=10):
        """
        Initialize the memory manager.
        
        Args:
            using_gpu (bool): Whether GPU is being used
            monitoring_enabled (bool): Whether to enable memory monitoring
            cleanup_interval (int): Number of iterations between cleanup operations
        """
        self.using_gpu = using_gpu and TORCH_AVAILABLE
        self.monitoring_enabled = monitoring_enabled
        self.cleanup_interval = cleanup_interval
        self.inference_count = 0

    def log_gpu_memory_usage(self, label=""):
        """
        Log current GPU memory usage if monitoring is enabled and GPU is being used.
        
        Args:
            label (str): Label for the memory usage log entry
        """
        if not (self.monitoring_enabled and self.using_gpu):
            return
        
        try:
            # Get current GPU memory allocation
            allocated = torch.cuda.memory_allocated() / (1024 * 1024)  # Convert to MB
            reserved = torch.cuda.memory_reserved() / (1024 * 1024)    # Convert to MB
            logger.info(f"GPU Memory [{label}] - Allocated: {allocated:.2f} MB, Reserved: {reserved:.2f} MB")
        except Exception as e:
            logger.warning(f"Error monitoring GPU memory: {str(e)}")
    
    def cleanup_memory(self, force=False):
        """
        Perform memory cleanup operations.
        
        Args:
            force (bool): Whether to force cleanup regardless of interval
            
        Returns:
            bool: True if cleanup was performed, False otherwise
        """
        if self.using_gpu:
            self.inference_count += 1
            
            # Only clean up periodically to avoid overhead
            if force or (self.inference_count % self.cleanup_interval == 0):
                logger.debug("Performing memory cleanup")
                
                # Python garbage collection
                gc.collect()
                
                # CUDA memory cache cleanup
                if TORCH_AVAILABLE and torch.cuda.is_available():
                    torch.cuda.empty_cache()
                
                # Log memory usage after cleanup
                if force:
                    self.log_gpu_memory_usage("After forced cleanup")
                else:
                    self.log_gpu_memory_usage("After periodic cleanup")
                
                return True
        
        return False 