#!/usr/bin/env python3

"""
Memory management utilities for monitoring and cleaning up GPU memory.

This module provides tools for tracking GPU memory usage and performing
periodic cleanup to prevent memory leaks during prolonged operation.
"""

import gc
import logging
import time

logger = logging.getLogger(__name__)


class MemoryManager:
    """
    Manages memory usage and cleanup for ASR and LLM components.
    
    This class is responsible for tracking GPU memory usage, performing
    periodic cleanup, and helping prevent memory leaks during operation.
    """
    
    def __init__(self, using_gpu=False, monitoring_enabled=True, cleanup_interval=10):
        """
        Initialize the memory manager.
        
        Args:
            using_gpu (bool): Whether GPU is being used
            monitoring_enabled (bool): Whether to monitor GPU memory usage
            cleanup_interval (int): Number of inference cycles between cleanups
        """
        self.using_gpu = using_gpu
        self.monitoring_enabled = monitoring_enabled
        self.cleanup_interval = cleanup_interval
        self.inference_count = 0
        self.last_cleanup_time = time.time()
        
        # Track peak memory usage
        self.peak_gpu_memory = 0
        self.peak_ram_memory = 0
        
        # Conditionally import torch
        if self.using_gpu:
            try:
                import torch
                self.torch_available = True
                torch.cuda.reset_peak_memory_stats()
                logger.info("PyTorch available for GPU memory management")
            except ImportError:
                self.torch_available = False
                logger.warning("PyTorch not available, GPU memory monitoring disabled")
    
    def log_gpu_memory_usage(self, label=""):
        """
        Log current GPU memory usage.
        
        Args:
            label (str): Optional label for the log message
        """
        if not self.using_gpu or not self.monitoring_enabled or not self.torch_available:
            return
        
        try:
            import torch
            
            # Get current GPU memory usage
            allocated = torch.cuda.memory_allocated(0) / 1024 / 1024
            reserved = torch.cuda.memory_reserved(0) / 1024 / 1024
            peak = torch.cuda.max_memory_allocated(0) / 1024 / 1024
            
            if label:
                logger.info(f"GPU Memory {label}:\n" + 
                           f"  Current: {allocated:.2f} MB allocated\n" +
                           f"  Reserved: {reserved:.2f} MB\n" +
                           f"  Peak: {peak:.2f} MB")
            else:
                logger.info(f"GPU Memory:\n" +
                           f"  Current: {allocated:.2f} MB allocated\n" +
                           f"  Reserved: {reserved:.2f} MB\n" +
                           f"  Peak: {peak:.2f} MB")
                
            # Log warning if memory usage is high
            if allocated > 1000:  # 1 GB threshold
                logger.warning(f"High GPU memory usage detected: {allocated:.2f} MB")
                
        except Exception as e:
            logger.error(f"Error monitoring GPU memory: {str(e)}")
    
    def cleanup_memory(self, force=False):
        """
        Perform memory cleanup operations.
        
        Args:
            force (bool): Force cleanup regardless of interval
        """
        self.inference_count += 1
        current_time = time.time()
        
        # Check if cleanup is due based on inference count or time elapsed
        if (not force and 
            self.inference_count % self.cleanup_interval != 0 and
            current_time - self.last_cleanup_time < 60):  # At least every 60 seconds
            return
        
        logger.info(f"Performing memory cleanup (forced={force})")
        
        # Run Python garbage collector
        gc.collect()
        
        # GPU-specific cleanup
        if self.using_gpu and self.torch_available:
            try:
                import torch
                
                # Log before cleanup
                before_allocated = torch.cuda.memory_allocated(0) / 1024 / 1024
                
                # Empty CUDA cache
                torch.cuda.empty_cache()
                
                # Run garbage collection again
                gc.collect()
                
                # Log after cleanup
                after_allocated = torch.cuda.memory_allocated(0) / 1024 / 1024
                
                logger.info(f"GPU memory cleanup: {before_allocated:.2f} MB → {after_allocated:.2f} MB " +
                           f"(freed {max(0, before_allocated - after_allocated):.2f} MB)")
                
            except Exception as e:
                logger.error(f"Error during GPU memory cleanup: {str(e)}")
        
        # Reset counter and timestamp
        self.inference_count = 0
        self.last_cleanup_time = current_time
        
    def get_detailed_memory_usage(self):
        """
        Get detailed memory usage information.
        
        Returns:
            dict: Detailed memory usage metrics
        """
        import psutil
        
        ram_used = psutil.virtual_memory().used / (1024 * 1024)  # MB
        self.peak_ram_memory = max(self.peak_ram_memory, ram_used)
        
        info = {
            'ram_total': psutil.virtual_memory().total / (1024 * 1024),  # MB
            'ram_used': ram_used,
            'ram_peak': self.peak_ram_memory,
            'ram_percent': psutil.virtual_memory().percent,
            'gpu_used': 0,
            'gpu_peak': 0,
            'gpu_total': 0,
            'gpu_percent': 0
        }
        
        if self.using_gpu and self.torch_available:
            try:
                import torch
                # Get current GPU memory usage
                gpu_allocated = torch.cuda.memory_allocated(0) / (1024 * 1024)    # MB
                gpu_reserved = torch.cuda.memory_reserved(0) / (1024 * 1024)      # MB
                gpu_peak = torch.cuda.max_memory_allocated(0) / (1024 * 1024)     # MB
                
                # Update peak if current usage is higher
                self.peak_gpu_memory = max(self.peak_gpu_memory, gpu_peak)
                
                info.update({
                    'gpu_used': gpu_allocated,
                    'gpu_reserved': gpu_reserved,
                    'gpu_peak': self.peak_gpu_memory,
                    'gpu_total': torch.cuda.get_device_properties(0).total_memory / (1024 * 1024)
                })
                info['gpu_percent'] = (info['gpu_used'] / info['gpu_total']) * 100
            except Exception as e:
                logger.error(f"Error getting GPU metrics: {str(e)}")
        
        return info
    
    def get_memory_info(self):
        """
        Get current memory info for diagnostics.
        
        Returns:
            dict: Memory information
        """
        info = {
            "using_gpu": self.using_gpu,
            "torch_available": self.torch_available,
            "monitoring_enabled": self.monitoring_enabled,
            "cleanup_interval": self.cleanup_interval,
            "inference_count": self.inference_count,
            "last_cleanup": f"{int(time.time() - self.last_cleanup_time)} seconds ago"
        }
        
        # Add GPU info if available
        if self.using_gpu and self.torch_available:
            try:
                import torch
                info["gpu_allocated_mb"] = f"{torch.cuda.memory_allocated(0) / 1024 / 1024:.2f}"
                info["gpu_reserved_mb"] = f"{torch.cuda.memory_reserved(0) / 1024 / 1024:.2f}"
                info["gpu_device"] = torch.cuda.get_device_name(0)
            except:
                pass
                
        return info 