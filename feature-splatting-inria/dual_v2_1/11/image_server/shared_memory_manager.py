import threading
from multiprocessing import shared_memory
from typing import Optional, Union, Any, Tuple
import numpy as np

class SharedMemoryManager:
    """Shared memory manager supporting multiple data types"""
    
    def __init__(self, shape: tuple, dtype: Any = np.uint8, name: Optional[str] = None):
        """Initialize shared memory manager
        
        Args:
            shape: shared memory shape (tuple)
            dtype: numpy data type (default: uint8)
            name: shared memory name, if None, create new one
        """
        # Store dtype information
        self.dtype = np.dtype(dtype)
        self.element_size = self.dtype.itemsize
        
        # Calculate size based on dtype
        self.size = int(np.prod(shape)) * self.element_size
        self.shape = shape
        self.lock = threading.RLock()  # reentrant lock
        
        if name:
            try:
                self.shm = shared_memory.SharedMemory(name=name)
                self.shm_name = name
                self.created = False
            except FileNotFoundError:
                self.shm = shared_memory.SharedMemory(create=True, size=self.size)
                self.shm_name = self.shm.name
                self.created = True
        else:
            self.shm = shared_memory.SharedMemory(create=True, size=self.size)
            self.shm_name = self.shm.name
            self.created = True
    
    def write_bytes(self, data: bytes) -> bool:
        """Write raw bytes to shared memory
        
        Args:
            data: bytes to write
            
        Returns:
            bool: True if successful, False if data too large
        """
        try:
            with self.lock:
                if len(data) > self.size:
                    return False
                
                # Directly write bytes to memory
                self.shm.buf[:len(data)] = data
                return True
        except Exception:
            return False
    
    def write_numpy(self, array: np.ndarray) -> bool:
        """Write numpy array to shared memory
        
        Args:
            array: numpy array to write
            
        Returns:
            bool: True if successful, False if data too large
        """
        try:
            return self.write_bytes(array.tobytes())
        except Exception:
            return False
    
    def read_bytes(self) -> Optional[bytes]:
        """Read raw bytes from shared memory
        
        Returns:
            Optional[bytes]: bytes if successful, None otherwise
        """
        try:
            with self.lock:
                # Return all bytes in the buffer
                return bytes(self.shm.buf[:self.size])
        except Exception:
            return None
    
    def read_numpy(self, dtype: Optional[Any] = None, shape: Optional[tuple] = None) -> Optional[np.ndarray]:
        """Read numpy array from shared memory
        
        Args:
            dtype: numpy dtype of the array (default: use instance dtype)
            shape: shape of the array (default: use instance shape)
            
        Returns:
            Optional[np.ndarray]: array if successful, None otherwise
        """
        try:
            data = self.read_bytes()
            if data is None:
                return None
            
            # Use instance dtype and shape if not specified
            read_dtype = dtype if dtype is not None else self.dtype
            read_shape = shape if shape is not None else self.shape
            
            array = np.frombuffer(data, dtype=read_dtype)
            return array.reshape(read_shape)
        except Exception:
            return None
    
    def get_name(self) -> str:
        """Get shared memory name"""
        return self.shm_name
    
    def get_size(self) -> int:
        """Get shared memory size"""
        return self.size
    
    def get_dtype(self) -> np.dtype:
        """Get data type of shared memory"""
        return self.dtype
    
    def get_shape(self) -> tuple:
        """Get shape of shared memory"""
        return self.shape
    
    def cleanup(self):
        """Clean up shared memory"""
        if hasattr(self, 'shm') and self.shm:
            self.shm.close()
            if self.created:
                try:
                    self.shm.unlink()
                except:
                    pass
    
    def __del__(self):
        """Destructor"""
        self.cleanup()