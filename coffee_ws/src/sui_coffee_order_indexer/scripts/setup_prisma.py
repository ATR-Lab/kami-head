#!/usr/bin/env python3
import os
import subprocess
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

class PrismaSetupNode(Node):
    def __init__(self):
        super().__init__('prisma_setup')
        
        try:
            # Get package share for prisma schema location only
            pkg_share = get_package_share_directory('sui_coffee_order_indexer')
            self.prisma_path = os.path.join(pkg_share, 'prisma')
            
            # Ensure prisma directory exists
            if not os.path.exists(self.prisma_path):
                raise FileNotFoundError(f"Prisma schema directory not found at {self.prisma_path}")
            
            # Use workspace root for persistent data (NOT install directory)
            workspace_root = Path.cwd()  # Should be coffee_ws when launched properly
            self.data_dir = workspace_root / "data" / "sui_indexer"
            self.data_dir.mkdir(parents=True, exist_ok=True)
            
            # Set absolute database path in workspace data directory
            self.db_path = self.data_dir / "sui_indexer.db"
            
            # Set database URL 
            os.environ['DATABASE_URL'] = f'file:{self.db_path}'
            
            # Log all paths for verification
            self.get_logger().info(f"Package share directory: {pkg_share}")
            self.get_logger().info(f"Prisma directory: {self.prisma_path}")
            self.get_logger().info(f"Workspace data directory: {self.data_dir}")
            self.get_logger().info(f"Database absolute path: {self.db_path}")
            self.get_logger().info(f"Database URL: {os.environ['DATABASE_URL']}")
            
            # Run setup
            self.setup_prisma()
            
        except Exception as e:
            self.get_logger().error(f'Error during initialization: {str(e)}')
            sys.exit(1)
        
    def setup_prisma(self):
        """Run Prisma generate and migrate."""
        try:
            # Remove existing database if it exists for fresh start
            if self.db_path.exists():
                self.get_logger().info(f"Removing existing database at: {self.db_path}")
                self.db_path.unlink()
            
            # Generate Prisma client (will use default location)
            self.get_logger().info('Generating Prisma client...')
            result = subprocess.run(
                ['prisma', 'generate'], 
                cwd=self.prisma_path, 
                check=True,
                capture_output=True,
                text=True,
                env=dict(os.environ)
            )
            self.get_logger().info('Prisma generate completed successfully')
            if result.stdout:
                self.get_logger().info(f'Generate output: {result.stdout}')
            if result.stderr:
                self.get_logger().warn(f'Generate stderr: {result.stderr}')
            
            # Push the schema to the database
            self.get_logger().info('Pushing schema to database...')
            push_cmd = ['prisma', 'db', 'push', '--force-reset', '--accept-data-loss']
            
            result = subprocess.run(
                push_cmd,
                cwd=self.prisma_path, 
                check=True,
                capture_output=True,
                text=True,
                env=dict(os.environ)
            )
            self.get_logger().info('Prisma db push completed successfully')
            if result.stdout:
                self.get_logger().info(f'Push output: {result.stdout}')
            if result.stderr:
                self.get_logger().warn(f'Push stderr: {result.stderr}')
            
            # Verify database was created in correct location
            if self.db_path.exists():
                size = self.db_path.stat().st_size
                self.get_logger().info(f"Database file created successfully at: {self.db_path}")
                self.get_logger().info(f"Database file size: {size} bytes")
                
                # Verify tables by connecting and querying
                try:
                    from prisma import Prisma
                    
                    # Initialize Prisma client 
                    db = Prisma()
                    
                    # Connect and verify tables exist
                    async def verify_tables():
                        await db.connect()
                        
                        # Try to query each table
                        cursor_count = await db.cursor.count()
                        cafe_count = await db.cafe.count()
                        coffeeorder_count = await db.coffeeorder.count()
                        
                        self.get_logger().info(f"Verified tables exist - Counts: Cursor={cursor_count}, Cafe={cafe_count}, CoffeeOrder={coffeeorder_count}")
                        await db.disconnect()
                    
                    # Run verification
                    import asyncio
                    asyncio.run(verify_tables())
                    
                except Exception as e:
                    self.get_logger().error(f"Failed to verify tables: {str(e)}")
                    raise
            else:
                self.get_logger().error(f"Database file not created at expected location: {self.db_path}")
                raise FileNotFoundError(f"Database file not created at {self.db_path}")
            
            self.get_logger().info('Prisma setup completed successfully')
            
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Prisma command failed with return code {e.returncode}')
            self.get_logger().error(f'Command output: {e.stdout}')
            self.get_logger().error(f'Command error: {e.stderr}')
            sys.exit(1)
        except Exception as e:
            self.get_logger().error(f'Unexpected error during Prisma setup: {str(e)}')
            sys.exit(1)

def main(args=None):
    rclpy.init(args=args)
    node = PrismaSetupNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 