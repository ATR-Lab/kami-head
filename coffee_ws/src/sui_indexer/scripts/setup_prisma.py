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
            # Get package paths
            self.pkg_share = get_package_share_directory('sui_indexer')
            self.prisma_path = os.path.join(self.pkg_share, 'prisma')
            
            # Ensure prisma directory exists
            if not os.path.exists(self.prisma_path):
                raise FileNotFoundError(f"Prisma schema directory not found at {self.prisma_path}")
            
            # Create data directory in package share
            self.data_dir = os.path.join(self.pkg_share, 'data')
            os.makedirs(self.data_dir, exist_ok=True)
            
            # Set absolute database path
            self.db_path = os.path.abspath(os.path.join(self.data_dir, 'sui_indexer.db'))
            
            # Set database URL with absolute path
            os.environ['DATABASE_URL'] = f'file:{self.db_path}'
            
            # Log all paths for verification
            self.get_logger().info(f"Package share directory: {self.pkg_share}")
            self.get_logger().info(f"Prisma directory: {self.prisma_path}")
            self.get_logger().info(f"Data directory: {self.data_dir}")
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
            # Remove existing database if it exists
            if os.path.exists(self.db_path):
                self.get_logger().info(f"Removing existing database at: {self.db_path}")
                os.remove(self.db_path)
            
            # Also check and remove any database in workspace data directory
            workspace_db = os.path.join(os.getcwd(), 'data', 'sui_indexer.db')
            if os.path.exists(workspace_db):
                self.get_logger().info(f"Removing duplicate database at: {workspace_db}")
                os.remove(workspace_db)
            
            # Generate Prisma client
            self.get_logger().info('Generating Prisma client...')
            result = subprocess.run(
                ['prisma', 'generate'], 
                cwd=self.prisma_path, 
                check=True,
                capture_output=True,
                text=True,
                env=dict(os.environ)
            )
            self.get_logger().info('Prisma generate output:')
            self.get_logger().info(result.stdout)
            if result.stderr:
                self.get_logger().warn(f'Prisma generate stderr: {result.stderr}')
            
            # Push the schema to the database
            self.get_logger().info('Pushing schema to database...')
            push_cmd = ['prisma', 'db', 'push', '--force-reset', '--accept-data-loss']
            self.get_logger().info(f"Running command: {' '.join(push_cmd)}")
            self.get_logger().info(f"In directory: {self.prisma_path}")
            self.get_logger().info(f"With DATABASE_URL: {os.environ['DATABASE_URL']}")
            
            result = subprocess.run(
                push_cmd,
                cwd=self.prisma_path, 
                check=True,
                capture_output=True,
                text=True,
                env=dict(os.environ)
            )
            self.get_logger().info('Prisma db push output:')
            self.get_logger().info(result.stdout)
            if result.stderr:
                self.get_logger().warn(f'Prisma db push stderr: {result.stderr}')
            
            # Verify database was created in correct location
            if os.path.exists(self.db_path):
                size = os.path.getsize(self.db_path)
                self.get_logger().info(f"Database file created successfully at: {self.db_path}")
                self.get_logger().info(f"Database file size: {size} bytes")
                
                # Double check no duplicate was created
                if os.path.exists(workspace_db):
                    self.get_logger().error(f"Unexpected duplicate database created at: {workspace_db}")
                    raise RuntimeError("Prisma created database in unexpected location")
                
                # Verify tables by connecting and querying
                try:
                    from prisma import Prisma
                    
                    # Initialize Prisma client with absolute path
                    db = Prisma(
                        datasource={
                            "url": f"file:{self.db_path}"
                        }
                    )
                    
                    # Connect and verify tables exist
                    async def verify_tables():
                        await db.connect()
                        
                        # Try to query each table
                        cursor_count = await db.cursor.count()
                        escrow_count = await db.escrow.count()
                        locked_count = await db.locked.count()
                        
                        self.get_logger().info(f"Verified tables exist - Counts: Cursor={cursor_count}, Escrow={escrow_count}, Locked={locked_count}")
                        await db.disconnect()
                    
                    # Run verification
                    import asyncio
                    asyncio.run(verify_tables())
                    
                except Exception as e:
                    self.get_logger().error(f"Failed to verify tables: {str(e)}")
                    raise
            else:
                self.get_logger().error(f"Database file not created at expected location: {self.db_path}")
                if os.path.exists(workspace_db):
                    self.get_logger().error(f"Database was created in wrong location: {workspace_db}")
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