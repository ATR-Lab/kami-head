# Sui Indexer

A ROS2 node for indexing Sui blockchain events.

## Description

The Sui Indexer node monitors and indexes events from a specified Sui package. It tracks events related to locks and shared objects, storing them in a local database for further processing.

## Database Location

**Development:** The SQLite database is stored in the workspace data directory at `<workspace_root>/data/sui_indexer/sui_indexer.db`. This ensures the database persists across `colcon build` cycles and is not affected by install directory changes.

**Production:** You can specify an absolute path using the `database_url` parameter to ensure proper data persistence and access permissions for production deployments.

**Important:** The database location has been moved out of the install directory to prevent data loss during builds. The Prisma client is also generated locally within the workspace to avoid virtual environment pollution.

## Usage

### Running the Indexer

To run the indexer, use the following command:

```bash
ros2 launch sui_indexer indexer.launch.py "package_id:='YOUR_PACKAGE_ID'"
```

For example:
```bash
ros2 launch sui_indexer indexer.launch.py "package_id:='0x052f4da5dddf486da555e6c6aea3818e8d8206931f74f7441be5417cf9eeb070'"
```

Note: The package_id must be enclosed in quotes to ensure it's treated as a string.

### Configuration Parameters

The following parameters can be configured when launching the indexer:

- `package_id` (Required): The Sui package ID to index
- `network` (Optional, default: 'testnet'): Sui network to connect to (testnet, mainnet, devnet)
- `polling_interval_ms` (Optional, default: 1000): Polling interval in milliseconds
- `default_limit` (Optional, default: 50): Default limit for event queries
- `database_url` (Optional, default: workspace data directory): Database URL for the indexer. When not specified, defaults to `<workspace_root>/data/sui_indexer/sui_indexer.db`. For production, use an absolute path (e.g., 'file:/var/lib/sui_indexer/sui_indexer.db')

Example with multiple parameters:
```bash
ros2 launch sui_indexer indexer.launch.py \
    "package_id:='0xfe09cf0b3d77678b99250572624bf74fe3b12af915c5db95f0ed5d755612eb68'" \
    "network:='mainnet'" \
    "polling_interval_ms:=2000" \
    "database_url:='file:/var/lib/sui_indexer/sui_indexer.db'"  # Example production path
```

## Topics

The indexer publishes to the following topics:
- `/sui_events`: Published events from the Sui blockchain
- `/indexer_status`: Status updates from the indexer

## Architecture Notes

**Workspace Integration:** This package follows ROS2 best practices by:
- Storing persistent data outside the install directory
- Generating Prisma client locally within workspace data directory
- Using environment isolation to prevent global variable pollution
- Supporting both development and production deployment patterns 