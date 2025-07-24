# Sui Indexer

A ROS2 node for indexing Sui blockchain events.

## Description

The Sui Indexer node monitors and indexes events from a specified Sui package. It tracks events related to locks and shared objects, storing them in a local database for further processing.

## Database Location

For development, the SQLite database is stored in the package source directory as `sui_indexer.db`. 

For production deployments, you should specify an absolute path using the `database_url` parameter to ensure proper data persistence and access permissions.

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
- `database_url` (Optional, default: 'file:sui_indexer.db'): Database URL for the indexer. For production, use an absolute path (e.g., 'file:/var/lib/sui_indexer/sui_indexer.db')

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