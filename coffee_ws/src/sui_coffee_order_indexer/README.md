# Sui Coffee Order Indexer

A ROS2 node for indexing coffee club events from the Sui blockchain and publishing them to ROS2 topics.

## Description

The Sui Coffee Order Indexer monitors and indexes events from the coffee club smart contract on Sui. It tracks cafe creation and coffee order events, storing them in a local database and publishing them to ROS2 topics for other nodes to consume.

**Architecture:** This indexer follows a decoupled design where it focuses solely on blockchain event processing. Coffee machine integration is handled by separate controller nodes that subscribe to the indexer's published events, following proper ROS2 architectural patterns.

## Database Location

**Development:** The SQLite database is stored in the workspace data directory at `<workspace_root>/data/sui_indexer/sui_indexer.db`. This ensures the database persists across `colcon build` cycles and is not affected by install directory changes.

**Production:** You can specify an absolute path using the `database_url` parameter to ensure proper data persistence and access permissions for production deployments.

**Important:** The database location has been moved out of the install directory to prevent data loss during builds. The Prisma client is also generated locally within the workspace to avoid virtual environment pollution.

## Usage

### Running the Indexer

To run the indexer, use the following command:

```bash
ros2 launch sui_coffee_order_indexer indexer.launch.py "package_id:='YOUR_PACKAGE_ID'"
```

For example:
```bash
ros2 launch sui_coffee_order_indexer indexer.launch.py "package_id:='0x2ee032ffc863a74a785ac3003fb8b61d639d9095b4431fdc1b12181c0a2a8c13'"
```

With specific network:
```bash
ros2 launch sui_coffee_order_indexer indexer.launch.py \
    "package_id:='0x2ee032ffc863a74a785ac3003fb8b61d639d9095b4431fdc1b12181c0a2a8c13'" \
    "network:='mainnet'"
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
ros2 launch sui_coffee_order_indexer indexer.launch.py \
    "package_id:='0x2ee032ffc863a74a785ac3003fb8b61d639d9095b4431fdc1b12181c0a2a8c13'" \
    "network:='mainnet'" \
    "polling_interval_ms:=2000" \
    "database_url:='file:/var/lib/sui_indexer/sui_indexer.db'"  # Example production path
```

## Topics

The indexer publishes to the following topics:
- `/sui_events`: Published coffee club events from the Sui blockchain
- `/indexer_status`: Status updates from the indexer

## Integration with Coffee Machines

**Decoupled Architecture:** This indexer does not directly control coffee machines. Instead, it publishes events to ROS2 topics that can be consumed by separate coffee controller nodes.

**Recommended Integration Pattern:**
```
Blockchain → Indexer → ROS2 Topics → Coffee Controller Node → Coffee Machine
```

Coffee controller nodes should:
- Subscribe to `/sui_events` topic
- Filter for `CoffeeOrderUpdated` events with status "Processing"  
- Queue and manage coffee machine operations
- Handle machine status, retries, and error recovery

This design provides better fault isolation, scalability, and follows ROS2 best practices.

## Architecture Notes

**Workspace Integration:** This package follows ROS2 best practices by:
- Storing persistent data outside the install directory
- Generating Prisma client locally within workspace data directory
- Using environment isolation to prevent global variable pollution
- Supporting both development and production deployment patterns

**Decoupled Design:** The indexer focuses solely on blockchain event processing, while coffee machine integration is handled by separate controller nodes. This provides:
- Better fault isolation (coffee machine issues don't crash the indexer)
- Proper separation of concerns (blockchain vs hardware control)
- Scalability (multiple coffee machines, queue management)
- Easier testing and maintenance 