# Zero-Trust Consensus Protocol Code Examples

Complete code examples for using the Zero-Trust Consensus Protocol in C, C++, and Python 3.

## C++ Examples

### Example 1: Basic Network Client Initialization

```cpp
#include "robocon_network_client/network_client.hpp"
#include <iostream>

int main() {
    // Initialize Network Client
    robocon::NetworkClient client("robot_001");
    
    if (!client.initialize()) {
        std::cerr << "Failed to initialize network client" << std::endl;
        return 1;
    }
    
    std::cout << "Network client initialized successfully" << std::endl;
    
    // Discover peers
    auto peers = client.discoverPeers(5000);
    std::cout << "Discovered " << peers.size() << " peers" << std::endl;
    
    for (const auto& peer : peers) {
        std::cout << "Peer: " << peer.peer_id 
                  << " Type: " << peer.peer_type << std::endl;
    }
    
    // Cleanup
    client.shutdown();
    return 0;
}
```

### Example 2: Action Proposal and Voting

```cpp
#include "robocon_network_client/network_client.hpp"
#include "robocon_network_client/consensus_manager.hpp"
#include <iostream>

int main() {
    robocon::NetworkClient client("robot_001");
    client.initialize();
    
    // Get consensus manager
    auto& consensus = client.getConsensusManager();
    
    // Set up consensus callback
    consensus.setConsensusCallback([](const robocon::ConsensusResult& result) {
        if (result.consensus_reached) {
            std::cout << "Consensus reached for action: " << result.action_id << std::endl;
            std::cout << "Yes votes: " << result.yes_votes << std::endl;
            std::cout << "Executor: " << result.executor_id << std::endl;
        } else {
            std::cout << "Consensus failed for action: " << result.action_id << std::endl;
        }
    });
    
    // Propose an action
    robocon::ActionSuggestion action(
        "action_001",           // action_id
        "robot_001",           // proposer_id
        "move_sheathing"       // action_type
    );
    
    action.target_location = "building_B_floor_2_room_5";
    action.estimated_cost = 150.0;
    action.required_capabilities = {"manipulation", "navigation"};
    action.behavior_tree_node = "MoveSheathing";
    
    // Propose action and wait for consensus
    auto result = consensus.proposeAction(action);
    
    if (result.consensus_reached) {
        std::cout << "Action approved! Executor: " << result.executor_id << std::endl;
        
        if (result.executor_id == "robot_001") {
            std::cout << "This robot will execute the action" << std::endl;
            // Execute action...
        } else {
            std::cout << "Monitoring executor: " << result.executor_id << std::endl;
            // Monitor executor...
        }
    }
    
    client.shutdown();
    return 0;
}
```

### Example 3: Motion Ledger Operations

```cpp
#include "robocon_network_client/network_client.hpp"
#include <iostream>

int main() {
    robocon::NetworkClient client("robot_001");
    client.initialize();
    
    auto& ledger = client.getLedgerManager();
    
    // Create a motion entry
    robocon::MotionEntry motion("motion_001", "robot_001");
    motion.pose = robocon::Pose(10.5, 20.3, 0.0, 0.0, 0.0, 0.0, 1.0);
    motion.precision = 0.01;
    motion.resolution = 0.1;
    motion.shape_hash = "shape_hash_001";
    
    // Append to ledger
    ledger.appendMotionLedger(motion);
    std::cout << "Motion entry appended" << std::endl;
    
    // Verify ledger integrity
    if (ledger.verifyMotionLedger("robot_001")) {
        std::cout << "Motion ledger verified" << std::endl;
    }
    
    // Retrieve motion ledger
    auto motions = ledger.getMotionLedger("robot_001");
    std::cout << "Motion ledger contains " << motions.size() << " entries" << std::endl;
    
    for (const auto& m : motions) {
        std::cout << "Motion: " << m.entry_id 
                  << " at (" << m.pose.x << ", " << m.pose.y << ", " << m.pose.z << ")" 
                  << std::endl;
    }
    
    // Create goal entry
    robocon::GoalEntry goal("goal_001", "robot_001", 
                           "Move sheathing to building B, floor 2, room 5");
    
    // Link goal to motion entries
    ledger.linkGoalToMotions("goal_001", {"motion_001", "motion_002"});
    
    // Append goal ledger
    ledger.appendGoalLedger(goal);
    std::cout << "Goal entry appended" << std::endl;
    
    client.shutdown();
    return 0;
}
```

### Example 4: Sensor Data Request

```cpp
#include "robocon_network_client/network_client.hpp"
#include <iostream>

int main() {
    robocon::NetworkClient client("robot_001");
    client.initialize();
    
    // Set up sensor data callback (for robot mode)
    client.setSensorDataRequestCallback([](const robocon::SensorDataRequest& request) 
                                         -> robocon::SensorData {
        std::cout << "Handling request for " << request.sensor_type << " data" << std::endl;
        
        robocon::SensorData data("response_" + request.request_id, 
                                 "robot_001", 
                                 request.sensor_type);
        
        // Generate sensor data based on type
        if (request.sensor_type == "lidar") {
            // Generate LiDAR data...
            data.data = generateLidarData();
            data.resolution = 0.1;
            data.precision = 0.01;
        }
        
        return data;
    });
    
    // Request sensor data from another robot
    robocon::SensorDataRequest request("req_001", "robot_002", "lidar");
    auto response = client.sendSensorDataRequest(request, 5000);
    
    if (response.success) {
        std::cout << "Received " << response.data.data.size() << " bytes" << std::endl;
    }
    
    client.shutdown();
    return 0;
}
```

### Example 5: Zero-Trust Identity Verification

```cpp
#include "robocon_network_client/network_client.hpp"
#include <iostream>

int main() {
    robocon::NetworkClient client("robot_001");
    client.initialize();
    
    // Discover peers
    auto peers = client.discoverPeers(5000);
    
    for (const auto& peer : peers) {
        std::cout << "Verifying peer: " << peer.peer_id << std::endl;
        
        // Verify peer identity
        if (client.verifyPeerIdentity(peer.peer_id)) {
            std::cout << "Peer identity verified" << std::endl;
            
            // Get peer's motion ledger hash
            auto& ledger = client.getLedgerManager();
            auto latest_hash = ledger.getLatestMotionHash(peer.peer_id);
            
            if (latest_hash.has_value()) {
                std::cout << "Latest motion hash: " << latest_hash.value() << std::endl;
                
                // Verify motion hash
                if (client.verifyMotionHash(peer.peer_id, latest_hash.value())) {
                    std::cout << "Motion hash verified" << std::endl;
                }
            }
        } else {
            std::cout << "Peer identity verification failed" << std::endl;
        }
    }
    
    client.shutdown();
    return 0;
}
```

## C Examples

Note: C API uses C-compatible wrappers or FFI bindings. This example shows conceptual usage:

### Example 1: Basic Initialization (Conceptual)

```c
#include <stdio.h>
#include <stdlib.h>
#include "robocon_network_client_c.h"  // Hypothetical C wrapper

int main() {
    // Initialize network client
    robocon_client_t* client = robocon_client_create("robot_001");
    
    if (!client) {
        fprintf(stderr, "Failed to create client\n");
        return 1;
    }
    
    if (robocon_client_initialize(client) != 0) {
        fprintf(stderr, "Failed to initialize client\n");
        robocon_client_destroy(client);
        return 1;
    }
    
    printf("Network client initialized\n");
    
    // Discover peers
    peer_info_list_t* peers = robocon_client_discover_peers(client, 5000);
    
    if (peers) {
        printf("Discovered %d peers\n", peers->count);
        
        for (size_t i = 0; i < peers->count; i++) {
            printf("Peer %zu: %s\n", i, peers->peers[i].peer_id);
        }
        
        robocon_peer_list_free(peers);
    }
    
    // Cleanup
    robocon_client_shutdown(client);
    robocon_client_destroy(client);
    
    return 0;
}
```

### Example 2: Action Proposal (Conceptual)

```c
#include <stdio.h>
#include "robocon_network_client_c.h"

int main() {
    robocon_client_t* client = robocon_client_create("robot_001");
    robocon_client_initialize(client);
    
    // Create action suggestion
    action_suggestion_t action;
    action.action_id = "action_001";
    action.proposer_id = "robot_001";
    action.action_type = "move_sheathing";
    action.target_location = "building_B_floor_2_room_5";
    action.estimated_cost = 150.0;
    
    // Propose action
    consensus_result_t result;
    if (robocon_client_propose_action(client, &action, &result) == 0) {
        if (result.consensus_reached) {
            printf("Consensus reached! Executor: %s\n", result.executor_id);
        } else {
            printf("Consensus failed\n");
        }
    }
    
    robocon_client_shutdown(client);
    robocon_client_destroy(client);
    
    return 0;
}
```

## Python 3 Examples

### Example 1: Basic Network Client Initialization

```python
#!/usr/bin/env python3
import robocon_network_client
import time

def main():
    # Initialize Network Client
    client = robocon_network_client.NetworkClient("robot_001")
    
    if not client.initialize():
        print("Failed to initialize network client")
        return 1
    
    print("Network client initialized successfully")
    
    # Discover peers
    peers = client.discover_peers(timeout_ms=5000)
    print(f"Discovered {len(peers)} peers")
    
    for peer in peers:
        print(f"Peer: {peer.peer_id} Type: {peer.peer_type}")
        print(f"Capabilities: {', '.join(peer.capabilities)}")
    
    # Cleanup
    client.shutdown()
    return 0

if __name__ == "__main__":
    main()
```

### Example 2: Action Proposal and Voting

```python
#!/usr/bin/env python3
import robocon_network_client
import time

def main():
    client = robocon_network_client.NetworkClient("robot_001")
    client.initialize()
    
    # Get consensus manager
    consensus = client.get_consensus_manager()
    
    # Set up consensus callback
    def consensus_callback(result):
        if result.consensus_reached:
            print(f"Consensus reached for action: {result.action_id}")
            print(f"Yes votes: {result.yes_votes}")
            print(f"Executor: {result.executor_id}")
        else:
            print(f"Consensus failed for action: {result.action_id}")
    
    consensus.set_consensus_callback(consensus_callback)
    
    # Propose an action
    action = robocon_network_client.ActionSuggestion(
        action_id="action_001",
        proposer_id="robot_001",
        action_type="move_sheathing"
    )
    
    action.target_location = "building_B_floor_2_room_5"
    action.estimated_cost = 150.0
    action.required_capabilities = ["manipulation", "navigation"]
    action.behavior_tree_node = "MoveSheathing"
    
    # Propose action and wait for consensus
    result = consensus.propose_action(action)
    
    if result.consensus_reached:
        print(f"Action approved! Executor: {result.executor_id}")
        
        if result.executor_id == "robot_001":
            print("This robot will execute the action")
            # Execute action...
        else:
            print(f"Monitoring executor: {result.executor_id}")
            # Monitor executor...
    
    client.shutdown()
    return 0

if __name__ == "__main__":
    main()
```

### Example 3: Motion Ledger Operations

```python
#!/usr/bin/env python3
import robocon_network_client
from robocon_network_client import Pose

def main():
    client = robocon_network_client.NetworkClient("robot_001")
    client.initialize()
    
    ledger = client.get_ledger_manager()
    
    # Create a motion entry
    motion = robocon_network_client.MotionEntry(
        entry_id="motion_001",
        robot_id="robot_001"
    )
    
    motion.pose = Pose(x=10.5, y=20.3, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0)
    motion.precision = 0.01
    motion.resolution = 0.1
    motion.shape_hash = "shape_hash_001"
    
    # Append to ledger
    ledger.append_motion_ledger(motion)
    print("Motion entry appended")
    
    # Verify ledger integrity
    if ledger.verify_motion_ledger("robot_001"):
        print("Motion ledger verified")
    
    # Retrieve motion ledger
    motions = ledger.get_motion_ledger("robot_001")
    print(f"Motion ledger contains {len(motions)} entries")
    
    for m in motions:
        print(f"Motion: {m.entry_id} at ({m.pose.x}, {m.pose.y}, {m.pose.z})")
    
    # Create goal entry
    goal = robocon_network_client.GoalEntry(
        entry_id="goal_001",
        robot_id="robot_001",
        natural_language_goal="Move sheathing to building B, floor 2, room 5"
    )
    
    # Link goal to motion entries
    ledger.link_goal_to_motions("goal_001", ["motion_001", "motion_002"])
    
    # Append goal ledger
    ledger.append_goal_ledger(goal)
    print("Goal entry appended")
    
    client.shutdown()
    return 0

if __name__ == "__main__":
    main()
```

### Example 4: Sensor Data Request

```python
#!/usr/bin/env python3
import robocon_network_client
import time

def main():
    client = robocon_network_client.NetworkClient("robot_001")
    client.initialize()
    
    # Set up sensor data callback (for robot mode)
    def sensor_data_callback(request):
        print(f"Handling request for {request.sensor_type} data")
        
        data = robocon_network_client.SensorData(
            data_id=f"response_{request.request_id}",
            robot_id="robot_001",
            sensor_type=request.sensor_type
        )
        
        # Generate sensor data based on type
        if request.sensor_type == "lidar":
            # Generate LiDAR data...
            data.data = generate_lidar_data()
            data.resolution = 0.1
            data.precision = 0.01
        
        return data
    
    client.set_sensor_data_request_callback(sensor_data_callback)
    
    # Request sensor data from another robot
    request = robocon_network_client.SensorDataRequest(
        request_id="req_001",
        robot_id="robot_002",
        sensor_type="lidar"
    )
    
    response = client.send_sensor_data_request(request, timeout_ms=5000)
    
    if response.success:
        print(f"Received {len(response.data.data)} bytes")
    
    client.shutdown()
    return 0

def generate_lidar_data():
    # Generate mock LiDAR data
    import random
    data = bytearray()
    for _ in range(1000):
        distance = random.uniform(0.0, 50.0)
        data.extend(distance.to_bytes(4, byteorder='little'))
    return bytes(data)

if __name__ == "__main__":
    main()
```

### Example 5: Zero-Trust Identity Verification

```python
#!/usr/bin/env python3
import robocon_network_client

def main():
    client = robocon_network_client.NetworkClient("robot_001")
    client.initialize()
    
    # Discover peers
    peers = client.discover_peers(5000)
    
    for peer in peers:
        print(f"Verifying peer: {peer.peer_id}")
        
        # Verify peer identity
        if client.verify_peer_identity(peer.peer_id):
            print("Peer identity verified")
            
            # Get peer's motion ledger hash
            ledger = client.get_ledger_manager()
            latest_hash = ledger.get_latest_motion_hash(peer.peer_id)
            
            if latest_hash:
                print(f"Latest motion hash: {latest_hash}")
                
                # Verify motion hash
                if client.verify_motion_hash(peer.peer_id, latest_hash):
                    print("Motion hash verified")
        else:
            print("Peer identity verification failed")
    
    client.shutdown()
    return 0

if __name__ == "__main__":
    main()
```

### Example 6: Complete Multi-Robot Coordination

```python
#!/usr/bin/env python3
import robocon_network_client
import time
import threading

class MultiRobotCoordinator:
    def __init__(self, robot_id):
        self.client = robocon_network_client.NetworkClient(robot_id)
        self.robot_id = robot_id
        self.running = False
    
    def initialize(self):
        if not self.client.initialize():
            return False
        
        # Set up callbacks
        consensus = self.client.get_consensus_manager()
        consensus.set_consensus_callback(self.on_consensus_result)
        
        self.client.set_sensor_data_request_callback(self.on_sensor_request)
        
        return True
    
    def on_consensus_result(self, result):
        if result.consensus_reached:
            print(f"[{self.robot_id}] Consensus reached: {result.action_id}")
            print(f"[{self.robot_id}] Executor: {result.executor_id}")
            
            if result.executor_id == self.robot_id:
                print(f"[{self.robot_id}] Executing action...")
                self.execute_action(result.action_id)
            else:
                print(f"[{self.robot_id}] Monitoring executor...")
                self.monitor_executor(result.executor_id)
    
    def on_sensor_request(self, request):
        print(f"[{self.robot_id}] Sensor request: {request.sensor_type}")
        # Generate sensor data...
        return self.generate_sensor_data(request)
    
    def propose_action(self, action_type, target_location):
        action = robocon_network_client.ActionSuggestion(
            action_id=f"action_{int(time.time())}",
            proposer_id=self.robot_id,
            action_type=action_type
        )
        action.target_location = target_location
        action.estimated_cost = 150.0
        action.required_capabilities = ["manipulation", "navigation"]
        
        consensus = self.client.get_consensus_manager()
        return consensus.propose_action(action)
    
    def execute_action(self, action_id):
        print(f"[{self.robot_id}] Executing action: {action_id}")
        # Action execution logic...
        time.sleep(5)  # Simulate execution
        print(f"[{self.robot_id}] Action completed: {action_id}")
    
    def monitor_executor(self, executor_id):
        print(f"[{self.robot_id}] Monitoring executor: {executor_id}")
        # Monitoring logic...
    
    def generate_sensor_data(self, request):
        data = robocon_network_client.SensorData(
            data_id=f"data_{request.request_id}",
            robot_id=self.robot_id,
            sensor_type=request.sensor_type
        )
        # Generate actual sensor data...
        return data
    
    def run(self):
        self.running = True
        
        # Discover peers
        peers = self.client.discover_peers(5000)
        print(f"[{self.robot_id}] Discovered {len(peers)} peers")
        
        # Main loop
        while self.running:
            time.sleep(1)
            # Periodic operations...
    
    def shutdown(self):
        self.running = False
        self.client.shutdown()

def main():
    coordinator = MultiRobotCoordinator("robot_001")
    
    if not coordinator.initialize():
        print("Failed to initialize")
        return 1
    
    # Run in separate thread
    thread = threading.Thread(target=coordinator.run)
    thread.start()
    
    # Propose an action
    time.sleep(2)
    result = coordinator.propose_action(
        "move_sheathing",
        "building_B_floor_2_room_5"
    )
    
    # Keep running
    try:
        thread.join()
    except KeyboardInterrupt:
        coordinator.shutdown()
    
    return 0

if __name__ == "__main__":
    main()
```

## Building and Running Examples

### C++

```bash
# Build example
g++ -std=c++17 example.cpp -lrobocon_network_client -o example

# Run example
./example
```

### Python 3

```bash
# Install Python bindings (if needed)
# pip install robocon-network-client

# Run example
python3 example.py
```

## Next Steps

- [Overview](overview.md) - Protocol overview and architecture
- [API Reference](api-reference.md) - Complete API documentation
- [Discovery Flow](discovery-flow.md) - Discovery protocol details
- [Voting Flow](voting-flow.md) - Voting protocol details

