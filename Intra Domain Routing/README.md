Intra-Domain Routing Protocol Implementation for Bisco GSR9999 Router

Project Overview
This project implements two intra-domain routing protocols - Distance Vector (DV) and Link State (LS) - for the Bisco GSR9999 router. The implementation is done in C++ and runs on a GSR9999 simulator, providing accurate and robust routing capabilities.

Team Members
- Yihang Pan
I completed this project on my own.

Implementation Details

Core Components

1. RoutingProtocolImpl Class
- Handles initialization and core routing functionality
- Manages packet processing and timer events
- Implements both DV and LS protocol logic
- Maintains routing tables and neighbor information

2. LS_P Class (Link State Protocol)
- Implements Link State protocol specific functionality
- Manages link state database
- Implements Dijkstra's algorithm for shortest path calculation
- Handles sequence numbers and flooding mechanism

Key Features

Protocol Support
- Distance Vector Protocol**
  - Poison reverse implementation
  - 30-second update interval
  - 45-second route timeout
  - Triggered updates on topology changes

- Link State Protocol**
  - Flooding mechanism for LSA distribution
  - 30-second update interval
  - 45-second link state entry timeout
  - Dijkstra's algorithm implementation

Router Functionality
- Neighbor discovery using PING/PONG mechanism
- Dynamic routing table updates
- Link cost calculations based on round-trip delay
- Efficient packet forwarding

Packet Types
1. DATA: Regular data packets
2. PING: Neighbor discovery packets
3. PONG: Response to PING packets
4. DV: Distance Vector protocol updates
5. LS: Link State protocol updates



Building and Running

Prerequisites
- C++ compiler (supporting C++17)
- GSR9999 simulator environment

Compilation
make clean
make


Running Tests

./simulator test_file DV|LS




