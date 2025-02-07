#include "RoutingProtocolImpl.h"
#include <string.h>
#include <algorithm>

// Constants
constexpr int pingSize = 12;        // Size of a PING packet
constexpr int pongSize = 12;        // Size of a PONG packet
constexpr unsigned short MAX_VAL = 65535; // Maximum value for link cost

// Class Constructor: Initializes the routing protocol for the node
RoutingProtocolImpl::RoutingProtocolImpl(Node *n) : RoutingProtocol(n) {
    sys = n;  // Assign the system node
}

// Class Destructor: Cleans up resources and clears protocol-specific data
RoutingProtocolImpl::~RoutingProtocolImpl() {
    // Clear the link map (adjacency information for neighbors)
    for (auto& entry : linkmap) {
        linkmap.erase(entry.first);
    }

    // If using Link State (LS) protocol, delete LS-specific structures
    if (protocol == P_LS && ls) {
        delete ls;
        ls = nullptr;
    }
}


// Alarm Utility: Sets an alarm for a specific duration and associates it with data
void RoutingProtocolImpl::setAlarmType(RoutingProtocol *r, unsigned int duration, void *d) {
    sys->set_alarm(r, duration, d); // Schedule the alarm
}

// Protocol Initialization: Configures the routing protocol based on user input
void RoutingProtocolImpl::init(unsigned short num_ports, unsigned short router_id, eProtocolType protocol_type) {
    // Ensure the protocol type is supported
    if (protocol_type != P_LS && protocol_type != P_DV) {
        fprintf(stderr, "Error: Unsupported protocol type\n");
        exit(EXIT_FAILURE);
    }

    // Store initialization parameters
    this->num_ports = num_ports;
    this->router_id = router_id;
    this->protocol = protocol_type;

    // Initialize periodic tasks: Ping and Update
    resetAlarm(this->ping);
    resetAlarm(this->update);

    // Set up protocol-specific tasks
    switch (protocol_type) {
        case P_LS:
            ls = new LS_P(router_id); // Initialize Link State protocol
            resetAlarm(this->linkstate);
            break;
        case P_DV:
            resetAlarm(this->distancevector);
            break;
    }
}

// Alarm Handler: Executes tasks based on the alarm type
void RoutingProtocolImpl::handle_alarm(void *data) {
    char* alarmtype = reinterpret_cast<char*>(data);

    // Compare the alarm type and invoke the corresponding handler
    if (strcmp(alarmtype, this->ping) == 0) {
        pingTime();
    } else if (strcmp(alarmtype, this->update) == 0) {
        checkTopo();
    } else if (strcmp(alarmtype, this->distancevector) == 0) {
        dvTime();
    } else if (strcmp(alarmtype, this->linkstate) == 0) {
        lsTime();
    } else {
        // Error for unrecognized alarm types
        fprintf(stderr, "Not able to recognize %s alarm\n", alarmtype);
        exit(EXIT_FAILURE);
    }
}


//Resets and schedules a new alarm for a specific task
void RoutingProtocolImpl::resetAlarm(const char* alarmType) {
    unsigned int duration = 0;

    // Determine the duration based on the type of alarm
    if (strcmp(alarmType, this->ping) == 0) {
        duration = Timers::PingAlarm;
    } else if (strcmp(alarmType, this->update) == 0) {
        duration = Timers::CheckAlarm;
    } else if (strcmp(alarmType, this->distancevector) == 0) {
        duration = Timers::DVAlarm;
    } else if (strcmp(alarmType, this->linkstate) == 0) {
        duration = Timers::LSAlarm;
    }

    // Set the alarm with the calculated duration
    setAlarmType(this, duration, (void*)alarmType);
}

// Handles the periodic ping operation for all ports
void RoutingProtocolImpl::pingTime() {
    // Send PING to all ports
    for (int i = 0; i < this->num_ports; i++) {
        sendPing(i);
    }
    // Reset the timer for the next PING event
    resetAlarm(this->ping);
}

// Handles periodic tasks for Link State (LS) protocol
void RoutingProtocolImpl::lsTime() {
    sendLS();              // Send Link State packets
    resetAlarm(this->linkstate); // Reset the LS timer
}

// Handles periodic tasks for Distance Vector (DV) protocol
void RoutingProtocolImpl::dvTime() {
    sendDV();              // Send Distance Vector updates
    resetAlarm(this->distancevector); // Reset the DV timer
}

// Checks the topology and triggers updates for LS or DV protocols
void RoutingProtocolImpl::checkTopo() {
    // Detect changes in topology or clean expired entries
    bool topologyChanged = detectLink(); 
    bool dvChanged = cleanDV();         

    if (this->protocol == P_DV && (topologyChanged || dvChanged)) {
        sendDV(); // Update and send Distance Vector if required
    } else if (this->protocol == P_LS && topologyChanged) {
        ls->shortestPath(routingtable); // Recalculate shortest paths
        sendLS();                       // Send updated LS packets
    }

    // Reset the topology-check timer
    resetAlarm(this->update);
}


//send DV
void RoutingProtocolImpl::sendDV() {
    for (const auto& entry : linkmap) {
        sendDVToNeighbor(entry.second.port_ID, entry.first);
    }
}


// Initializes the packet header with common fields
void RoutingProtocolImpl::initPacketHeader(char* packet, char type, unsigned short packet_size) {
    *packet = type;                                 // Set the packet type
    *(unsigned short*)(packet + 2) = htons(packet_size);  // Set the packet size (network byte order)
    *(unsigned short*)(packet + 4) = htons(this->router_id); // Set the source router ID
}


// Sends a PING packet through the specified port
void RoutingProtocolImpl::sendPing(int port) {
    char* ping_packet = (char*)malloc(pingSize); // Allocate memory for the PING packet
    initPacketHeader(ping_packet, PING, pingSize); // Initialize the packet header
    *(unsigned int*)(ping_packet + 8) = htonl(sys->time()); // Add the current time to the packet
    sys->send(port, ping_packet, pingSize); // Send the packet through the specified port
}


// Sends a Link State (LS) packet to all neighbors
void RoutingProtocolImpl::sendLS() {
    unsigned short ls_pack_size = 12 + (linkmap.size() * 4); // Calculate packet size based on the number of links
    for (auto& entry : linkmap) {
        char* ls_packet = (char*)malloc(ls_pack_size); // Allocate memory for the LS packet
        initPacketHeader(ls_packet, LS, ls_pack_size); // Initialize the packet header
        ls->createLS(ls_packet, ls_pack_size);         // Fill the LS packet with link state data
        sys->send(entry.second.port_ID, ls_packet, ls_pack_size); // Send the packet to the neighbor
    }
    ls->increSeq(); // Increment the sequence number for the next LS packet
}


// Sends a Distance Vector (DV) update to a specific neighbor
void RoutingProtocolImpl::sendDVToNeighbor(unsigned short port_ID, unsigned short d_ID) {
    unsigned short num_entries = dvtable.size(); // Count the number of DV entries
    unsigned short packet_size = 8 + num_entries * 4; // Calculate the DV packet size
    char* dv_packet = (char*)malloc(packet_size); // Allocate memory for the DV packet
    initPacketHeader(dv_packet, DV, packet_size); // Initialize the packet header
    *(unsigned short*)(dv_packet + 6) = htons(d_ID); // Add destination router ID to the packet

    // Populate the DV entries
    int offset = 8;
    for (const auto& entry : dvtable) {
        unsigned short node_ID = entry.first;           // Node ID
        unsigned short cost = entry.second.first;       // Cost to the node
        unsigned short next_hop = entry.second.second.first; // Next hop
        addDVEntry(dv_packet, offset, node_ID, cost, next_hop, d_ID); // Add the DV entry to the packet
        offset += 4;
    }
    sys->send(port_ID, dv_packet, packet_size); // Send the DV packet to the neighbor
}


// Adds an entry to the Distance Vector (DV) packet
void RoutingProtocolImpl::addDVEntry(char* packet, int offset, unsigned short node_ID, unsigned short cost, unsigned short next_hop, unsigned short d_ID) {
    // Apply split horizon with poison reverse
    if (d_ID == next_hop) {
        cost = MAX_VAL; // Poison the route if the next hop is the same as the destination
    }

    // Add the DV entry to the packet
    *(unsigned short*)(packet + offset) = htons(node_ID); 
    *(unsigned short*)(packet + offset + 2) = htons(cost); 
}


// Handles received DATA packets
void RoutingProtocolImpl::recvData(char* packet, unsigned short size) {
    // Extract the destination ID from the DATA packet
    unsigned short dest_ID = ntohs(*(unsigned short*)(packet + 6));

    // Check if the packet has reached its final destination
    if (dest_ID == this->router_id) {
        freePacket(packet); 
        return;             
    }
    updateTable(dest_ID, packet, size);
}



// Creates a PONG packet in response to a PING
char* RoutingProtocolImpl::createPong(unsigned short recv_id, unsigned int sendtime, unsigned short packet_size) {
    char* pong_packet = (char*)malloc(packet_size); // Allocate memory for the PONG packet
    initPacketHeader(pong_packet, PONG, packet_size); // Initialize the packet header
    *(unsigned short*)(pong_packet + 6) = htons(recv_id); // Add the receiver ID
    *(unsigned int*)(pong_packet + 8) = htonl(sendtime);  // Add the send time
    return pong_packet; // Return the created packet
}

// Updates the neighbor cost and link map based on received PONG
void RoutingProtocolImpl::updateNeighborCost(unsigned short s_ID, unsigned short port, unsigned short linkcost) {
    linkcosts[s_ID] = linkcost; // Update link cost
    unsigned int expire_timeout = sys->time() + Timers::PongTimeout;

    // Update or insert link map entry
    auto it = linkmap.find(s_ID);
    if (it != linkmap.end()) {
        it->second.expire_timeout = expire_timeout;
    } else {
        linkmap[s_ID] = {expire_timeout, port};
    }
}

// Handles received PING packets
void RoutingProtocolImpl::recvPing(unsigned short port, char* packet, unsigned short size) {
    if (size != pingSize) { // Validate packet size
        freePacket(packet); // Free memory if invalid
        return;
    }

    unsigned short recv_id = ntohs(*(unsigned short*)(packet + 4)); // Extract the sender ID
    unsigned int sendtime = ntohl(*(unsigned int*)(packet + 8)); // Extract the send time

    // Create a PONG response and send it back
    char* pong_packet = createPong(recv_id, sendtime, pongSize); 
    sys->send(port, pong_packet, pongSize);

    freePacket(packet); // Free memory after handling the packet
}

// Handles received PONG packets and updates neighbor costs
void RoutingProtocolImpl::recvPong(unsigned short port, char* packet) {
    unsigned short recv_id = ntohs(*(unsigned short*)(packet + 6)); // Extract the receiver ID
    if (this->router_id != recv_id) { // Validate receiver ID
        freePacket(packet);
        return;
    }

    unsigned int sendtime = ntohl(*(unsigned int*)(packet + 8)); // Extract send time
    unsigned short s_ID = ntohs(*(unsigned short*)(packet + 4)); // Extract sender ID
    unsigned short linkcost = sys->time() - sendtime; // Calculate link cost

    updateNeighborCost(s_ID, port, linkcost); // Update the link cost for the neighbor

    // Handle protocol-specific actions
    switch (this->protocol) {
        case P_LS:
            if (ls->updateNeighborLink(s_ID, Timers::LSTimeout, linkcost, sys->time())) {
                ls->shortestPath(routingtable); // Recalculate shortest paths
                sendLS();                       // Send updated LS packets
            }
            break;
        case P_DV:
            if (updateDVTable()) sendDV(); // Update and send DV updates
            break;
    }

    freePacket(packet); // Free memory after handling the packet
}



// Handles received Link State (LS) packets
void RoutingProtocolImpl::recvLS(unsigned short port, char* packet, unsigned short size) {
    // Check sequence number; discard if invalid or outdated
    if (!ls->checkSeqNum(packet)) {
        freePacket(packet);
        return;
    }

    // Update Link State information
    ls->updateLsState(packet, Timers::LSTimeout, sys->time(), size);

    // Flood the packet to all neighbors except the source
    for (const auto& link : linkmap) {
        if (link.second.port_ID != port) { 
            char* flood = (char*)malloc(size);
            memcpy(flood, packet, size);
            sys->send(link.second.port_ID, flood, size);
        }
    }

    // Recompute shortest paths and update routing table
    ls->shortestPath(routingtable);

    
    freePacket(packet);
}





// Handles received packets based on their type
void RoutingProtocolImpl::recv(unsigned short port, void *packet, unsigned short size) {
    char type = *(char*)packet; // Extract the packet type from the first byte

    // Invoke the appropriate handler based on packet type
    switch (type) {
        case DATA:
            recvData((char*)packet, size);
            break;
        case PING:
            recvPing(port, (char*)packet, size);
            break;
        case PONG:
            recvPong(port, (char*)packet);
            break;
        case LS:
            recvLS(port, (char*)packet, size);
            break;
        case DV:
            recvDV((char*)packet, size);
            break;
        default:
            fprintf(stderr, "Packet received has incorrect type\n");
            freePacket(packet); // Free memory if packet type is invalid
            break;
    }
}


// Frees dynamically allocated memory for a packet
void RoutingProtocolImpl::freePacket(void* packet) {
    free(packet); // Free the allocated packet memory
}

// Handles received Distance Vector (DV) packets
void RoutingProtocolImpl::recvDV(char* packet, unsigned short size) {
    // Extract source router ID from the packet
    unsigned short s_ID = ntohs(*(unsigned short*)(packet + 4));
    
    // Check if source is a neighbor, if not discard the packet
    auto lit = linkcosts.find(s_ID);
    if (lit == linkcosts.end()) {
        freePacket(packet);
        return;
    }

    // Parse DV entries from packet into separate vectors for IDs and costs
    vector<unsigned short> node_IDs;
    vector<unsigned short> cost_VYs;
    for (int i = 8; i < size; i += 4) {
        node_IDs.push_back(ntohs(*(unsigned short*)(packet + i)));
        cost_VYs.push_back(ntohs(*(unsigned short*)(packet + i + 2)));
    }

    bool isUpdated = false;
    unsigned short cost_AV = lit->second;  // Cost to the neighbor node
    unsigned int current_time = sys->time();

    // Process each destination in the DV update
    for (size_t i = 0; i < node_IDs.size(); i++) {
        unsigned short node_ID = node_IDs[i];
        unsigned short cost_VY = cost_VYs[i];

        // Skip entries for self or with infinite cost
        if (node_ID == this->router_id || cost_VY == MAX_VAL) {
            continue;
        }

        unsigned short cost_AYV = cost_AV + cost_VY;  // Total path cost
        auto it = dvtable.find(node_ID);

        if (it == dvtable.end()) {
            // Add new destination to DV table
            dvtable[node_ID] = std::make_pair(cost_AYV,
                              std::make_pair(s_ID, current_time));
            isUpdated = true;
        } else {
            unsigned short current_cost = it->second.first;
            unsigned short current_hop = it->second.second.first;

            // Update route if:
            // 1. Found better path, or
            // 2. Current next hop reports increased cost
            if (cost_AYV < current_cost ||
                (cost_AYV > current_cost && current_hop == s_ID)) {
                it->second = std::make_pair(cost_AYV,
                            std::make_pair(s_ID, current_time));
                isUpdated = true;
            } else if (current_hop == s_ID) {
                // Just update timestamp for current route
                it->second.second.second = current_time;
            }
        }
    }

    // Clean up stale routes
    vector<unsigned short> toErase;
    for (const auto& entry : dvtable) {
        unsigned short dest_ID = entry.first;
        unsigned short hop = entry.second.second.first;
        
        // Check routes going through the source
        if (dest_ID != hop && hop == s_ID) {
            // Remove if destination is no longer in source's DV
            if (std::find(node_IDs.begin(), node_IDs.end(), dest_ID) == node_IDs.end()) {
                toErase.push_back(dest_ID);
                isUpdated = true;
            }
        }
    }

    // Remove all stale routes
    for (unsigned short dest_ID : toErase) {
        dvtable.erase(dest_ID);
    }

    // If routes changed, update routing table and send updates
    if (isUpdated) {
        updateRoutingTableDV();
        sendDV();
    }

    freePacket(packet);
}


// Removes stale entries from the DV table
bool RoutingProtocolImpl::cleanDV() {
    bool isUpdated = false;
    unsigned int current_time = sys->time();
    vector<unsigned short> toErase;

    // Identify entries that have expired
    for (const auto& entry : dvtable) {
        unsigned short dest_ID = entry.first;
        unsigned int last_refresh_time = entry.second.second.second;

        if (current_time - last_refresh_time > Timers::DVTimeout) { // Check expiration
            toErase.push_back(dest_ID);
            isUpdated = true;
        }
    }

    // Remove expired entries
    for (unsigned short dest_ID : toErase) {
        dvtable.erase(dest_ID);
    }

    // Update routing table if entries were removed
    if (isUpdated) {
        updateRoutingTableDV();
    }

    return isUpdated; // Return whether any entries were cleaned
}










// Updates the Distance Vector (DV) table and returns if changes occurred
bool RoutingProtocolImpl::updateDVTable() {
   bool isUpdated = false;
   unsigned int current_time = sys->time();

   // 1. Update routes for direct neighbors
   for (const auto& [neighbor_ID, cost] : linkcosts) {
       auto it = dvtable.find(neighbor_ID);
       if (it != dvtable.end()) {
           if (it->second.second.first == neighbor_ID) {
               if (it->second.first != cost) {
                   // Update route if cost changed
                   it->second = std::make_pair(cost,
                               std::make_pair(neighbor_ID, current_time));
                   isUpdated = true;
               } else {
                   // Only update timestamp
                   it->second.second.second = current_time;
               }
           }
       } else {
           // Add new neighbor route
           dvtable[neighbor_ID] = std::make_pair(cost,
                                 std::make_pair(neighbor_ID, current_time));
           isUpdated = true;
       }
   }

   // 2. Check if any existing routes could be improved by direct links
   for (auto& [dest_ID, cost_hop_time] : dvtable) {
       auto lit = linkcosts.find(dest_ID);
       if (lit != linkcosts.end()) {
           unsigned short direct_cost = lit->second;
           if (direct_cost < cost_hop_time.first) {
               // Update to use direct route if it's better
               cost_hop_time = std::make_pair(direct_cost,
                              std::make_pair(dest_ID, current_time));
               isUpdated = true;
           }
       }
   }

   // 3. Remove routes through failed links
   vector<unsigned short> toErase;
   for (const auto& [dest_ID, cost_hop_time] : dvtable) {
       unsigned short next_hop = cost_hop_time.second.first;
       if (linkcosts.find(next_hop) == linkcosts.end()) {
           // Mark route for removal if next hop is no longer accessible
           toErase.push_back(dest_ID);
           isUpdated = true;
       }
   }

   // Remove all marked routes
   for (unsigned short dest_ID : toErase) {
       dvtable.erase(dest_ID);
   }

   // 4. Update routing table if changes occurred
   if (isUpdated) {
       updateRoutingTableDV();
   }

   return isUpdated;
}


// Updates the routing table based on the DV table
void RoutingProtocolImpl::updateRoutingTableDV() {
    vector<unsigned short> toErase;

    // Update routing table with valid DV entries
    for (auto& [dest_ID, cost_hop_time] : dvtable) {
        routingtable[dest_ID] = cost_hop_time.second.first; // Set next hop
    }

    // Identify invalid entries for removal
    for (auto& [dest_ID, hop] : routingtable) {
        if (dvtable.find(dest_ID) == dvtable.end()) {
            toErase.push_back(dest_ID);
        }
    }

    // Remove invalid entries from the routing table
    for (auto& dest_ID : toErase) {
        routingtable.erase(dest_ID);
    }
}


// Validates a received packet by checking its size and type
void RoutingProtocolImpl::validatePacket(char* packet, unsigned short size) {
    if (size < 8) { // Minimum valid packet size is 8 bytes
        fprintf(stderr, "Packet size is invalid: %d bytes\n", size);
        freePacket(packet); // Free invalid packet
        return;
    }

    char type = *packet; // Extract the packet type
    // Validate the packet type
    if (type != DATA && type != PING && type != PONG && type != LS && type != DV) {
        fprintf(stderr, "Invalid packet type: %d\n", type);
        freePacket(packet); // Free invalid packet
        return;
    }

    unsigned short packet_size = ntohs(*(unsigned short*)(packet + 2)); // Extract size
    // Validate the declared size matches the actual size
    if (packet_size != size) {
        fprintf(stderr, "Packet size mismatch: Expected %d, Received %d\n", packet_size, size);
        freePacket(packet);
    }
}

// Updates the routing table when the DV table changes
void RoutingProtocolImpl::updateTable(unsigned short s_ID, char* packet, unsigned short size) {
    auto it = routingtable.find(s_ID);
    if (it != routingtable.end()) {
        unsigned short next_hop = it->second;
        auto link_it = linkmap.find(next_hop);
        if (link_it != linkmap.end()) {
            sys->send(link_it->second.port_ID, packet, size); // Forward packet to next hop
            return;
        }
    }

    // Log unreachable destination and free the packet
    fprintf(stderr, "Destination %d is unreachable\n", s_ID);
    freePacket(packet);
}

// Detects link changes and updates the topology
bool RoutingProtocolImpl::detectLink() {
    set<unsigned short> changed_s_ID; // Set to track IDs of changed links
    bool ischange = cleanLinks(changed_s_ID); // Remove expired links and track changes

    if (ischange) {
        // Handle protocol-specific actions for link changes
        if (this->protocol == P_LS) {
            ls->updateLS(changed_s_ID);     // Update Link State records
            ls->shortestPath(routingtable); // Recalculate shortest paths
        } else {
            if (updateDVTable()) sendDV();  // Update DV table and send updates
        }
    }
    return ischange; // Return whether topology changed
}


// Cleans up expired links from the link map
bool RoutingProtocolImpl::cleanLinks(set<unsigned short>& changed_s_ID) {
    bool ischange = false;
    auto it = linkmap.begin();

    while (it != linkmap.end()) {
        LinkTable& lnk = it->second;
        if (sys->time() > lnk.expire_timeout) {
            linkcosts.erase(it->first);       // Remove link from cost table
            changed_s_ID.insert(it->first);  // Mark the link as changed
            it = linkmap.erase(it);          // Remove link from the map
            ischange = true;                 // Indicate a change
        } else {
            ++it;
        }
    }
    return ischange; // Return whether any links were cleaned
}







// Constructor: Initializes the Link State Protocol for a router
LS_P::LS_P(unsigned short router_id) 
    : router_id(router_id), seqnum(0) {
    linkstate.clear(); // Clear the local link state records
    id2seq.clear();    // Clear the sequence number map
    recordtable.clear(); // Clear the global link state record table
}




//initialize the router to start sending LS Packets
void LS_P::setRouterID(unsigned short router_id) {
    this->router_id = router_id;
    this->seqnum = 0;       
    linkstate.clear();      
    id2seq.clear();         
}




// Destructor: Cleans up dynamically allocated resources
LS_P::~LS_P() {
    for (auto rec : linkstate) {
        free(rec); // Free each dynamically allocated link record
    }
    linkstate.clear();

    // Free all global link state records
    for (auto& [s_ID, rec_vec] : recordtable) {
        for (auto rec : *rec_vec) {
            free(rec); // Free individual records
        }
        delete rec_vec; // Delete record vectors
    }
    recordtable.clear();
}


// Updates the topology by removing all records related to a specific node
void LS_P::changeTopology(unsigned short n_ID) {
    auto it = recordtable.find(n_ID);
    if (it == recordtable.end()) return; // If node ID not found, return

    auto rec_vec = it->second;
    for (auto rec : *rec_vec) {
        free(rec); // Free each link record
    }
    recordtable.erase(it); // Erase the node's records
    delete rec_vec;        // Delete the vector itself
}



// Adds or updates a link state entry for a neighbor
void LS_P::upsertLink(unsigned short hop_id, unsigned short linkcost, unsigned int timeout, unsigned int time) {
    LS_Entry* rec = returnLS(hop_id); // Look for an existing record
    if (rec) {
        rec->expire_timeout = time + timeout; // Update expiration time
        rec->linkcost = linkcost;            // Update link cost
    } else {
        // Create a new record if none exists
        rec = static_cast<LS_Entry*>(malloc(sizeof(LS_Entry)));
        rec->hop_id = hop_id;
        rec->linkcost = linkcost;
        rec->expire_timeout = time + timeout;
        linkstate.push_back(rec); // Add the new record
    }
}


// Updates neighbor link cost and expiration time
bool LS_P::updateNeighborLink(unsigned short s_ID, unsigned int timeout, unsigned short linkcost, unsigned int time) {
    bool ischanged = false;
    LS_Entry* rec = returnLS(s_ID); // Look for the link in the state

    if (rec) {
        if (rec->linkcost != linkcost) {
            ischanged = true; // Mark as changed if link cost differs
            rec->linkcost = linkcost;
        }
        rec->expire_timeout = time + timeout; // Update expiration time
    } else {
        upsertLink(s_ID, linkcost, timeout, time); // Add new link state entry
        ischanged = true;
    }
    return ischanged; // Return whether a change occurred
}


// Processes incoming LS packets and updates the global link state
void LS_P::updateLsState(char* packet, unsigned int timeout, unsigned int time, unsigned short size) {
    unsigned short s_ID = ntohs(*(unsigned short*)(packet + 4)); // Extract sender ID

    if (!checkSeqNum(packet)) { // Discard packet if sequence number is stale
        return;
    }

    auto ls_rec_vec = new vector<LS_Entry*>(); // Prepare a new vector for link state records
    unsigned int record_count = (size - 12) / 4;

    // Parse all link records from the packet
    for (unsigned int i = 0; i < record_count; ++i) {
        unsigned int offset = 12 + (i * 4);
        unsigned short hop_id = ntohs(*(unsigned short*)(packet + offset));
        unsigned short linkcost = ntohs(*(unsigned short*)(packet + offset + 2));

        auto rec = static_cast<LS_Entry*>(malloc(sizeof(LS_Entry)));
        rec->hop_id = hop_id;
        rec->linkcost = linkcost;
        ls_rec_vec->push_back(rec); // Add record to the vector
    }

    // Update the global link state record table
    auto it = recordtable.find(s_ID);
    if (it != recordtable.end()) {
        for (auto rec : *(it->second)) {
            free(rec); // Free old records
        }
        delete it->second; // Delete old vector
    }
    recordtable[s_ID] = ls_rec_vec; // Store the new records

    upsertLink(s_ID, 0, timeout, time); // Update expiration time for the sender
}

// addNodeToPacket
void LS_P::addNodeToPacket(char* packet, int offset, unsigned short node_ID, unsigned short linkcost) {
    *(unsigned short*)(packet + offset) = (unsigned short)htons(node_ID);
    *(unsigned short*)(packet + offset + 2) = (unsigned short)htons(linkcost);
}

// Creates a Link State (LS) packet
void LS_P::createLS(char* packet, unsigned short packet_size) {
    *(char*)packet = LS;                                     // Set packet type
    *(unsigned short*)(packet + 2) = htons(packet_size);     // Set packet size
    *(unsigned short*)(packet + 4) = htons(this->router_id); // Add source ID
    *(unsigned int*)(packet + 8) = htonl(this->seqnum);      // Add sequence number

    int offset = 12; // Start after the header

    // Add link state records
    for (auto rec : linkstate) {
        addNodeToPacket(packet, offset, rec->hop_id, rec->linkcost);
        offset += 4; // Move to next record
    }
}



// Returns the LS record for the given node ID, or nullptr if not found
LS_Entry* LS_P::returnLS(unsigned short s_ID) {
    for (LS_Entry* rec : linkstate) {
        if (rec->hop_id == s_ID) {
            return rec; // Found the record
        }
    }
    return nullptr; // Not found
}



// Removes the link state record for the specified node ID
void LS_P::removeLS(unsigned short hop_id) {
    auto it = linkstate.begin();
    while (it != linkstate.end()) {
        LS_Entry* rec = *it;
        if (rec->hop_id == hop_id) {
            it = linkstate.erase(it); // Erase record
            free(rec);                
        } else {
            ++it; // Move to next record
        }
    }
}


// Updates Link State by removing and adjusting topology for changed nodes
void LS_P::updateLS(set<unsigned short>& changed_s_ID) {
    for (auto& id : changed_s_ID) {
        removeLS(id);    
        changeTopology(id);
    }
}


// Removes expired records from the local link state
bool LS_P::removeExpiredRecords(unsigned int time) {
    bool ischanged = false;
    auto it = linkstate.begin();

    // Iterate through the link state and remove expired records
    while (it != linkstate.end()) {
        if ((*it)->expire_timeout < time) {
            LS_Entry* rec = *it;
            it = linkstate.erase(it); // Remove the record from the list
            free(rec);                // Free the memory
            ischanged = true;
        } else {
            ++it;
        }
    }
    return ischanged; // Return whether changes occurred
}

// Checks and removes expired records from the link state
bool LS_P::checkLS(unsigned int time) {
    bool ischanged = removeExpiredRecords(time); // Remove stale records

    if (ischanged) {
        changeTopology(this->router_id); // Update topology if changes occurred
    }
    return ischanged;
}


// Calculates the shortest path for all nodes using Dijkstra's algorithm
void LS_P::shortestPath(unordered_map<unsigned short, unsigned short>& routingtable) {
    routingtable.clear(); // Clear the routing table
    unordered_map<unsigned short, pair<unsigned short, unsigned short>> hopcost; // Node to cost & hop

    // Initialize hop cost for direct links
    for (auto rec : linkstate) {
        hopcost[rec->hop_id] = make_pair(rec->linkcost, rec->hop_id);
    }

    // Dijkstra's algorithm for shortest path
    while (!hopcost.empty()) {
        // Find the node with the minimum cost
        auto min_it = min_element(hopcost.begin(), hopcost.end(),
                                  [](const auto& a, const auto& b) { return a.second.first < b.second.first; });
        unsigned short min_cost = min_it->second.first;
        unsigned short temp_id = min_it->first;

        routingtable[temp_id] = min_it->second.second; // Update routing table
        hopcost.erase(min_it); // Remove the processed node

        // Update costs for neighbors of the processed node
        updateShortestPath(temp_id, min_cost, hopcost);
    }
}

// Adds or updates the cost for neighbors of a processed node
void LS_P::updateShortestPath(unsigned short temp_id, unsigned short min_cost, unordered_map<unsigned short, pair<unsigned short, unsigned short>>& hopcost) {
    auto recit = recordtable.find(temp_id);
    if (recit != recordtable.end()) {
        for (auto rec : *recit->second) {
            unsigned short path_cost = min_cost + rec->linkcost;
            if (hopcost.find(rec->hop_id) == hopcost.end() || hopcost[rec->hop_id].first > path_cost) {
                hopcost[rec->hop_id] = make_pair(path_cost, temp_id); // Update cost and next hop
            }
        }
    }
}


// Increments the sequence number, preventing overflow
void LS_P::increSeq() {
    if (this->seqnum == MAX_VAL) { // Reset sequence number if it overflows
        this->seqnum = 0;
    } else {
        ++this->seqnum;
    }
}

// Checks and updates the sequence number of received LS packets
bool LS_P::checkSeqNum(char* packet) {
    unsigned short s_ID = ntohs(*(unsigned short*)(packet + 4)); // Extract source ID
    unsigned int seqNum = ntohl(*(unsigned int*)(packet + 8));   // Extract sequence number

    // Ignore packets originating from the current router
    if (s_ID == this->router_id) {
        return false;
    }

    auto it = id2seq.find(s_ID);
    // Update sequence number if the packet's sequence is newer
    if (it == id2seq.end() || it->second < seqNum) {
        id2seq[s_ID] = seqNum; // Store the new sequence number
        return true;
    }
    return false; // Sequence number is not newer
}