#ifndef ROUTINGPROTOCOLIMPL_H
#define ROUTINGPROTOCOLIMPL_H

#include "global.h"
#include "RoutingProtocol.h"
#include "Node.h"
#include <arpa/inet.h>
#include <unordered_map>
#include <vector>
#include <set>

/// Timer constants used for periodic tasks in the protocol
namespace Timers {
    constexpr unsigned int PingAlarm = 10000;   // Period for PING packets: 10 seconds
    constexpr unsigned int LSTimeout = 45000;  // Timeout for LS records: 45 seconds
    constexpr unsigned int DVTimeout = 45000;  // Timeout for DV records: 45 seconds
    constexpr unsigned int LSAlarm = 30000;    // Period for LS updates: 30 seconds
    constexpr unsigned int CheckAlarm = 1000;  // Period for topology checks: 1 second
    constexpr unsigned int PongTimeout = 10000; // Timeout for PONG response: 10 seconds
    constexpr unsigned int DVAlarm = 30000;    // Period for DV updates: 30 seconds
}


/// LS_Entry: Represents a link state Entry in the LS protocol
struct LS_Entry {
    unsigned int expire_timeout; ///< Expiration time for the link record
    unsigned short hop_id;       ///< Neighbor ID
    unsigned short linkcost;     ///< Cost to reach the neighbor

    /// Constructor
    LS_Entry(unsigned int expire_timeout_, unsigned short hop_id_, unsigned short linkcost_)
        : expire_timeout(expire_timeout_), hop_id(hop_id_), linkcost(linkcost_) {}

    /// Copy constructor
    LS_Entry(LS_Entry *ls_rec)
        : expire_timeout(ls_rec->expire_timeout), hop_id(ls_rec->hop_id), linkcost(ls_rec->linkcost) {}
};

/// LinkTable: Represents an entry in the link map
struct LinkTable {
    unsigned int expire_timeout; ///< Expiration time for the link
    unsigned short port_ID;      ///< Port ID associated with this link
};


/// Class for Link State (LS) protocol
class LS_P {
public:
    LS_P(unsigned short router_id); ///< Constructor
    ~LS_P(); ///< Destructor

    void setRouterID(unsigned short router_id); ///< Set the router ID for initialization
    void updateShortestPath(unsigned short temp_id, unsigned short min_cost,
                                  std::unordered_map<unsigned short, std::pair<unsigned short, unsigned short>>& hopcost);
    void removeLS(unsigned short hop_id);
    bool removeExpiredRecords(unsigned int time);
    void updateLS(std::set<unsigned short>& changed_s_ID);
    void changeTopology(unsigned short n_id);
    bool checkLS(unsigned int time);
    LS_Entry* returnLS(unsigned short s_ID);
    void updateLsState(char* packet, unsigned int timeout, unsigned int time, unsigned short size);
    bool updateNeighborLink(unsigned short s_ID, unsigned int timeout, unsigned short linkcost, unsigned int time);
    void shortestPath(std::unordered_map<unsigned short, unsigned short>& routingtable);
    void createLS(char* packet, unsigned short packet_size);
    void increSeq(); ///< Increment sequence number
    bool checkSeqNum(char* packet);

    /// Data members for managing LS protocol state
    std::unordered_map<unsigned short, unsigned int> id2seq; ///< Map of node IDs to sequence numbers
    std::unordered_map<unsigned short, std::vector<LS_Entry*>*> recordtable; ///< Link state database
    std::vector<LS_Entry*> linkstate; ///< Active link states

private:
    void upsertLink(unsigned short hop_id, unsigned short linkcost, unsigned int timeout, unsigned int time);
    void addNodeToPacket(char* packet, int offset, unsigned short node_ID, unsigned short linkcost);

    unsigned short router_id; ///< ID of the router
    unsigned int seqnum; ///< Sequence number for LS packets
};


/// Class for managing routing protocol functionalities
class RoutingProtocolImpl : public RoutingProtocol {
public:
    RoutingProtocolImpl(Node *n); ///< Constructor
    ~RoutingProtocolImpl(); ///< Destructor

    /// Initialization and periodic tasks
    void init(unsigned short num_ports, unsigned short router_id, eProtocolType protocol_type);
    void handle_alarm(void *data);
    void setAlarmType(RoutingProtocol *r, unsigned int duration, void *d); 

    /// Packet handling
    void recv(unsigned short port, void *packet, unsigned short size);
    void recvData(char* packet, unsigned short size);
    void recvPing(unsigned short port, char* packet, unsigned short size);
    void recvPong(unsigned short port, char* packet);
    void recvLS(unsigned short port, char* packet, unsigned short size);
    void recvDV(char* packet, unsigned short size);

    /// DV operations
    void sendDV();
    void sendDVToNeighbor(unsigned short port_ID, unsigned short d_ID);
    void updateRoutingTableDV();

    /// Timer management
    
    void resetAlarm(const char* alarmType);
    void pingTime();
    void checkTopo();
    void lsTime();
    void dvTime();

    /// Packet creation
    void initPacketHeader(char* packet, char type, unsigned short packet_size);
    void sendPing(int port);
    void sendLS();
    char* createPong(unsigned short recv_id, unsigned int sendtime, unsigned short packet_size);
    void addDVEntry(char* packet, int offset, unsigned short node_ID, unsigned short cost, unsigned short next_hop, unsigned short d_ID);

    /// Link and topology management
    void updateNeighborCost(unsigned short s_ID, unsigned short port, unsigned short linkcost);
    bool detectLink();
    bool cleanLinks(std::set<unsigned short>& changed_s_ID);

    /// DV table management
    bool cleanDV();

    bool updateDVTable();


    /// Utility functions
    void freePacket(void* packet);
    void validatePacket(char* packet, unsigned short size);
    void updateTable(unsigned short s_ID, char* packet, unsigned short size);

    /// Data members for managing protocol state
    Node *sys; ///< Pointer to the system interface
    unsigned short num_ports; ///< Number of router ports
    unsigned short router_id; ///< ID of the router
    eProtocolType protocol; ///< Current protocol type (DV or LS)

    /// Alarm types
    const char* ping = "ping";
    const char* distancevector = "distancevector";
    const char* linkstate = "linkstate";
    const char* update = "update";

    /// Routing and link data
    std::unordered_map<unsigned short, LinkTable> linkmap; ///< Link state table
    std::unordered_map<unsigned short, unsigned short> routingtable; ///< Routing table
    std::unordered_map<unsigned short, unsigned short> linkcosts; ///< Neighbor link costs
    std::unordered_map<unsigned short, std::pair<unsigned short, std::pair<unsigned short, unsigned int>>> dvtable; ///< Distance Vector table

    /// LS protocol instance
    LS_P* ls = nullptr; ///< LS protocol handler
};



#endif // ROUTINGPROTOCOLIMPL_H
