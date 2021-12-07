/*
 * Copyright (c) 2008 Princeton University
 * Copyright (c) 2016 Georgia Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef __MEM_RUBY_NETWORK_GARNET2_0_GARNETNETWORK_HH__
#define __MEM_RUBY_NETWORK_GARNET2_0_GARNETNETWORK_HH__

#include <iostream>
#include <map>
#include <vector>

#include "mem/ruby/network/Network.hh"
#include "mem/ruby/network/fault_model/FaultModel.hh"
#include "mem/ruby/network/garnet2.0/CommonTypes.hh"
#include "params/GarnetNetwork.hh"

class FaultModel;
class NetworkInterface;
class Router;
class NetDest;
class NetworkLink;
class CreditLink;

//// Updown Routing
// code begin
using namespace std;
// code end

class GarnetNetwork : public Network
{
  public:
    typedef GarnetNetworkParams Params;
    GarnetNetwork(const Params *p);
    ~GarnetNetwork() = default;

    void init();

    // Configuration (set externally)

    //// Updown Routing
    // code begin
    void populate_routingTable(\
                       std::vector<int>& path_, int ylen);

    void configure_network();

    char get_direction(int src, int dst) {
        upDn_ tmp(src, dst);
        return (global_upDn.at(tmp));
    }
    // code end

    // for 2D topology
    int getNumRows() const { return m_num_rows; }
    int getNumCols() { return m_num_cols; }

    //// 3D NoC
    // code begin
    int getNumLayers() const { return m_num_layers; }
    // code end

    // for network
    uint32_t getNiFlitSize() const { return m_ni_flit_size; }
    uint32_t getVCsPerVnet() const { return m_vcs_per_vnet; }
    uint32_t getBuffersPerDataVC() { return m_buffers_per_data_vc; }
    uint32_t getBuffersPerCtrlVC() { return m_buffers_per_ctrl_vc; }
    int getRoutingAlgorithm() const { return m_routing_algorithm; }

    bool isFaultModelEnabled() const { return m_enable_fault_model; }
    FaultModel* fault_model;


    // Internal configuration
    bool isVNetOrdered(int vnet) const { return m_ordered[vnet]; }
    VNET_type
    get_vnet_type(int vc)
    {
        int vnet = vc/getVCsPerVnet();
        return m_vnet_type[vnet];
    }
    int getNumRouters();
    int get_router_id(int ni);


    // Methods used by Topology to setup the network
    void makeExtOutLink(SwitchID src, NodeID dest, BasicLink* link,
                     const NetDest& routing_table_entry);
    void makeExtInLink(NodeID src, SwitchID dest, BasicLink* link,
                    const NetDest& routing_table_entry);
    void makeInternalLink(SwitchID src, SwitchID dest, BasicLink* link,
                          const NetDest& routing_table_entry,
                          PortDirection src_outport_dirn,
                          PortDirection dest_inport_dirn);

    //! Function for performing a functional write. The return value
    //! indicates the number of messages that were written.
    uint32_t functionalWrite(Packet *pkt);

    // Stats
    void collateStats();
    void regStats();
    void print(std::ostream& out) const;

    // increment counters
    void increment_injected_packets(int vnet) { m_packets_injected[vnet]++; }
    void increment_received_packets(int vnet) { m_packets_received[vnet]++; }

    void
    increment_packet_network_latency(Cycles latency, int vnet)
    {
        m_packet_network_latency[vnet] += latency;
    }

    void
    increment_packet_queueing_latency(Cycles latency, int vnet)
    {
        m_packet_queueing_latency[vnet] += latency;
    }

    void increment_injected_flits(int vnet) { m_flits_injected[vnet]++; }
    void increment_received_flits(int vnet) { m_flits_received[vnet]++; }

    void
    increment_flit_network_latency(Cycles latency, int vnet)
    {
        m_flit_network_latency[vnet] += latency;
    }

    void
    increment_flit_queueing_latency(Cycles latency, int vnet)
    {
        m_flit_queueing_latency[vnet] += latency;
    }

    void
    increment_total_hops(int hops)
    {
        m_total_hops += hops;
    }

    //// irregular_Mesh_XY
    //// SWNoC
    // code begin
    std::string conf_file;
    // code end

    //// Updown Routing with Escape_VC
    // code begin
    uint32_t escape_vc;
    // code end

    //// Updown Routing
    // code begin
    struct entry {
    int next_router_id;
    std::string direction_;
    entry() :
        next_router_id(-1),
        direction_("Unknown")
    {}
    entry(int id, std::string dirn) :
            next_router_id(id),
            direction_(dirn)
    {}
    };

    struct upDn_ {
        int src;
        int dst;

        bool operator==(const upDn_ &pair_) const {
            return (src == pair_.src && dst == pair_.dst);
        }

        bool operator<(const upDn_ &pair_)  const {
            return ((src < pair_.src) ||
                    (src == pair_.src && dst < pair_.dst));
        }

        upDn_(int src_, int dst_) :
            src(src_), dst(dst_)
        {}
    };

    std::vector<std::vector<std::vector< entry > > > routingTable;

    std::map<upDn_, char> global_upDn;
    // code end

    //// WestFirst Adaptive: Implement of WestFirst Adaptive Routing
    // code begin
    Router*
    get_RouterInDirn(PortDirection outport_dir, int upstream_id);
    // code end

  protected:
    // Configuration
    int m_num_rows;
    int m_num_cols;
    //// 3D NoC
    // code begin
    int m_num_layers;
    // code end
    uint32_t m_ni_flit_size;
    uint32_t m_vcs_per_vnet;
    uint32_t m_buffers_per_ctrl_vc;
    uint32_t m_buffers_per_data_vc;
    int m_routing_algorithm;
    bool m_enable_fault_model;

    // Statistical variables
    Stats::Vector m_packets_received;
    Stats::Vector m_packets_injected;
    Stats::Vector m_packet_network_latency;
    Stats::Vector m_packet_queueing_latency;

    Stats::Formula m_avg_packet_vnet_latency;
    Stats::Formula m_avg_packet_vqueue_latency;
    Stats::Formula m_avg_packet_network_latency;
    Stats::Formula m_avg_packet_queueing_latency;
    Stats::Formula m_avg_packet_latency;

    Stats::Vector m_flits_received;
    Stats::Vector m_flits_injected;
    Stats::Vector m_flit_network_latency;
    Stats::Vector m_flit_queueing_latency;

    Stats::Formula m_avg_flit_vnet_latency;
    Stats::Formula m_avg_flit_vqueue_latency;
    Stats::Formula m_avg_flit_network_latency;
    Stats::Formula m_avg_flit_queueing_latency;
    Stats::Formula m_avg_flit_latency;

    Stats::Scalar m_total_ext_in_link_utilization;
    Stats::Scalar m_total_ext_out_link_utilization;
    Stats::Scalar m_total_int_link_utilization;
    Stats::Scalar m_average_link_utilization;
    Stats::Vector m_average_vc_load;

    Stats::Scalar  m_total_hops;
    Stats::Formula m_avg_hops;

  private:
    GarnetNetwork(const GarnetNetwork& obj);
    GarnetNetwork& operator=(const GarnetNetwork& obj);

    std::vector<VNET_type > m_vnet_type;
    std::vector<Router *> m_routers;   // All Routers in Network
    std::vector<NetworkLink *> m_networklinks; // All flit links in the network
    std::vector<CreditLink *> m_creditlinks; // All credit links in the network
    std::vector<NetworkInterface *> m_nis;   // All NI's in Network
};

inline std::ostream&
operator<<(std::ostream& out, const GarnetNetwork& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif //__MEM_RUBY_NETWORK_GARNET2_0_GARNETNETWORK_HH__
