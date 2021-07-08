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


#include "mem/ruby/network/garnet2.0/GarnetNetwork.hh"

#include <cassert>
#include <fstream>

#include "base/cast.hh"
#include "mem/ruby/common/NetDest.hh"
#include "mem/ruby/network/MessageBuffer.hh"
#include "mem/ruby/network/garnet2.0/CommonTypes.hh"
#include "mem/ruby/network/garnet2.0/CreditLink.hh"
#include "mem/ruby/network/garnet2.0/GarnetLink.hh"
#include "mem/ruby/network/garnet2.0/NetworkInterface.hh"
#include "mem/ruby/network/garnet2.0/NetworkLink.hh"
#include "mem/ruby/network/garnet2.0/Router.hh"
#include "mem/ruby/system/RubySystem.hh"

using namespace std;

/*
 * GarnetNetwork sets up the routers and links and collects stats.
 * Default parameters (GarnetNetwork.py) can be overwritten from command line
 * (see configs/network/Network.py)
 */

GarnetNetwork::GarnetNetwork(const Params *p)
    : Network(p)
{
    m_num_rows = p->num_rows;
    //// 3D NoC
    // code begin
    m_num_layers = p->num_layers;
    // code end
    m_ni_flit_size = p->ni_flit_size;
    m_vcs_per_vnet = p->vcs_per_vnet;
    m_buffers_per_data_vc = p->buffers_per_data_vc;
    m_buffers_per_ctrl_vc = p->buffers_per_ctrl_vc;
    m_routing_algorithm = p->routing_algorithm;
    //// irregular_Mesh_XY
    //// SWNoC
    // code begin
    if ((m_routing_algorithm ==2) or (m_routing_algorithm == 3)) {
        conf_file = p->conf_file;
        cout << "Configuration file to read from: "\
            << conf_file << endl;
    }
    // code end
    m_enable_fault_model = p->enable_fault_model;
    if (m_enable_fault_model)
        fault_model = p->fault_model;

    m_vnet_type.resize(m_virtual_networks);

    for (int i = 0 ; i < m_virtual_networks ; i++) {
        if (m_vnet_type_names[i] == "response")
            m_vnet_type[i] = DATA_VNET_; // carries data (and ctrl) packets
        else
            m_vnet_type[i] = CTRL_VNET_; // carries only ctrl packets
    }

    // record the routers
    for (vector<BasicRouter*>::const_iterator i =  p->routers.begin();
         i != p->routers.end(); ++i) {
        Router* router = safe_cast<Router*>(*i);
        m_routers.push_back(router);

        // initialize the router's network pointers
        router->init_net_ptr(this);
    }

    // record the network interfaces
    for (vector<ClockedObject*>::const_iterator i = p->netifs.begin();
         i != p->netifs.end(); ++i) {
        NetworkInterface *ni = safe_cast<NetworkInterface *>(*i);
        m_nis.push_back(ni);
        ni->init_net_ptr(this);
    }

    //// Updown Routing
    // code begin
    // resize the routingTable
    if ((m_routing_algorithm ==2) or (m_routing_algorithm == 3)) {
        routingTable.resize(m_routers.size());
        configure_network();
    }
    // code end
}

//// Updown Routing
//// Updown Routing+
// code begin

// populate_routingTable() called by configure_network(),
// input path_, get entry_, and push entry_ to the end of routingTable.
void
GarnetNetwork::populate_routingTable\
(std::vector< int >& path_, int ylen) {
    // original 'src' and 'dst' pair for this path
    // get the reference of the last, first unit in vector
    int dst = path_.back();
    int src = path_.front();
    entry entry_;
    for (int curr_ = 0, nxt_ = curr_ + 1;
        curr_ < path_.size() && nxt_ < path_.size();
        curr_++, nxt_++) {
        string dirn_;
        if (path_[nxt_] == (path_[curr_] - 1)) {
            // West
            // set outport direction according to path_[nxt_]
            entry_ = {path_[nxt_], "West"};
        }
        else if (path_[nxt_] == (path_[curr_] + 1)) {
            // East
            entry_ = {path_[nxt_], "East"};
        }
        else if (path_[nxt_] == (path_[curr_] + ylen)) {
            // North
            entry_ = {path_[nxt_], "North"};
        }
        else if (path_[nxt_] == (path_[curr_] - ylen)) {
            // South
            entry_ = {path_[nxt_], "South"};
        }
        else if (path_[nxt_] == path_[curr_]) {
            // skip do nothing...
        }
        else {
            //// Updown Routing
            entry_ = {path_[nxt_], "X"};
            // cout << " this is not possible" << endl;
            // assert(0);
        }

        // push entry_ to the end of routingTable
        // routingTable[src][dst] including lots of step from src to dst,
        // each step/entry_ contains next node number and direction.
        // entry_ = {path_[nxt_], "West"}
        routingTable[src][dst].push_back(entry_);
    }

    return;
}

// code end

//// Updown Routing
// code begin

void
GarnetNetwork::configure_network()
{
    // open the file pointed by conf_file for read
    ifstream inFile(conf_file);

    //// Updown Routing+ for 3D
    // code begin
    string word;
    // read xlen, ylen
    inFile >> word;
    int xlen = stoi(word);
    inFile >> word;
    int ylen = stoi(word);

    assert(m_num_rows == xlen);
    // assert(m_routers.size() == xlen*ylen);
    // assert(m_num_layers == zlen);
    // assert(m_routers.size() == xlen*ylen*zlen);

    if (m_num_layers == 1) {
        // resize the table
        routingTable.resize(xlen*ylen);
        for (int i = 0; i < xlen*ylen; ++i) {
            routingTable[i].resize(xlen*ylen);
        }
    }
    else {
        inFile >> word;
        int zlen = stoi(word);

        routingTable.resize(xlen*ylen*zlen);
        for (int i = 0; i < xlen*ylen*zlen; ++i) {
            routingTable[i].resize(xlen*ylen*zlen);
        }
    }
    // code end

    bool top_ = false;
    bool spinRing = false;
    bool up_dn = false;
    bool up_dn_path = false;
    bool path_start = false;
    bool path_end = false;
    std::vector<int> tmp_path;

    while (!(inFile.eof())) {
        // eof, end of file
        inFile >> word;

        if ((word.find("Topology") != -1)) {
            // find "Topology" in word
            top_ = true;
            spinRing = false;
            up_dn = false;
            up_dn_path = false;
        }
        if ((word.find("SpinRing") != -1)) {
            top_ = false;
            spinRing = true;
            up_dn = false;
            up_dn_path = false;
        }
        if ((word.find("UP/DOWN") != -1)) {
            top_ = false;
            spinRing = false;
            up_dn = true;
            up_dn_path = false;
        }
        if ((word.find("UP/DOWN_PATHS") != -1)) {
            top_ = false;
            spinRing = false;
            up_dn = false;
            up_dn_path = true;
        }

        //// skip to UP/DOWN_PATHS,
        // and read up_dn_path to temp_path
        if ( up_dn_path ) {

            if (inFile.peek() == EOF) {
                // if end of file, then path_end
                path_start = false;
                path_end = true;
            }
            if ((!path_start) &&
               (path_end) &&
               (tmp_path.size()>0)) {
                // if path_end but temp_path exit
                populate_routingTable(tmp_path, ylen);
                // change dst node to outport dirn, and push into routingTable
                tmp_path.clear();
            }
            if (word =="[") {
                // path_end
                path_start = false;
                path_end = true;
            }
            if (path_start &&
                !path_end) {
                // cout << stoi(word);
                tmp_path.push_back(stoi(word));
            }
            if (word == ":") {
                // path_start
                path_start = true;
                path_end = false;
            }
            assert(!top_);
            assert(!up_dn);
            assert(!spinRing);
        }

    }

    // from the beginning of the file
    inFile.clear();
    inFile.seekg(0, std::ios::beg);

    string line;
    int src = 0;
    int dst = 0;
    bool start = false;
    bool stop = false;

    //// skip to UP/DOWN
    // read up_dn to global_upDn
    while (std::getline(inFile, line)) {
        if ( start &&
            line.empty()) {
            // UP/DOWN end
            start = false;
            stop = true;
        }
        if ( start &&
            !stop ) {
            // cout << line << endl;
            // break this line into deliminter,
            // for example, deliminter in "a b" is " "
            for (auto x : line) {
                if (x == 'u') {
                    // if links is Up
                    // cout << x << endl;
                    pair<upDn_, char> p((upDn_{src,dst}),x);
                    // record src, dst, and x(u or d)
                    global_upDn.insert(p);
                    // insert into global_upDn
                }
                if (x == 'd') {
                    // cout << x << endl;
                    pair<upDn_, char> p((upDn_{src,dst}),x);
                    global_upDn.insert(p);
                }
                if (x == ' ') {
                    // do not increment dst here here
                } else {
                    dst += 1;
                }
            }
            // first from src 0 to dst 0,1,2,3,..., then,1,2,...
            dst = 0;	// reset
            src += 1;	// increment
            // cout.flush();
        }
        if ((line.find("UP/DOWN") != -1)) {
            // find "UP/DOWN"
            // cout << line << endl;
            // cout.flush();
            start = true;
        }
    }

    // close the file.
    inFile.close();
    // cout global--map
    /*for (auto& t : global_upDn)
        std::cout << t.first.src << " "
                  << t.first.dst << " "
                  << t.second << " "
                  << "\n";*/
    // assert(0);
}

// code end

void
GarnetNetwork::init()
{
    Network::init();

    for (int i=0; i < m_nodes; i++) {
        m_nis[i]->addNode(m_toNetQueues[i], m_fromNetQueues[i]);
    }

    // The topology pointer should have already been initialized in the
    // parent network constructor
    assert(m_topology_ptr != NULL);
    m_topology_ptr->createLinks(this);

    // Initialize topology specific parameters
    if (getNumRows() > 0) {
        // Only for Mesh topology
        // m_num_rows and m_num_cols are only used for
        // implementing XY or custom routing in RoutingUnit.cc
        m_num_rows = getNumRows();

        //// 3D NoC
        // code begin
        m_num_layers = getNumLayers();
        m_num_cols = (m_routers.size() / m_num_layers) / m_num_rows;
        assert(m_num_rows * m_num_cols * m_num_layers == m_routers.size());
        // code end

    } else {
        m_num_rows = -1;
        m_num_cols = -1;
        //// 3D NoC
        // code begin
        m_num_layers = -1;
        // code end
    }

    // FaultModel: declare each router to the fault model
    if (isFaultModelEnabled()) {
        for (vector<Router*>::const_iterator i= m_routers.begin();
             i != m_routers.end(); ++i) {
            Router* router = safe_cast<Router*>(*i);
            int router_id M5_VAR_USED =
                fault_model->declare_router(router->get_num_inports(),
                                            router->get_num_outports(),
                                            router->get_vc_per_vnet(),
                                            getBuffersPerDataVC(),
                                            getBuffersPerCtrlVC());
            assert(router_id == router->get_id());
            router->printAggregateFaultProbability(cout);
            router->printFaultVector(cout);
        }
    }
}

/*
 * This function creates a link from the Network Interface (NI)
 * into the Network.
 * It creates a Network Link from the NI to a Router and a Credit Link from
 * the Router to the NI
*/

void
GarnetNetwork::makeExtInLink(NodeID src, SwitchID dest, BasicLink* link,
                            const NetDest& routing_table_entry)
{
    assert(src < m_nodes);

    GarnetExtLink* garnet_link = safe_cast<GarnetExtLink*>(link);

    // GarnetExtLink is bi-directional
    NetworkLink* net_link = garnet_link->m_network_links[LinkDirection_In];
    net_link->setType(EXT_IN_);
    CreditLink* credit_link = garnet_link->m_credit_links[LinkDirection_In];

    m_networklinks.push_back(net_link);
    m_creditlinks.push_back(credit_link);

    PortDirection dst_inport_dirn = "Local";
    m_routers[dest]->addInPort(dst_inport_dirn, net_link, credit_link);
    m_nis[src]->addOutPort(net_link, credit_link, dest);
}

/*
 * This function creates a link from the Network to a NI.
 * It creates a Network Link from a Router to the NI and
 * a Credit Link from NI to the Router
*/

void
GarnetNetwork::makeExtOutLink(SwitchID src, NodeID dest, BasicLink* link,
                             const NetDest& routing_table_entry)
{
    assert(dest < m_nodes);
    assert(src < m_routers.size());
    assert(m_routers[src] != NULL);

    GarnetExtLink* garnet_link = safe_cast<GarnetExtLink*>(link);

    // GarnetExtLink is bi-directional
    NetworkLink* net_link = garnet_link->m_network_links[LinkDirection_Out];
    net_link->setType(EXT_OUT_);
    CreditLink* credit_link = garnet_link->m_credit_links[LinkDirection_Out];

    m_networklinks.push_back(net_link);
    m_creditlinks.push_back(credit_link);

    PortDirection src_outport_dirn = "Local";

    //// Updown Routing+: Implement of Updown Routing
    // code begin
    m_routers[src]->addOutPort(dest, src_outport_dirn, net_link,
                               routing_table_entry,
                               link->m_weight, credit_link);
    // code end

    m_nis[dest]->addInPort(net_link, credit_link);
}

/*
 * This function creates an internal network link between two routers.
 * It adds both the network link and an opposite credit link.
*/

void
GarnetNetwork::makeInternalLink(SwitchID src, SwitchID dest, BasicLink* link,
                                const NetDest& routing_table_entry,
                                PortDirection src_outport_dirn,
                                PortDirection dst_inport_dirn)
{
    GarnetIntLink* garnet_link = safe_cast<GarnetIntLink*>(link);

    // GarnetIntLink is unidirectional
    NetworkLink* net_link = garnet_link->m_network_link;
    net_link->setType(INT_);
    CreditLink* credit_link = garnet_link->m_credit_link;

    m_networklinks.push_back(net_link);
    m_creditlinks.push_back(credit_link);

    m_routers[dest]->addInPort(dst_inport_dirn, net_link, credit_link);

    //// Updown Routing+: Implement of Updown Routing
    // code begin
    // cout << endl;
    // cout << "src:" << src << endl;
    m_routers[src]->addOutPort(dest, src_outport_dirn, net_link,
                               routing_table_entry,
                               link->m_weight, credit_link);
    // code end

}

// Total routers in the network
int
GarnetNetwork::getNumRouters()
{
    return m_routers.size();
}

// Get ID of router connected to a NI.
int
GarnetNetwork::get_router_id(int ni)
{
    return m_nis[ni]->get_router_id();
}

void
GarnetNetwork::regStats()
{
    Network::regStats();

    // Packets
    m_packets_received
        .init(m_virtual_networks)
        .name(name() + ".packets_received")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;

    m_packets_injected
        .init(m_virtual_networks)
        .name(name() + ".packets_injected")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;

    m_packet_network_latency
        .init(m_virtual_networks)
        .name(name() + ".packet_network_latency")
        .flags(Stats::oneline)
        ;

    m_packet_queueing_latency
        .init(m_virtual_networks)
        .name(name() + ".packet_queueing_latency")
        .flags(Stats::oneline)
        ;

    for (int i = 0; i < m_virtual_networks; i++) {
        m_packets_received.subname(i, csprintf("vnet-%i", i));
        m_packets_injected.subname(i, csprintf("vnet-%i", i));
        m_packet_network_latency.subname(i, csprintf("vnet-%i", i));
        m_packet_queueing_latency.subname(i, csprintf("vnet-%i", i));
    }

    m_avg_packet_vnet_latency
        .name(name() + ".average_packet_vnet_latency")
        .flags(Stats::oneline);
    m_avg_packet_vnet_latency =
        m_packet_network_latency / m_packets_received;

    m_avg_packet_vqueue_latency
        .name(name() + ".average_packet_vqueue_latency")
        .flags(Stats::oneline);
    m_avg_packet_vqueue_latency =
        m_packet_queueing_latency / m_packets_received;

    m_avg_packet_network_latency
        .name(name() + ".average_packet_network_latency");
    m_avg_packet_network_latency =
        sum(m_packet_network_latency) / sum(m_packets_received);

    m_avg_packet_queueing_latency
        .name(name() + ".average_packet_queueing_latency");
    m_avg_packet_queueing_latency
        = sum(m_packet_queueing_latency) / sum(m_packets_received);

    m_avg_packet_latency
        .name(name() + ".average_packet_latency");
    m_avg_packet_latency
        = m_avg_packet_network_latency + m_avg_packet_queueing_latency;

    // Flits
    m_flits_received
        .init(m_virtual_networks)
        .name(name() + ".flits_received")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;

    m_flits_injected
        .init(m_virtual_networks)
        .name(name() + ".flits_injected")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;

    m_flit_network_latency
        .init(m_virtual_networks)
        .name(name() + ".flit_network_latency")
        .flags(Stats::oneline)
        ;

    m_flit_queueing_latency
        .init(m_virtual_networks)
        .name(name() + ".flit_queueing_latency")
        .flags(Stats::oneline)
        ;

    for (int i = 0; i < m_virtual_networks; i++) {
        m_flits_received.subname(i, csprintf("vnet-%i", i));
        m_flits_injected.subname(i, csprintf("vnet-%i", i));
        m_flit_network_latency.subname(i, csprintf("vnet-%i", i));
        m_flit_queueing_latency.subname(i, csprintf("vnet-%i", i));
    }

    m_avg_flit_vnet_latency
        .name(name() + ".average_flit_vnet_latency")
        .flags(Stats::oneline);
    m_avg_flit_vnet_latency = m_flit_network_latency / m_flits_received;

    m_avg_flit_vqueue_latency
        .name(name() + ".average_flit_vqueue_latency")
        .flags(Stats::oneline);
    m_avg_flit_vqueue_latency =
        m_flit_queueing_latency / m_flits_received;

    m_avg_flit_network_latency
        .name(name() + ".average_flit_network_latency");
    m_avg_flit_network_latency =
        sum(m_flit_network_latency) / sum(m_flits_received);

    m_avg_flit_queueing_latency
        .name(name() + ".average_flit_queueing_latency");
    m_avg_flit_queueing_latency =
        sum(m_flit_queueing_latency) / sum(m_flits_received);

    m_avg_flit_latency
        .name(name() + ".average_flit_latency");
    m_avg_flit_latency =
        m_avg_flit_network_latency + m_avg_flit_queueing_latency;


    // Hops
    m_avg_hops.name(name() + ".average_hops");
    m_avg_hops = m_total_hops / sum(m_flits_received);

    // Links
    m_total_ext_in_link_utilization
        .name(name() + ".ext_in_link_utilization");
    m_total_ext_out_link_utilization
        .name(name() + ".ext_out_link_utilization");
    m_total_int_link_utilization
        .name(name() + ".int_link_utilization");
    m_average_link_utilization
        .name(name() + ".avg_link_utilization");

    m_average_vc_load
        .init(m_virtual_networks * m_vcs_per_vnet)
        .name(name() + ".avg_vc_load")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;
}

void
GarnetNetwork::collateStats()
{
    RubySystem *rs = params()->ruby_system;
    double time_delta = double(curCycle() - rs->getStartCycle());

    for (int i = 0; i < m_networklinks.size(); i++) {
        link_type type = m_networklinks[i]->getType();
        int activity = m_networklinks[i]->getLinkUtilization();

        if (type == EXT_IN_)
            m_total_ext_in_link_utilization += activity;
        else if (type == EXT_OUT_)
            m_total_ext_out_link_utilization += activity;
        else if (type == INT_)
            m_total_int_link_utilization += activity;

        m_average_link_utilization +=
            (double(activity) / time_delta);

        vector<unsigned int> vc_load = m_networklinks[i]->getVcLoad();
        for (int j = 0; j < vc_load.size(); j++) {
            m_average_vc_load[j] += ((double)vc_load[j] / time_delta);
        }
    }

    // Ask the routers to collate their statistics
    for (int i = 0; i < m_routers.size(); i++) {
        m_routers[i]->collateStats();
    }
}

void
GarnetNetwork::print(ostream& out) const
{
    out << "[GarnetNetwork]";
}

GarnetNetwork *
GarnetNetworkParams::create()
{
    return new GarnetNetwork(this);
}

uint32_t
GarnetNetwork::functionalWrite(Packet *pkt)
{
    uint32_t num_functional_writes = 0;

    for (unsigned int i = 0; i < m_routers.size(); i++) {
        num_functional_writes += m_routers[i]->functionalWrite(pkt);
    }

    for (unsigned int i = 0; i < m_nis.size(); ++i) {
        num_functional_writes += m_nis[i]->functionalWrite(pkt);
    }

    for (unsigned int i = 0; i < m_networklinks.size(); ++i) {
        num_functional_writes += m_networklinks[i]->functionalWrite(pkt);
    }

    return num_functional_writes;
}
