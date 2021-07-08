# Copyright (c) 2010 Advanced Micro Devices, Inc.
#               2016 Georgia Institute of Technology
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function
from __future__ import absolute_import

from m5.params import *
from m5.objects import *

from common import FileSystemConfig

from topologies.BaseTopology import SimpleTopology

#### SWNoC
## code begin
import math
## code end

# Creates a generic Mesh assuming an equal number of cache
# and directory controllers.
# XY routing is enforced (using link weights)
# to guarantee deadlock freedom.

class SWNoC_3D(SimpleTopology):
    description='SWNoC_3D'

    def __init__(self, controllers):
        self.nodes = controllers

    # Makes a generic mesh
    # assuming an equal number of cache and directory cntrls

    def makeTopology(self, options, network, IntLink, ExtLink, Router):
        ################ initialize ################
        nodes = self.nodes

        num_routers = options.num_cpus
        num_rows = options.mesh_rows
        #### 3D NoC: Add layers parameter
        ## code begin
        num_layers = options.mesh_layers
        num_planars = int(num_routers / num_layers)
        num_columns = int(num_planars / num_rows)
        ## code end

        ######### latency define and parameter check #########
        # default values for link latency and router latency.
        # Can be over-ridden on a per link/router basis
        link_latency = options.link_latency # used by simple and garnet
        router_latency = options.router_latency # only used by garnet

        # There must be an evenly divisible number of cntrls to routers
        # Also, obviously the number or rows must be <= the number of routers
        cntrls_per_router, remainder = divmod(len(nodes), num_routers)
        assert(num_rows > 0 and num_rows <= num_routers)
        #### 3D NoC: check num_layers
        ## code begin
        assert(num_layers > 0 and num_layers <= num_routers)
        assert(num_columns * num_rows * num_layers == num_routers)
        ## code end

        ################ router setting ################
        # Create the routers in the mesh
        routers = [Router(router_id=i, latency = router_latency) \
            for i in range(num_routers)]
        network.routers = routers

        ################ nodes setting ################
        # link counter to set unique link ids
        link_count = 0

        # Add all but the remainder nodes to the list of nodes to be uniformly
        # distributed across the network.
        network_nodes = []
        remainder_nodes = []
        for node_index in range(len(nodes)):
            if node_index < (len(nodes) - remainder):
                network_nodes.append(nodes[node_index])
            else:
                remainder_nodes.append(nodes[node_index])

        ################ ext_links ################
        # Connect each node to the appropriate router
        ext_links = []
        for (i, n) in enumerate(network_nodes):
            cntrl_level, router_id = divmod(i, num_routers)
            assert(cntrl_level < cntrls_per_router)
            ext_links.append(ExtLink(link_id=link_count, ext_node=n,
                                    int_node=routers[router_id],
                                    latency = link_latency))
            link_count += 1

        # Connect the remainding nodes to router 0.  These should only be
        # DMA nodes.
        for (i, node) in enumerate(remainder_nodes):
            assert(node.type == 'DMA_Controller')
            assert(i < remainder)
            ext_links.append(ExtLink(link_id=link_count, ext_node=node,
                                    int_node=routers[0],
                                    latency = link_latency))
            link_count += 1

        network.ext_links = ext_links

        #### 3D SWNoC
        ## code begin
        input_ = options.conf_file
        print("Configs file: ", options.conf_file)

        with open (input_, "r") as f:
            # "r": default mode, read only

            # read network size
            data_ = f.readline()
            data = data_.split(" ")
            rows_ = int(data[0])
            cols_ = int(data[1])
            lays_ = int(data[1])
            print ("Network rows: %d Network cols: %d Network lays: %d" %\
            (rows_, cols_, lays_))

            # skip a line 'not necessary'
            next(f)

            # Now create a list and read all the file content into that list,
            # later refer to that list to make connections in Mesh.

            # Make an empty list,
            # list size is (row*col*lay)x(row*col*lay)
            topology = [[0 for x in range(cols_*rows_*lays_)] \
            for y in range(rows_*cols_*lays_)]

            # Read the file content into that list
            x, y = 0, 0;
            for line in f:
                for word in line.split():
                    topology[x][y] = int(word)
                    y = y + 1
                x = x + 1  # increment row

                if ((x == rows_ * cols_ * lays_) and
                    (y == rows_ * cols_ * lays_)):
                    break

                y = 0 # reset col index on each row completetion

        # num_rows must match rows_
        # num_columns must match columns_
        # num_layers must match layers_
        assert (num_rows == rows_),\
        "Both topology-row: %d and commandline-row: %d must match" \
        % (num_rows, rows_)
        assert (num_columns == cols_),\
        "Both topology-col: %d and commandline-col: %d must match" \
        % (num_columns, cols_)
        assert (num_columns == cols_),\
        "Both topology-lay: %d and commandline-lay: %d must match" \
        % (num_layers, lays_)
        ## code end

        ################ int_links ################
        # Create the mesh links.
        int_links = []

        # planar_links
        #### 3D SWNoC: planar_links
        ## code begin
        for lay in range(num_layers):
        # num_planars: nodes per planar
            for src in range(0, num_planars):
                for dst in range(src+1, num_planars):
                    if (topology[lay*num_planars +src]\
                    [lay*num_planars +dst] == 1):
                        # x,y of source node
                        ysrc = int((lay*num_planars + src) / num_columns)
                        xsrc = (lay*num_planars + src) % num_columns
                        # x,y of destination node
                        ydst = int((lay*num_planars + dst) / num_columns)
                        xdst = (lay*num_planars + dst) % num_columns

                        # East to West / West to East links
                        if ((ysrc == ydst) and (xsrc != xdst)):
                            # East output to West input links
                            # (latency = link length)
                            link_length = xdst-xsrc
                            east_out = lay*num_planars + src
                            west_in = lay*num_planars + dst
                            int_links.append(IntLink(link_id=link_count,
                                                     src_node=\
                                                     routers[east_out],
                                                     dst_node=\
                                                     routers[west_in],
                                                     src_outport="East",
                                                     dst_inport="West",
                                                     latency = link_length,
                                                     weight=1))
                            link_count += 1

                            # West output to East input links
                            east_in = lay*num_planars + src
                            west_out = lay*num_planars + dst
                            int_links.append(IntLink(link_id=link_count,
                                                     src_node=\
                                                     routers[west_out],
                                                     dst_node=\
                                                     routers[east_in],
                                                     src_outport="West",
                                                     dst_inport="East",
                                                     latency = link_length,
                                                     weight=1))
                            link_count += 1

                        # North to South / South to North links
                        if ((xsrc == xdst) and (ysrc != ydst)):
                            # North output to South input links
                            # (latency = link length)
                            link_length = ydst-ysrc
                            north_out = lay*num_planars + src
                            south_in = lay*num_planars + dst
                            int_links.append(IntLink(link_id=link_count,
                                                     src_node=\
                                                     routers[north_out],
                                                     dst_node=\
                                                     routers[south_in],
                                                     src_outport="North",
                                                     dst_inport="South",
                                                     latency = link_length,
                                                     weight=1))
                            link_count += 1

                            # South output to North input links
                            north_in = lay*num_planars + src
                            south_out = lay*num_planars + dst
                            int_links.append(IntLink(link_id=link_count,
                                                     src_node=\
                                                     routers[south_out],
                                                     dst_node=\
                                                     routers[north_in],
                                                     src_outport="South",
                                                     dst_inport="North",
                                                     latency = link_length,
                                                     weight=1))
                            link_count += 1


                        # NorthEast to SouthWest / SouthWest to NorthEast links
                        if ((xsrc < xdst) and (ysrc < ydst)):
                            # NorthEast output to SouthWest input links
                            # (latency = link length)
                            link_length = int(math.ceil((((xdst-xsrc)**2)+\
                            ((ydst-ysrc)**2))**0.5))
                            northeast_out = lay*num_planars + src
                            southwest_in = lay*num_planars + dst
                            int_links.append(IntLink(link_id=link_count,
                                                     src_node=\
                                                     routers[northeast_out],
                                                     dst_node=\
                                                     routers[southwest_in],
                                                     src_outport="NorthEast",
                                                     dst_inport="SouthWest",
                                                     latency = link_length,
                                                     weight=1))
                            link_count += 1

                            # SouthWest output to NorthEast input links
                            northeast_in = lay*num_planars + src
                            southwest_out = lay*num_planars + dst
                            int_links.append(IntLink(link_id=link_count,
                                                     src_node=\
                                                     routers[southwest_out],
                                                     dst_node=\
                                                     routers[northeast_in],
                                                     src_outport="SouthWest",
                                                     dst_inport="NorthEast",
                                                     latency = link_length,
                                                     weight=1))
                            link_count += 1


                        # NorthWest to SouthEast / SouthEast to NorthWest links
                        if ((xsrc > xdst) and (ysrc < ydst)):
                            # NorthWest output to SouthEast input links
                            # (latency = link length)
                            link_length = int(math.ceil((((xsrc-xdst)**2)+\
                            ((ydst-ysrc)**2))**0.5))
                            northwest_out = lay*num_planars + src
                            southeast_in = lay*num_planars + dst
                            int_links.append(IntLink(link_id=link_count,
                                                     src_node=\
                                                     routers[northwest_out],
                                                     dst_node=\
                                                     routers[southeast_in],
                                                     src_outport="NorthWest",
                                                     dst_inport="SouthEast",
                                                     latency = link_length,
                                                     weight=1))
                            link_count += 1

                            # SouthWest output to NorthEast input links
                            northwest_in = lay*num_planars + src
                            southeast_out = lay*num_planars + dst
                            int_links.append(IntLink(link_id=link_count,
                                                     src_node=\
                                                     routers[southeast_out],
                                                     dst_node=\
                                                     routers[northwest_in],
                                                     src_outport="SouthEast",
                                                     dst_inport="NorthWest",
                                                     latency = link_length,
                                                     weight=1))
                            link_count += 1
        ## code end

        # vertical_links
        #### 3D SWNoC: vertical_links
        ## code begin
        # Up to Down / Down to Up links
        for lay in range(num_layers-1):
            for planar in range(num_planars):
            # num_planars: nodes per planar
                if (topology[planar + (lay*num_planars)]\
                [planar + ((lay+1)*num_planars)] == 1):
                    # Up to Down links (latency = link length)
                    link_length = 1
                    up_out = planar + (lay * num_planars)
                    down_in = planar + ((lay+1) * num_planars)
                    int_links.append(IntLink(link_id=link_count,
                                             src_node=routers[up_out],
                                             dst_node=routers[down_in],
                                             src_outport="Up",
                                             dst_inport="Down",
                                             latency = link_length,
                                             weight=1))
                    link_count += 1

                    # Down to Up links (latency = link length)
                    down_out = planar + ((lay+1) * num_planars)
                    up_in = planar + ((lay) * num_planars)
                    int_links.append(IntLink(link_id=link_count,
                                             src_node=routers[down_out],
                                             dst_node=routers[up_in],
                                             src_outport="Down",
                                             dst_inport="Up",
                                             latency = link_length,
                                             weight=1))
                    link_count += 1

        ## code end

        network.int_links = int_links

    # Register nodes with filesystem
    def registerTopology(self, options):
        for i in range(options.num_cpus):
            FileSystemConfig.register_node([i],
                    MemorySize(options.mem_size) / options.num_cpus, i)
