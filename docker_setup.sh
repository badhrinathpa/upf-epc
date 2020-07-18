#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright(c) 2019 Intel Corporation

set -e
# TCP port of bess/web monitor
gui_port=8000
bessd_port=10514

# Driver options. Choose any one of the three
#
# "dpdk" set as default
# "af_xdp" uses AF_XDP sockets via DPDK's vdev for pkt I/O. This version is non-zc version. ZC version still needs to be evaluated.
# "af_packet" uses AF_PACKET sockets via DPDK's vdev for pkt I/O.
# "sim" uses Source() modules to simulate traffic generation
mode="af_xdp"
#mode="af_xdp"
#mode="af_packet"
#mode="sim"

# Gateway interface(s)
#
# In the order of ("s1u" "sgi")
ifaces=("ens803f2" "ens803f3")

# Static IP addresses of gateway interface(s) in cidr format
#
# In the order of (s1u sgi)
ipaddrs=(198.18.0.1/30 198.19.0.1/30)

# MAC addresses of gateway interface(s)
#
# In the order of (s1u sgi)
macaddrs=(68:05:ca:33:2e:20 68:05:ca:33:2e:21)

# Static IP addresses of the neighbors of gateway interface(s)
#
# In the order of (n-s1u n-sgi)
nhipaddrs=(198.18.0.2 198.19.0.2)

# Static MAC addresses of the neighbors of gateway interface(s)
#
# In the order of (n-s1u n-sgi)
nhmacaddrs=(68:05:ca:31:fa:7a 68:05:ca:31:fa:7b)

# IPv4 route table entries in cidr format per port
#
# In the order of ("{r-s1u}" "{r-sgi}")
routes=("11.1.1.128/27 11.1.1.160/27 11.1.1.192/27 11.1.1.224/27" "13.1.1.128/27 13.1.1.160/27 13.1.1.192/27 13.1.1.224/27")

num_ifaces=${#ifaces[@]}
num_ipaddrs=${#ipaddrs[@]}

# Set up static route and neighbor table entries of the SPGW
function setup_trafficgen_routes() {
	for ((i = 0; i < num_ipaddrs; i++)); do
		sudo ip netns exec pause ip neighbor add "${nhipaddrs[$i]}" lladdr "${nhmacaddrs[$i]}" dev "${ifaces[$i % num_ifaces]}"
		routelist=${routes[$i]}
		for route in $routelist; do
			sudo ip netns exec pause ip route add "$route" via "${nhipaddrs[$i]}"
		done
	done
}

# Assign IP address(es) of gateway interface(s) within the network namespace
function setup_addrs() {
	for ((i = 0; i < num_ipaddrs; i++)); do
		sudo ip netns exec pause ip addr add "${ipaddrs[$i]}" dev "${ifaces[$i % $num_ifaces]}"
	done
}

# Set up mirror links to communicate with the kernel
#
# These vdev interfaces are used for ARP + ICMP updates.
# ARP/ICMP requests are sent via the vdev interface to the kernel.
# ARP/ICMP responses are captured and relayed out of the dpdk ports.
function setup_mirror_links() {
	for ((i = 0; i < num_ifaces; i++)); do
		sudo ip netns exec pause ip link add "${ifaces[$i]}" type veth peer name "${ifaces[$i]}"-vdev
		sudo ip netns exec pause ip link set "${ifaces[$i]}" up
		sudo ip netns exec pause ip link set "${ifaces[$i]}-vdev" up
		sudo ip netns exec pause ip link set dev "${ifaces[$i]}" address "${macaddrs[$i]}"
	done
	setup_addrs
}

# Set up interfaces in the network namespace. For non-"dpdk" mode(s)
function move_ifaces() {
	for ((i = 0; i < num_ifaces; i++)); do
		sudo ip link set "${ifaces[$i]}" netns pause up
		sudo ip netns exec pause ip link set "${ifaces[$i]}" promisc off
		sudo ip netns exec pause ip link set "${ifaces[$i]}" xdp off
		if [ "$mode" == 'af_xdp' ]; then
			sudo ip netns exec pause ethtool --features "${ifaces[$i]}" ntuple off
			sudo ip netns exec pause ethtool --features "${ifaces[$i]}" ntuple on
			sudo ip netns exec pause ethtool -N "${ifaces[$i]}" flow-type udp4 action 0
			sudo ip netns exec pause ethtool -N "${ifaces[$i]}" flow-type tcp4 action 0
			sudo ip netns exec pause ethtool -u "${ifaces[$i]}"
		fi
	done
	setup_addrs
}

# Stop previous instances of bess* before restarting
docker stop pause bess bess-routectl bess-web bess-cpiface bess-pfcpiface || true
docker rm -f pause bess bess-routectl bess-web bess-cpiface bess-pfcpiface || true
sudo rm -rf /var/run/netns/pause

# Build
make docker-build

# Run bess-pfcpiface depending on mode type
docker run --name bess-pfcpiface -td --restart on-failure \
	-v "$PWD/conf/upf.json":/conf/upf.json \
	upf-epc-pfcpiface:"$(<VERSION)" \
	-config /conf/upf.json

