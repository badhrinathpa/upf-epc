// SPDX-License-Identifier: Apache-2.0
// Copyright(c) 2020 Intel Corporation

package main

import (
	"log"
	"net"

	"github.com/prometheus/client_golang/prometheus"
	"github.com/wmnsk/go-pfcp/ie"
)

type simFastPath struct {
}

func (s *simFastPath) setInfo(udpConn *net.UDPConn, udpAddr net.Addr, pconn *PFCPConn) {
}

func (s *simFastPath) isConnected(accessIP *net.IP) bool {
	return true
}

func (s *simFastPath) sendMsgToUPF(method string, pdrs []pdr, fars []far) uint8 {
	return ie.CauseRequestAccepted
}

func (s *simFastPath) sendDeleteAllSessionsMsgtoUPF() {
}

func (s *simFastPath) exit() {
	log.Println("Exit function SimFastPath")
}

func (s *simFastPath) portStats(uc *upfCollector, ch chan<- prometheus.Metric) {
}

func (s *simFastPath) summaryLatencyJitter(uc *upfCollector, ch chan<- prometheus.Metric) {
}

func (s *simFastPath) setUpfInfo(u *upf, conf *Conf) {
	log.Println("setUpfInfo simFastPath")
	u.accessIP = net.ParseIP("0.0.0.0")
	u.coreIP = net.ParseIP("0.0.0.0")
}

func (s *simFastPath) sim(u *upf, method string) {
}
