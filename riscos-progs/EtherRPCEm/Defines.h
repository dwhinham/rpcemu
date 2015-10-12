/* h.EYEquates */
//  This program is free software; you can redistribute it and/or modify it 
//  under the terms of version 2 of the GNU General Public License as 
//  published by the Free Software Foundation;
//
//  This program is distributed in the hope that it will be useful, but WITHOUT 
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for 
//  more details.
//
//  You should have received a copy of the GNU General Public License along with
//  this program; if not, write to the Free Software Foundation, Inc., 59 
//  Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//  
//  The full GNU General Public License is included in this distribution in the
//  file called LICENSE.
//
//  The code in this file is (C) 2003 J Ballance as far as
//  the above GPL permits


// service calls of interest
#define Service_EnumerateNetworkDrivers  0x9b
#define Service_DCIDriverStatus          0x9d
#define Service_DCIFrameTypeFree         0x9e
#define Service_DCIProtocolStatus        0x9f
#define Service_MbufManagerStatus        0xa2

// OS_Module reason codes
#define RMAClaim     6
#define RMAFree      7

// SWI names--numbers

#define SWIDCIVersion     0
#define SWIInquire        1
#define SWIGetNetworkMTU  2
#define SWISetNetworkMTU  3
#define SWITransmit       4
#define SWIFilter         5
#define SWIStats          6
#define SWIMulticastreq   7

#undef  Mbuf_OpenSession
#define Mbuf_OpenSession      0x4a580
#undef  Mbuf_CloseSession
#define Mbuf_CloseSession     0x4a581
#undef  Mbuf_Memory
#define Mbuf_Memory           0x4a582
#undef  Mbuf_Statistic
#define Mbuf_Statistic        0x4a583
#undef  Mbuf_Control
#define Mbuf_Control          0x4a584

// Features supported flags
#define EYMulticast      1 << 0        // Multicast supported
#define EYPromiscuous    1 << 1        // promiscuous reception supported
#define EYRxOwn          1 << 2        // i/f rx'es own packets
#define EYStnNoReq       1 << 3        // fixed station number set in machines CMOS
#define EYRxErrPkt       1 << 4        // station can receive errored packets
#define EYHasHWAdd       1 << 5        // Interface has hardware address
#define EYChgHWAdd       1 << 6        // Interface can alter its HW address
#define EYPtP            1 << 7        // i/f is point to point link
#define EYStdStats       1 << 8        // driver supplies std stats
#define EYExtStats       1 << 9        // driver supplies extended stats
#define EYVirtDrvr       1 << 10       // driver is virtual driver
#define EYVirtDrvrSW     1 << 11       // driver is virtual driver software only

#define EYFlagsSupported ( EYMulticast | EYPromiscuous | /*EYRxOwn |*/ EYRxErrPkt | EYHasHWAdd | EYChgHWAdd | EYStdStats | EYExtStats )
                         
#define EY_MTU           1500      // max ethernet packet length (allow for poss extra byte ..
#define ETH_ZLEN         64     // count to get min ethernet packet length

#define IFF_PROMISC  2   // needs setting correct
#define IFF_ALLMULTI 1   // needs setting correct

// error return codes
#define ENODEV 1
#define ENOMEM 2
#define EAGAIN 3


