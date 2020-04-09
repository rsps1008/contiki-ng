/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

#include "contiki.h"
#include "net/routing/routing.h"
#include "net/netstack.h"
#include "net/ipv6/simple-udp.h"
#include "sys/node-id.h"
#include "net/ipv6/uip-ds6-route.h"
#include "net/ipv6/uip-sr.h"
#include "net/mac/tsch/tsch.h"

#include "sys/log.h"
#define LOG_MODULE "App"
#define LOG_LEVEL LOG_LEVEL_INFO

#define WITH_SERVER_REPLY  1
#define UDP_CLIENT_PORT	8765
#define UDP_SERVER_PORT	5678

#define DEBUG DEBUG_PRINT
#include "net/ipv6/uip-debug.h"

static struct simple_udp_connection udp_conn;

PROCESS(udp_server_process, "UDP server");
AUTOSTART_PROCESSES(&udp_server_process);
/*---------------------------------------------------------------------------*/
static void
udp_rx_callback(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
  LOG_INFO(";%.*s;%d;", datalen, (char *) data, (int8_t)packetbuf_attr(PACKETBUF_ATTR_RSSI));
  LOG_INFO_6ADDR(sender_addr);
  LOG_INFO_("\n");
#if WITH_SERVER_REPLY
  /* send back the same string to the client as an echo reply */
  LOG_INFO("Sending response.\n");
  simple_udp_sendto(&udp_conn, data, datalen, sender_addr);
#endif /* WITH_SERVER_REPLY */
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_server_process, ev, data)
{
  static int is_coordinator;
  PROCESS_BEGIN();

  is_coordinator = 1;

#if CONTIKI_TARGET_COOJA || CONTIKI_TARGET_Z1
  is_coordinator = (node_id == 1);
#endif

  /* Initialize DAG root */
  if(is_coordinator) {
    NETSTACK_ROUTING.root_start();
  }
  NETSTACK_MAC.on();

  /* Initialize UDP connection */
  simple_udp_register(&udp_conn, UDP_SERVER_PORT, NULL,
                      UDP_CLIENT_PORT, udp_rx_callback);
  #if WITH_PERIODIC_ROUTES_PRINT
  {
    static struct etimer et;
    /* Print out routing tables every minute */
    etimer_set(&et, CLOCK_SECOND * 60);
    while(1) {
      /* Used for non-regression testing */
      #if (UIP_MAX_ROUTES != 0)
        LOG_INFO("Routing entries: %u\n", uip_ds6_route_num_routes());
      #endif
      #if (UIP_SR_LINK_NUM != 0)
        LOG_INFO("Routing links: %u\n", uip_sr_num_nodes());
      #endif
      PROCESS_YIELD_UNTIL(etimer_expired(&et));
      etimer_reset(&et);
    }
  }
  #endif /* WITH_PERIODIC_ROUTES_PRINT */

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
