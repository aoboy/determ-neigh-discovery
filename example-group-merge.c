/** ------------------------------------------------------------------------
 * Department of Automatic Control,
 * KTH - Royal Institute of Technology,
 * School of Electrical Engineering
 * @address: Osquldasvag 10, SE-10044, STOCKHOLM, Sweden
 * @author: António Gonga < gonga@ee.kth.se>
 *
 * @date: March 9th, 2015
 * @filename: example-deterministic-app.c
 * @description: this example is used together with the medal RDC layer
 *               for testbed evaluations such as INDRIYA and TWIST*
 * NOTICE: This file is part of research I have been developing at KTH. You
 *         are welcome to modify it, AS LONG AS you leave this head notice
 *         and the author is properly acknowledged.
 * ------------------------------------------------------------------------*/

#include "./dnde-neigh-G.h"

#include "contiki.h"
#include "net/rime.h"
#include "random.h"

#include "dev/button-sensor.h"

#include "dev/leds.h"

#include "node-id.h"

#define DEBUG 1

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static struct rime_sniffer pkt_rcv_sniffer;

static uint8_t start_app_flag = 0;

//static void (* input_pkt)(void);
static void input_pkt();


RIME_SNIFFER(pkt_rcv_sniffer, input_pkt, NULL);

/*---------------------------------------------------------------------------*/
PROCESS(example_all2all_process, "example all2all process");
AUTOSTART_PROCESSES(&example_all2all_process);
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/**extern uint8_t get_curr_neighbors();
extern uint8_t get_elapsed_rounds();
extern uint16_t get_discovery_latency();*/
extern void start_protocol(uint8_t start_id);
/*---------------------------------------------------------------------------*/
static void input_pkt(){
}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
//#define STARTER_ID 174
#define STARTER_ID 1
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/**
 * @brief PROCESS_THREAD
 */
PROCESS_THREAD(example_all2all_process, ev, data){
    static struct etimer et;
    /** the process exits removing the sniffer created.*/
    //PROCESS_EXITHANDLER(rime_sniffer_remove(&pkt_rcv_sniffer);)

    PROCESS_BEGIN();

    /** Create a sniffer to receive incoming packets*/
    RIME_SNIFFER(pkt_rcv_sniffer, input_pkt, NULL);

    /** Tell RIME to add the sniffer*/
    rime_sniffer_add(&pkt_rcv_sniffer);
 
     //start up protocol..
  
    if(start_app_flag == 0){
	start_app_flag = 1;
      
	etimer_set(&et, 3*CLOCK_SECOND);
	
	PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

	start_protocol(1);
    }
    PRINTF("NET_SIZE: %u, PLD_SIZE: %u, T: %u\n", CONF_NETWORK_SIZE, NETWORK_PAYLOAD_SIZE,       
               get_node_period());
    
    while(1) {
        PROCESS_YIELD();
      
      
        /*uint16_t nodePeriod = get_node_period();
	uint16_t randValue  = random_rand() % ((nodePeriod*nodePeriod)/4);
	PRINTF("Random:%u->%u, ID:R[1][0]=%u.%u, %u\n", randValue, random_int(nodePeriod*nodePeriod/4), 
	                                  rimeaddr_node_addr.u8[1], rimeaddr_node_addr.u8[0], RTIMER_NOW());
	
        //etimer_set(&et, CLOCK_SECOND);
	
	//PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));*/
	
    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/

