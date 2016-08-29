/// FILE 1 - MAIN PROGRAM FILE.. 
/** ------------------------------------------------------------------------
 * Department of Automatic Control,
 * KTH - Royal Institute of Technology,
 * School of Electrical Engineering
 * @address: Osquldasvag 10, SE-10044, STOCKHOLM, Sweden
 * @author: António Gonga < gonga@ee.kth.se>
 *
 * @date: March 9th, 2015
 * @filename: example-generic-dnde-G.c
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
//////////////////////////////////////////////////////////////////////////////////////////////
///FILE 2 -  ALGORITHM FILE_1.. 
//////////////////////////////////////////////////////////////////////////////////////////////
/**
 * ------------------------------------------------------------------------
 * KTH Royal Institute of Technology
 *   School of Electrical Engineering
 *      Dept. of Automatic Control
 * ------------------------------------------------------------------------
 * @author: António Gonga < gonga@ee.kth.se>, PhD candidate
 * @date: March 9th 2015
 * @file: generic-dnde-v2.c 
 * @brief: file contains functions for manipulating the neighbors list
 * ------------------------------------------------------------------------ 
 * @info: Auxiliary functions file for a generic deterministic neighbor 
 *               discovery with epidemics. USES periods instead 
 *               of duty-cycles
 * 
 */


///=========================================================================/
///=========================================================================/
#include "contiki-conf.h"
#include "dev/leds.h"
#include "dev/radio.h"
#include "dev/watchdog.h"
#include "lib/random.h"
/** RDC driver file*/
#include "./generic-dnde.h"
/** */
#include "net/netstack.h"
#include "net/rime.h"
#include "sys/compower.h"
#include "sys/pt.h"
#include "sys/rtimer.h"


#include "dev/leds.h"
#include "./cc2420.h"
#include "dev/button-sensor.h"
#include "dev/serial-line.h"
#include "node-id.h"

#include "./dnde-neigh-G.h"

#include <string.h>

///=========================================================================/
///=========================================================================/
#define CDEBUG 1
#if CDEBUG
#include <stdio.h>
volatile char *cooja_debug_ptr;
#define COOJA_DEBUG_PRINTF(...) \
    do{ char tmp[100]; sprintf(tmp,__VA_ARGS__); cooja_debug_ptr=tmp; } while(0);
#else //COOJA_DEBUG_PRINTF
#define COOJA_DEBUG_PRINTF(...)
#endif //COOJA_DEBUG_PRINTF
///=========================================================================/
///=========================================================================/
#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else   //DEBUG
#define PRINTF(...)
#endif //DEBUG
///=========================================================================/
///=========================================================================/
#define ONE_KILO            (1024)
#define ONE_MSEC            (RTIMER_SECOND/ONE_KILO)
#define TS                  (15*ONE_MSEC)
#define TS_75P              ((3*TS)/4)
#define TS_50P              (TS/2)
#define TS_25P              (TS/4)
#define TS_20P              (TS/5)
#define HALF_MSEC           (ONE_MSEC/2)
///=========================================================================/
///=========================================================================/
#define CCA_COUNT_MAX                2
#define CCA_COUNT_MAX_TX             6
#define CCA_CHECK_TIME               RTIMER_ARCH_SECOND/8192
#define CCA_SLEEP_TIME               (RTIMER_ARCH_SECOND / 200)+1 //1,6ms
///=========================================================================/
///=========================================================================/
static volatile uint16_t slot_counter        = 0;
static volatile uint8_t  probe_slot_counter  = 0;
static volatile uint16_t probe_time          = 0;
static volatile uint16_t period_length       = 0;
static volatile uint16_t slot_counter_upperB = 0;
static volatile uint16_t slot_upperBound     = 0;
static volatile uint16_t T_period_lenghth    = 0;
static volatile uint16_t node_slots_offset   = 0;

static volatile uint8_t probe_offset = 0;
static volatile uint16_t probe_jfact = 0;

static volatile uint16_t anchor_time = 0;

static volatile uint8_t probe_distance       = 2;
static volatile uint8_t curr_frac_nodes      = 0;
static volatile uint8_t curr_h1_nodes        = 0;

static volatile uint8_t overflow_flag        = 0;
static volatile uint8_t beacon_2_flag        = 0;
///=========================================================================/
///=========================================================================/
#if CONF_ND_DUTYCYCLE !=0
//node duty-cycle: 10 is default
static volatile uint8_t node_dcycle = CONF_ND_DUTYCYCLE;
#else //
static volatile uint8_t node_dcycle = 1;
#endif //CONF_ND_DUTYCYCLE !=0
///=========================================================================/
#define DUTY_CYCLES_VEC_SIZE 4 //add, 4, 6, 8, 12 later
//static const uint8_t duty_cyles [DUTY_CYCLES_VEC_SIZE] = {1, 5, 10}; //x100
static const uint8_t duty_cyles [] = {1,1,2, 3, 4, 5}; //x100

///=========================================================================/
#define PERIODS_VEC_SIZE 2
static const uint8_t periods_vec [] = {200, 133, 167, 133,100, 150, 133, 100}; //x100
//static const uint8_t periods_vec [] = {100, 67, 40, 40, 100, 67 }; //x100
///=========================================================================/
static uint8_t i3e154_channels [] ={11, 12, 13, 14, 15, 16, 17,
                                    18, 19, 20, 21, 22, 23, 24, 25, 26};
///=========================================================================/
static volatile uint16_t discovery_time  = 0;
//static volatile uint16_t slotRefTime    = 0; //no longer used

static struct pt pt;
static struct rtimer generic_timer;
///=========================================================================/
///=========================================================================/
static volatile uint8_t radio_is_on_flag       = 0;
static volatile uint8_t radio_read_flag        = 0;

static volatile uint16_t energy_counter        = 0;
static volatile uint8_t discovery_is_on        = 0;
static volatile uint8_t rand_offset_wait_flag  = 0;
static volatile uint8_t rounds_counter         = 0;
static volatile uint8_t isAnchorFlag           = 0;
static volatile uint8_t txPacketFlag           = 0;
static volatile uint16_t extra_probe_counter   = 0;

static volatile uint8_t energy_per_period      = 0;
static volatile uint8_t extra_probe_per_period = 0;
///=========================================================================/
static char power_cycle(struct rtimer *rt, void* ptr);
static char schedule_fixed(struct rtimer *rt, rtimer_clock_t next_time);

static void print_nodes(uint8_t );
///=========================================================================/
static const char *h2_Print             = "h2_output";
static const char *auto_msg             = "newround";
static const char *energyPrint          = "energyPrint";
static volatile uint8_t printOutputFlag = 0;
///=========================================================================/
///=========================================================================/
PROCESS(autotrigger_process, "Generic DNDE auto-trigger process");
PROCESS(output_process, "Generic DNDE Output serial process");

//.....UNNECESSAY FUNCTIONS REMOVED ... 


///=========================================================================/
///=========================================================================/
/**
 * @brief get_discovery_time
 * @return
 */
uint16_t get_discovery_time(){
  discovery_time = node_slots_offset + slot_counter + 1;
  return discovery_time;
}
///=========================================================================/
///=========================================================================/
/**
 * @brief get_anchor_time
 * @return
 */
uint16_t get_anchor_time(){
  return anchor_time;
}
///=========================================================================/
///=========================================================================/
/**
 * @brief set_period
 */
static void set_period(void){
  //period_length = (2*100)/node_dcycle;
  uint8_t pidx = rimeaddr_node_addr.u8[0]%PERIODS_VEC_SIZE;
  //uint8_t pidx = random_int(PERIODS_VEC_SIZE);
  period_length = periods_vec[pidx];
}
///=========================================================================/
///=========================================================================/
/**
 * @brief get_node_period
 * @return
 */
uint8_t get_node_period(){
  //first set period if it is not been set yet...
  return period_length;
}
///=========================================================================/
///=========================================================================/
/**
 * @brief get_node_dutycycle
 * @return
 */
uint8_t get_node_dutycycle(){
  return node_dcycle;
}
///=========================================================================/
///=========================================================================/
/**
 * @brief set_period_upper_bound
 */
static void set_period_upper_bound(){
    //T_period_lenghth = (period_length*period_length)/(2*probe_distance);
  uint16_t tmp_period = periods_vec[0];
  
  T_period_lenghth = (tmp_period*tmp_period)/(2*probe_distance);
}
///=========================================================================/
///=========================================================================/
/**
 * @brief schedule_probe
 */
static void schedule_probe(){
  probe_slot_counter = (probe_slot_counter + 2)%(period_length/2);
  probe_time = slot_counter + 1 + probe_slot_counter;
}
///=========================================================================/
///=========================================================================/
/**
 * @brief initialize_settings
 */
static void initialize_settings(){

    //since we now know the duty cycle we can compute the cycle_length
    set_period();
    //and finally we can now compute the node 'extended' period.
    set_period_upper_bound();
}
///=========================================================================/
///=========================================================================/
/**
 * @brief restart_discovery
 */
static void restart_discovery(void){
     energy_counter     = 0;
     
     discovery_is_on    = 0;
     
     discovery_time     = 0;     
     
     slot_counter       = 0;
     
     probe_slot_counter = 0;
     
     probe_time         = 0;
     
     probe_jfact        = 0;
     
     curr_frac_nodes    = 0;   
     
     curr_h1_nodes      = 0;
     
     extra_probe_counter= 0;
     
     //beacon 2 sent using random offset
     //this flag enables that to happen..
     beacon_2_flag      = 0;
     
     
     /*#if CONF_ASYMMETRIC != 0
      uint8_t dc_idx = random_int(DUTY_CYCLES_VEC_SIZE-1);
      node_dcycle    = duty_cyles[dc_idx];
      #endif *///CONF_ASYMMETRIC
     
      //clear neighbors table to comence a new round..
      neighs_flush_all();

      //now we can call to restart the process..
      ///@note: INDRIYA ONLY...AUTOMATIC MODE..

      process_post(&autotrigger_process, serial_line_event_message, auto_msg); 
         
}
///=========================================================================/
///=========================================================================/
/**
 * @brief transmit_epidemic
 */
static void transmit_epidemic(){
  
    packetbuf_clear();
    
    data_packet_t *dpkt = (data_packet_t*)packetbuf_dataptr();
    
    if(isAnchorFlag){
	dpkt->type = ANCHOR_PKT;
    }else{
        dpkt->type = PROBE_PKT;
    }
    //add sender offset.
    dpkt->offset   = probe_offset;
    dpkt->period   = get_node_period();
    
    uint8_t pldSize = neighs_add2payload(&dpkt->data[0], isAnchorFlag, probe_offset);
    
    if(pldSize){
      
	  rtimer_clock_t t0;
	  uint8_t i = 0, pkt_seen = 0, ccaCounter= 0;
	  
	  ccaCounter= randomint_between(CCA_COUNT_MAX, CCA_COUNT_MAX_TX);
	  
	  for( i = 0; i < ccaCounter && beacon_2_flag; i++){
	    
	      t0 = RTIMER_NOW();
	      
	      while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + CCA_CHECK_TIME)){
		//do nothing.. KUNANGA
	      }
	      
	      if(NETSTACK_RADIO.channel_clear() == 0){
		    pkt_seen = 1;
		    break; // maybe return here .. collision is expected
	      }

	  }
	  
	  if(pkt_seen == 0 && radio_read_flag == 0){
	     NETSTACK_RADIO.send((void*)packetbuf_dataptr(),DATAPKT_HDR_LEN + pldSize);
	  }
    }
}
///=========================================================================/
///=========================================================================/
/**
 * @brief schedule_fixed
 * @param rt
 * @param next_time
 * @return
 */
static char schedule_fixed(struct rtimer *rt, rtimer_clock_t next_time){
   
    if(RTIMER_CLOCK_LT(next_time, RTIMER_NOW() + 1)) {
      next_time = RTIMER_NOW() + 1;
    }

    int ret = rtimer_set(&generic_timer, next_time, 1,
		      (void (*)(struct rtimer *, void *))power_cycle, NULL);
    if(ret){
	  PRINTF("synchronization failed\n");
    }  
    
    return 0;
}
///=========================================================================/
///=========================================================================/
/**
 * @brief power_cycle
 * @param rt
 * @param ptr
 * @return
 */
static char power_cycle(struct rtimer *rt, void* ptr){


   //start protothread function
   PT_BEGIN(&pt);
   
   while(1){
     
	if(rand_offset_wait_flag){
	  
	   for(slot_counter = 0; slot_counter < node_slots_offset; slot_counter++){
	       rtimer_clock_t tnow_wait = RTIMER_NOW();
	       
	       schedule_fixed(rt, tnow_wait + TS);
	       PT_YIELD(&pt);	     
	  }
	  
	  //random slots offset is over.. disable flag
	  rand_offset_wait_flag = 0;
	  COOJA_DEBUG_PRINTF("END oF RANDOM OFFSET:%u..%u\n", rounds_counter,node_slots_offset);
	}//else{}

	static rtimer_clock_t t0;
	//rtimer_clock_t tnow;
	
	//for(slot_counter = 0; slot_counter < slot_counter_upperB+2; slot_counter++){
	for(slot_counter = 0; slot_counter <= slot_upperBound; slot_counter++){

	    //this offset is transmitted
	    probe_offset = slot_counter%period_length;	    	    	    
	    
	    if(slot_counter != 0 && (slot_counter % period_length) == 0){
		  
		  anchor_time = slot_counter - probe_offset;
		  
		  //update j coeficient of neighbor nodes
		  neighs_jfactor_update();
	    }
	    //txPacketFlag
	    //this is an anchor slot..
	    if(slot_counter == 0 || (slot_counter%period_length) == 0){
	      
	          isAnchorFlag       = 1; //ANCHOR flag		 
		  txPacketFlag       = 1;  //Set beacon Flag to TRUE
		  
		  //Compute next probe		
		  probe_slot_counter = random_int(period_length/2+2);
		  
		  probe_slot_counter = 2*probe_jfact + 2;
		  probe_time = slot_counter + probe_slot_counter;	
		  
		  probe_jfact = (1+probe_jfact)%(period_length/4+1);
		  
		  energy_per_period++;
		  
		  //reset overflow_flag only used for indirect discovery
		  overflow_flag = 0;
		  
		  //COOJA_DEBUG_PRINTF("AnchorID:%u, ProbeID:%u\n", slot_counter, probe_slot_counter);
	    }else{//ELSE OF (slot_counter%period_length)
	          //check it is a probe slot..
	          if(slot_counter == probe_time){

		       isAnchorFlag  = 0; //RESET Anchor Flag			
		       txPacketFlag  = 1;  //Set beacon Flag to TRUE
		       
		       //the probe offset on a probe slot is based on the 
		       //probe slot counter.. bcz of strip-probbing.
		       probe_offset  = probe_slot_counter;
		       
		       energy_per_period++;
		       
		       //reset overflow_flag only used for indirect discovery
		       overflow_flag = 0;		       
		       
		       //COOJA_DEBUG_PRINTF("ProbeID:%u, time:%u\n", probe_slot_counter, probe_time);
		  }else{//ELSE slot_counter == probe_time
		       
		        //probe also possible Hop 2 neighbors
			if(isthere_anchor(2, slot_counter)){
			  
			    isAnchorFlag  = 0; //RESET Anchor Flag
			    txPacketFlag  = 1; //Set beacon Flag to TRUE
			    
			    extra_probe_counter++;

			    extra_probe_per_period++;
			    
		            //reset overflow_flag only used for indirect discovery
		            overflow_flag = 1;
		  
			    //COOJA_DEBUG_PRINTF("TgT_ID:%u, time:%u\n",  probe_offset, slot_counter + probe_offset);
			} //END of		        
		  } //END OF slot_counter == probe_time
	      
	    }//END OF (slot_counter%period_length)
	    
	    if(txPacketFlag){
	      
	        txPacketFlag = 0;//RESET txPacketFlag
	        
	        //total energy
	        energy_counter++;				
		
	        //COOJA_DEBUG_PRINTF("PACKET_TX_INIT:%u,%u\n", slot_counter, RTIMER_NOW());
	      
	        rtimer_clock_t tnow = RTIMER_NOW();
	      
		cc2420_set_channel(i3e154_channels[0]);
		
		if(radio_is_on_flag == 0){
		      //COOJA_DEBUG_PRINTF("b17\n");
		      on();
		}		

		
		//send first beacon without offset
		beacon_2_flag = 0;
		
		//trabsmit beacon packet
		transmit_epidemic();
	        //wait for sometime to receive some packets
		schedule_fixed(rt, tnow + 3*TS_20P);
		PT_YIELD(&pt);
		
		//Once the cycle returns, it is time to send the second beacon
		//transmit Anchor Probe Here
	        tnow = RTIMER_NOW();
		
		//send second beacon with offset
		beacon_2_flag = 1;
		
		//trabsmit beacon packet
		transmit_epidemic();
		
		//here we schedule to comence the next time slot.
		schedule_fixed(rt, tnow + 2*TS_20P +overflow_flag*TS_20P);
		PT_YIELD(&pt);

		 //we need to switch the radio off.. 
		if(radio_is_on_flag == 1){
		    //COOJA_DEBUG_PRINTF("b17\n");
		    off();
		}
		
		//RESET txPacketFlag
		txPacketFlag = 0;//RESET txPacketFlag
		
	    }else{ //END if(txPacketFlag)
	    
		  //turn radio off 
		  if(radio_is_on_flag == 1){
		      //COOJA_DEBUG_PRINTF("b17\n");
		      off();
		  }		
		  
		  //here we schedule to comence the next time slot.
		  schedule_fixed(rt, RTIMER_NOW() + TS);
		  PT_YIELD(&pt);	    
	  
	    } //END if(txPacketFlag)
	    
	    //printing function...
	    /*if(slot_counter!= 0 && slot_counter % slot_counter_upperB == 0){

		    //h2_Print
		    process_post(&output_process,PROCESS_EVENT_CONTINUE, h2_Print);

	    } */ //end of
	    
	    //print energy consumption 
	    if(slot_counter != 0 && (slot_counter % period_length) == 0){
	      
	          //print energy consumption values
		  print_nodes(1);	
	          //process_post(&output_process,PROCESS_EVENT_CONTINUE, energyPrint);
		  
		  energy_per_period = 0;
		  extra_probe_per_period = 0;		  
	    }	    

	} //END for LOOP
      
      COOJA_DEBUG_PRINTF("END oF DISCOVERY CYCLE:%u\n", rounds_counter);
      //AN UPPER_BOUND HAVE BEEN REACHED.. Exit the 
      discovery_is_on = 0;
      restart_discovery();
      
      //cycle is over, YIELD waiting for another call..
      PT_YIELD(&pt);
      
   }//end of WHILE(1)
   
   //end protothread function:AG
   PT_END(&pt);
}
///=========================================================================/
///=========================================================================/
/**
 * @brief start_discovery_process
 */
static void start_discovery_process(){
  uint16_t Tup_bound = 0, random_slot = 1;
  
  uint16_t random_slot = 1 + random_rand()%(period_length);
  
  Tup_bound = (period_length*period_length)/4; 
  
  //random_slot = random_slot + random_rand()%Tup_bound ;
  
  if (rimeaddr_node_addr.u8[0] < 130){
    random_slot = random_slot + random_rand()%period_length;
  }else{
    random_slot = random_slot + Tup_bound + random_rand()%period_length;
  }
  //enable random offset wait flag..
  rand_offset_wait_flag = 1;
  node_slots_offset     = random_slot;
  slot_upperBound       = 20000-node_slots_offset;
  
  /*uint16_t random_slot = 1 + random_int(T_period_lenghth);
  node_slots_offset    = random_slot;
  
  slot_counter_upperB = T_period_lenghth;
  slot_counter_upperB = random_slot + slot_counter_upperB;
  
  rtimer_clock_t startTime = RTIMER_NOW() + random_slot*TS;*/
  
  rtimer_clock_t startTime = RTIMER_NOW() + 1;

  int ret = rtimer_set(&generic_timer, startTime, 1,
                     (void (*)(struct rtimer *, void *))power_cycle, NULL);
  if(ret){
        PRINTF("synchronization failed\n");
  }
}
///=========================================================================/
///=========================================================================/
/**
 * @brief input
 */
static void input(){
  int len = 0;

   //read packet from the radio driver
   radio_read_flag = 1;

   len = NETSTACK_RADIO.read(packetbuf_dataptr(), PACKETBUF_SIZE);

   radio_read_flag = 0;

   //check length of the packet if its a valid packet..
   if(len <= 0){
       return ;
   }

   data_packet_t *inpkt = (data_packet_t*)packetbuf_dataptr();
   
   if((inpkt->type & 0x0F) == DATA_PKT){
     
       //get the discovery time..
       discovery_time = node_slots_offset + slot_counter + 1;

       ///add new nodes to the list
       //neighs_sregister(&inpkt->data[0]);
       
       //add new nodes 
       neighs_register(inpkt, len-DATAPKT_HDR_LEN, probe_offset);              
       
       //uint8_t tmp_num_neighs = neighs_num_nodes();
       
       uint8_t tmp_num_neighs = neighs_xhops(1);

       ///check if we have discovered all our neighbors
       if(curr_frac_nodes < tmp_num_neighs){

           ///set current number of neighbors
           curr_frac_nodes = tmp_num_neighs;

           /////discovery_time = node_slots_offset + slot_counter + 1;

	   process_post(&output_process,PROCESS_EVENT_CONTINUE, NULL);
       }
       
   } ///inpkt->type == DATA_PKT   
}
///=========================================================================/
///=========================================================================/
/**
 * @brief send_packet
 * @param sent
 * @param ptr
 */
static void send_packet(mac_callback_t sent, void *ptr){
   //do nothing...
}
///=========================================================================/
///=========================================================================/
/**
 * @brief send_list
 * @param sent
 * @param ptr
 * @param buf_list
 */
static void
send_list(mac_callback_t sent, void *ptr, struct rdc_buf_list *buf_list){
    //do nothing...
}
///=========================================================================/
///=========================================================================/
/**
 * @brief on
 * @return
 */
static int rd_on(void){
  return NETSTACK_RADIO.on();
}
///=========================================================================/
///=========================================================================/
/**
 * @brief off
 * @param keep_radio_on
 * @return
 */
static int rd_off(int keep_radio_on){
    return NETSTACK_RADIO.off();
}
///=========================================================================/
///=========================================================================/
/**
 * @brief channel_check_interval
 * @return
 */
static unsigned short channel_check_interval(void){
  //return 8;
  return 16;
}
///=========================================================================/
///=========================================================================/
/**
 * @brief init
 */
static void init(){
  
    ///initialize setting must be called before neighbors init bcz
    initialize_settings();
    
    ///initialize list of neighbors
    neighs_init();

    //on();
    
    PT_INIT(&pt);
    
    /** start the node discovery process NOTE: ONLY USED FOR COOJA SIMULATIONS*/
    process_start(&autotrigger_process, NULL);

    process_start(&output_process, NULL);

    ///START UP PROTOCOL...INDRIYA ONLY.....
    ///@todo..comment this code afterwards..
    cc2420_set_channel(i3e154_channels[random_int(num_channels)]);
}
///=========================================================================/
///=========================================================================/
const struct rdc_driver generic_driver = {
    "Generic-DNDEpidemic RDC v1.0",
    init,
    send_packet,
    send_list,
    input,
    rd_on,
    rd_off,
    channel_check_interval,
};

///=========================================================================/
///=========================================================================/
/**
 * @brief start_protocol
 * @param starterID
 */
void start_protocol(uint8_t starterID){
   if(1){
     
      rounds_counter++;
      
      //initialize_settings();
      //clear neighbors table to comence a new round..
      //neighs_flush_all();      
      
      //PRINTF("DC:%u..%u\n", node_dcycle, random_rand());
      
      discovery_is_on = 1;
      
      
      off();
      
      start_discovery_process();
   }
}
///=========================================================================/
///=========================================================================/
/**
 * @brief PROCESS_THREAD
 */
PROCESS_THREAD(output_process, ev, data){

    PROCESS_BEGIN();

    while(1){

        PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_CONTINUE);
	
	char *cmd_str = (char*)data;
	
	if(cmd_str == NULL){
	
		PRINTF(">:%d,%d,%u,%u,%u,%u,%u,%u\n", rounds_counter, rimeaddr_node_addr.u8[0],
                   CHANNEL_SIZE, neighs_xhops(1), neighs_h2_indirect(),get_discovery_time(), energy_usage(),period_length);
	}else{
	     if(!strcmp(cmd_str, "energyPrint")){
	        print_nodes(1);
	     }

	     if(!strcmp(cmd_str, "h2_output")){
	        print_nodes(0);
	     }	     	     
	}
    } //END WHILE(1)
    PROCESS_END();
}
///=========================================================================/
///=========================================================================/
/**
 * @brief PROCESS_THREAD
 */
PROCESS_THREAD(autotrigger_process, ev, data){
    static struct etimer et;

    PROCESS_BEGIN();
    
    int8_t val = -5;

    PRINTF("nodeID: %u.%u booted, T=%u, sz:%d\n",
                  rimeaddr_node_addr.u8[1],
                     rimeaddr_node_addr.u8[0],
                    period_length, sizeof(val) );

    while(1){

        PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message);

        char *command = (char*)data;

        if(command != NULL){
	  
            if(!strcmp(command, "allconv")){
                //cc2420_set_channel(26);

                discovery_is_on = 0;
            } //end of "allconv"

            if(!strcmp(command, "newround")){
                //increment number of rounds.
                rounds_counter = rounds_counter + 1;

                //acts like a barrier/mutex
                discovery_is_on = 1;

                off();

                start_discovery_process();

            } //end of !strcmp(command, "newround"
        } //end of command != NULL
    }
    PROCESS_END();
}
///=========================================================================/
///=========================================================================/
/**
 * @brief print_nodes
 * @param optionPrint
 */
static void print_nodes(uint8_t optionPrint){
       if(optionPrint){
	      PRINTF("EN>:%d,%d,%u,%u,%u,%u,%u,%u,%u,%u\n", rounds_counter, rimeaddr_node_addr.u8[0],
                 CHANNEL_SIZE, neighs_xhops(1), neighs_h2_indirect(),get_discovery_time(), 
		     energy_usage(),energy_usage_period(), get_node_period(), neighs_xhops(2));
		  
	      energy_per_period = 0;
	      extra_probe_per_period = 0;	
       }else{
	  //print all nodes discovered
	  struct nodelist_item *dnl = neighs_getlist();
	  PRINTF("ID:%u/%u/%u/%u->", rimeaddr_node_addr.u8[0],  
				    neighs_xhops(1),neighs_h2_indirect(), neighs_xhops(2));
	  
	  uint16_t t1, t2;
	  
	  for( ;    dnl != NULL;    dnl = list_item_next(dnl)){
	      
		t1 = dnl->tknown;
		t2 = dnl->tconfirmed;
		
		if(dnl->hopcount == 1 && ((t2-t1) > 0)){
		    PRINTF("|%u,%u,%u,%u", dnl->node_id, dnl->tknown, dnl->tconfirmed, dnl->period);
		}
	  }
	  PRINTF("|\n");
       }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// FILE 3
//////////////////////////////////////////////////////////////////////////////////////////////
#include "lib/list.h"
#include "lib/memb.h"
#include "lib/random.h"
#include "contiki-conf.h"
#include "net/rime/rimeaddr.h"

#include "./dnde-neigh-G.h"

#include <string.h>

///=========================================================================/
#define  PROBE_ABS(t1) (signed short)(t1) < 0 ?\
                     (signed short)(-(t1)) : ((signed short)(t1))
///=========================================================================/
///=========================================================================/
#define CDEBUG 0
#if CDEBUG
#include <stdio.h>
//volatile char *cooja_debug_ptr;
volatile char *cooja_debug_ptr;
#define COOJA_DEBUG_PRINTF(...) \
    do{ char tmp[100]; sprintf(tmp,__VA_ARGS__); cooja_debug_ptr=tmp; } while(0);
#else //COOJA_DEBUG_PRINTF
#define COOJA_DEBUG_PRINTF(...)
#endif //COOJA_DEBUG_PRINTF

///=========================================================================/
///=========================================================================/
#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else   //DEBUG
#define PRINTF(...)
#endif //DEBUG
///=========================================================================/
///=========================================================================/
LIST(neighs_list);
#if CONF_NETWORK_SIZE != 0
MEMB(neighs_memb, struct nodelist_item, CONF_NETWORK_SIZE);
#else
MEMB(neighs_memb, struct nodelist_item, 2);
#endif //CONF_NETWORK_SIZE != 0
///=========================================================================/
///=========================================================================/
/**
 * @brief neighs_init
 */
void neighs_init(){
    //
    list_init(neighs_list);
    memb_init(&neighs_memb);
      
    neighs_flush_all();
}
///=========================================================================/
///=========================================================================/
/**
 * @brief rlocked
 */
static uint8_t rlocked;
#define GET_RLOCK() rlocked++
static void RELEASE_RLOCK(void) {
  if(rlocked == 1) {
    /*if(lock_on) {
      on();
      lock_on = 0;
    }
    if(lock_off) {
      off();
      lock_off = 0;
    }*/
  }
  rlocked--;
}
///=========================================================================/
///=========================================================================/
/**
 * @brief compute_node_period
 * @param node_duty_cycle
 * @return
 */
uint8_t compute_node_period(uint8_t node_duty_cycle){
  uint8_t periodLength = 0;
  
  if(node_duty_cycle != 0){
      periodLength = (2*100)/ node_duty_cycle;
  }
  return periodLength;
}
///=========================================================================/
///=========================================================================/
/**
 * @brief neighs_get
 * @param nodeid
 * @return
 */
struct nodelist_item *neighs_get(uint8_t nodeid){
    struct nodelist_item *localp = NULL;
    
    localp =list_head(neighs_list);

    for( ; localp != NULL; localp = list_item_next(localp)){
        if (localp->node_id == nodeid){
            return localp;
        }
    }
    return NULL;
}
///=========================================================================/
///=========================================================================/
/**
 * @brief neighs_getlist
 * @return
 */
struct nodelist_item *neighs_getlist(/*nodelist_item *list_head*/){
       //*list_head = list_head(neighs_list);
  return list_head(neighs_list);
}
///=========================================================================/
///=========================================================================/
/**
 * @brief neighs_add_itself
 */
void neighs_add_itself(){

    struct nodelist_item *ais = NULL;
    
    ais = neighs_get(rimeaddr_node_addr.u8[0]);
    
    if(ais == NULL){
        
        //we add a new element..
        ais = memb_alloc(&neighs_memb);

	if(ais != NULL){
            ais->node_id  = rimeaddr_node_addr.u8[0];
	    
            ais->hopcount = 0;
	    ais->offset   = 0;
	    ais->offsetj  = 0;
	    ais->max_txhop2 = 0;
	    ais->tconfirmed = 0;
	    ais->tknown     = 0;
            ais->next     = NULL;
	    
	    ais->t_anchor   = 0;
	    ais->j_factor   = 0;
	    ais->period     = get_node_period();

            //add new element to the list..
            list_add(neighs_list, ais);

            return;
        }
    }
}
///=========================================================================/
///=========================================================================/
/**
 * @brief get_fraction_nodes
 * @return
 */
uint8_t neighs_num_nodes(){
    uint8_t frac_neigh = 0;

    struct nodelist_item *lpf = NULL;
    
    lpf = list_head(neighs_list);
    
    for( ; lpf != NULL; lpf = list_item_next(lpf)){
        frac_neigh = frac_neigh + 1;
    }

    return frac_neigh;
}
///=========================================================================/
///=========================================================================/
/**
 * @brief neighs_xhops
 * @param xhops
 * @return
 */
uint8_t neighs_xhops(uint8_t xhops){
  uint8_t frac_neigh = 0;
  
    struct nodelist_item *lpf = NULL;

    lpf = list_head(neighs_list);
    
    for( ; lpf != NULL; lpf = list_item_next(lpf)){
        
	if (lpf->hopcount == xhops){
	  frac_neigh = frac_neigh + 1;
	}
    } 
    
    return frac_neigh;
}
///=========================================================================/
///=========================================================================/
/**
 * @brief neighs_h2_indirect
 * @return
 */
uint8_t neighs_h2_indirect(){
  uint8_t frac_neigh = 0;
  uint16_t t1, t2;
  
    struct nodelist_item *lpf = NULL;

    lpf = list_head(neighs_list);
    
    
    for( ; lpf != NULL; lpf = list_item_next(lpf)){

 	t1 = lpf->tknown;
	t2 = lpf->tconfirmed;   
	
	if ((lpf->hopcount == 1) && ((t2-t1) > 0)){
	  frac_neigh = frac_neigh + 1;
	}
    } 
    
    return frac_neigh;
}
///=========================================================================/
///=========================================================================/
/**
 * @brief neighs_full
 * @return
 */
uint8_t neighs_all_found(){
    uint8_t i = 0;

    i = list_length(neighs_list);

    if (i ==num_neighbors){
        return 1;
    }
    return 0;
}
///=========================================================================/
///=========================================================================/
/**
 * @brief neighs_remove
 * @param nodeid
 */
void neighs_remove(uint8_t nodeid){
  
    struct nodelist_item *n2r = NULL;
    
    n2r = neighs_get(nodeid);
    
    if(n2r != NULL){
        list_remove(neighs_list, n2r);
        memb_free(&neighs_memb, n2r);
    }
}
///=========================================================================/
///=========================================================================/
/**
 * @brief neighs_flush_all
 */
void neighs_flush_all(void){
  struct nodelist_item *n2fa = NULL;

  while(1){

    n2fa = list_pop(neighs_list);

    if(n2fa != NULL) {
	list_remove(neighs_list, n2fa);
	memb_free(&neighs_memb, n2fa);
    }else {
	///add first entry here ??
	///
	neighs_add_itself();

	break;
    }
  } ///end of while(1)
}
///=========================================================================/
///=========================================================================/
/**
 * @brief neighs_jfactor_update:
 */
/**void neighs_jfactor_update(){
    struct nodelist_item *localp = NULL;
    
    localp = list_head(neighs_list);
    
    for( ; localp != NULL; localp = list_item_next(localp)){
        if (localp->node_id != rimeaddr_node_addr.u8[0]){
	  
            //local variable enables tracking offset to neighbors
	    uint16_t ta_i = get_anchor_time();
	    uint16_t ta_j = time_neighbor_anchor(localp);
	    
	    if(ta_j > ta_i){
	      	    
		localp->j_factor = localp->j_factor + 1;
		
		if (localp->hopcount == 1 &&  localp->period != get_node_period()){
		    uint8_t offsetNode = ta_j - ta_i;
		    //set the new offset.
		    localp->offsetj    = offsetNode;
		}
	    }
        }
    }  
}*/
void neighs_jfactor_update(){
    struct nodelist_item *localp = NULL;
    
    localp = list_head(neighs_list);
    
    for( ; localp != NULL; localp = list_item_next(localp)){
        if (localp->node_id != rimeaddr_node_addr.u8[0]){
	  
            //local variable enables tracking offset to neighbors
	    uint16_t ta_i = get_anchor_time();
	    uint16_t ta_j = time_neighbor_anchor(localp);
	    
	    uint16_t tp_i   = get_node_period();	    
	    uint16_t tp_j   = localp->period;
	    
	    if(ta_i > ta_j){
		if (tp_i <= tp_j){
		    localp->j_factor = localp->j_factor + 1;	
		}else{
		    localp->j_factor = localp->j_factor + tp_i/tp_j;
		}
	    }
	    
	    ta_j = time_neighbor_anchor(localp);
	    
	    if ((localp->hopcount == 1) && (ta_j > ta_i) &&  tp_j != tp_i){
		uint8_t offsetnode = ta_j - ta_i;
		//set the new offset for broadcast.
		localp->offsetj    = offsetnode;
	    }	    
        }
    }  
}
///=========================================================================/
///=========================================================================/
/**
 * @brief time_neighbor_anchor
 * @param n_item
 * @return
 */
uint16_t time_neighbor_anchor(struct nodelist_item *n_item){
  uint8_t  periodL = n_item->period;
  uint16_t ta = n_item->t_anchor + n_item->offset + periodL*n_item->j_factor;
  
  return ta;
}
///=========================================================================/
///=========================================================================/
/**
 * @brief isthere_anchor
 * @param hops_away
 * @param curr_time
 * @return
 */
uint8_t isthere_anchor(uint8_t hops_away, uint16_t curr_time){
  uint16_t time_anchor = 0;
  
    struct nodelist_item *lpf = NULL;

    lpf = list_head(neighs_list); 
   
    for( ; lpf != NULL; lpf = list_item_next(lpf)){
      
      if(lpf->hopcount  == hops_away && lpf->max_txhop2 <= MAX_TRIES_H2_NEIGH){
	   
	  time_anchor = time_neighbor_anchor(lpf);
      
	  if( time_anchor == curr_time + 1 || time_anchor == curr_time + 2 ){	
		
		lpf->max_txhop2 = lpf->max_txhop2 + 1;
		
		COOJA_DEBUG_PRINTF("%u-->%u @ %u\n",rimeaddr_node_addr.u8[0], 
				  lpf->node_id, time_anchor);
		
		return 1;
	  }
      }
    }
    
  return 0;
}
///=========================================================================/
///=========================================================================/
#define N_ABS(my_val) (((signed short)(my_val)) < 0) ? 0-(my_val) : (my_val)
///=========================================================================/
///=========================================================================/
/**
 * @brief neighs_register: receive a packet whose payload contains nodes 
 *                         that are 1 hop and 2 hops away. 
 * @param pkt_hdr: The pointer to the packet received buffer
 * @return
 */
uint8_t 
neighs_register(data_packet_t *pkt_hdr, int pldLen, uint8_t probe_counter){
    uint8_t k = 0;
    int16_t offsetH1, offsetH2;
           
    //computes the offset to the sender of the packet.
    int16_t ownOffset  = ((int16_t)probe_counter - pkt_hdr->offset);
    
    //uint8_t periodLength = compute_node_period(pkt_hdr->energy);
    
    offsetH1 = ownOffset;   
    
    //if the offset is negative, we compute the positive offset by summing
    //the sender's period length.
    if (ownOffset < 0){
          offsetH1 = pkt_hdr->period + ownOffset;
	  
	  //COOJA_DEBUG_PRINTF("dc:%u\n",compute_node_period(pkt_hdr->energy));
    }    
    
    //go though all items in the packet and add them accordingly..
    for ( k = 0; k < pldLen; k++){
      
        uint8_t dpos = k*DATA_ITEM_LEN;

        struct data_item_t *ditem = (struct data_item_t*)(&pkt_hdr->data[dpos]);

	//filter packets based on hop-count number, remove also my id
        if ((ditem->node_id != 0 && (ditem->dc_hopc & HOPC_MASK) ) && 
	                 (ditem->node_id != rimeaddr_node_addr.u8[0]) &&
	                (ditem->dc_hopc  <= MAX_HOPCOUNT)){	
	
	      //COOJA_DEBUG_PRINTF("rcd %u,%u,%u,%d,%u, %d\n",ditem->node_id, 
	     //ditem->hopcount, ditem->offset,ownOffset, offsetH1, periodLength);

	      struct nodelist_item *nli = NULL;
	
	      //check if the nodeID is already registered/received.. 
	      nli = neighs_get(ditem->node_id);
	
	      if(nli == NULL){
	
		  //we add a new element..
		  nli = memb_alloc(&neighs_memb);
	    
		  if(nli != NULL){
		      //set the id of this node
		      nli->node_id    = ditem->node_id;
		      //extract the hopcount..
		      nli->hopcount   = ditem->dc_hopc;
		      //we do not know yet :)
		      nli->max_txhop2 = 0;
		      
		      //get the time of reception
		      nli->tknown     = get_discovery_time();
		      //equal time of confirmation if node is discovered
		      //without help of epidemics
		      nli->tconfirmed = get_discovery_time();
		      
		      //if node is received as hop=2, jfactor is what allows
		      //to locate it.. we explain later..
		      nli->j_factor   = 0;	
		      
		      //what is the anchor time when this node was received
		      nli->t_anchor   = get_anchor_time();
		      
		      //retrieve the period of node.
		      nli->period = ditem->period;
		      
		      ///@TODO add something if offset greater than T/2
		      //offset = T-offset.

		      if(ditem->dc_hopc == 1){
			   nli->offset  = offsetH1;			   
			   nli->offsetj = offsetH1;
                           COOJA_DEBUG_PRINTF("%u ADD_d(%u) offset:%2u(%d)\n", rimeaddr_node_addr.u8[0], nli->node_id, offsetH1,
			                                                       ownOffset);			   
		      }else{
			
			  if(ditem->dc_hopc == 2){
			      
			      //compute offset of a hop 2 neighbor.
			      offsetH2    = ( ditem->offset + ownOffset); 
			      
			      if(offsetH2 < 0){
				  offsetH2 =  nli->period + offsetH2;
			      }
			      
			      nli->offset = offsetH2;

			      COOJA_DEBUG_PRINTF("%u Epid(h2)-> %u offset:%2d\n",rimeaddr_node_addr.u8[0], ditem->node_id, offsetH2);
			  }
		      }
      		    
		      nli->next     = NULL;

		      //add new element to the list..
		      list_add(neighs_list, nli);
		  }
	      }else{
		 
		  //we update an existing element..PROBLEM HERE..
		  if( (nli->node_id == ditem->node_id) && 
	                           (ditem->dc_hopc  < nli->hopcount)){

		        //COOJA_DEBUG_PRINTF("NULL#%u:%u-%u\n", nli->hopcount, ditem->hopcount, ditem->node_id);
		        //node already known and gets confirmation now.
			if(ditem->dc_hopc  < nli->hopcount){
			  
			    nli->hopcount   = ditem->dc_hopc ;
			    nli->tconfirmed = get_discovery_time();
			    nli->offset     = offsetH1;
			    nli->offsetj    = offsetH1;
			    
			    //generic discovery
			    nli->j_factor   = 0;			    
			    nli->t_anchor   = get_anchor_time();
			    
			    COOJA_DEBUG_PRINTF("%u ADD_i(%u) offset:%2u(%d)\n", rimeaddr_node_addr.u8[0], nli->node_id, offsetH1,
			                                                       ownOffset);				    
			}		  
		  }
		  //neighs_update(ditem);            
	      } //else
        
       } //(ditem->node_id != 0 || (ditem->hopcount <= MAX_HOPCOUNT))

    } // for ( k = 0;   

    return 0;
}
///=========================================================================/
///=========================================================================/
/**
 * @brief neighs_add2payload
 * @param data_ptr
 * @param isAnchor
 * @param probe_offset
 * @return
 */
uint8_t 
neighs_add2payload(uint8_t *data_ptr, uint8_t isAnchor, uint8_t probe_offset){
    uint8_t pkt_offset = 0, hopc=0;

    struct nodelist_item *headl = NULL;
    
    headl = list_head(neighs_list);
    
    for(;            headl != NULL;    headl = list_item_next(headl)){
	
        if(headl->node_id != 0 && headl->hopcount < MAX_HOPCOUNT){

            struct data_item_t *d2a = (struct data_item_t *)(&data_ptr[pkt_offset]);

            d2a->node_id       = headl->node_id;
	    
            d2a->dc_hopc       = headl->hopcount + 1;
	    d2a->period        = headl->period;
	    
	    d2a->offset        = headl->offsetj; //headl->offset;

            pkt_offset = pkt_offset + DATA_ITEM_LEN;
	    
	    //COOJA_DEBUG_PRINTF("Added %u,%u\n", d2a->node_id, hopc);
        }
    }
    if(pkt_offset){
        return pkt_offset;
    }
    //this is a serious error.. :(
    COOJA_DEBUG_PRINTF("AddedNothing\n");	
    return 0;
}
///=========================================================================/
///=========================================================================/
/**
 * @brief log2 - computes the logarithm base 2, i.e, log(N,2) of a number N
 * @param n
 * @return
 */
uint8_t log2_n(uint16_t n){
    int bits = 1;
    int b;
    for (b = 8; b >= 1; b/=2){
        int s = 1 << b;
        if (n >= s){
            n >>= b;
            bits += b;
        }
    }
    bits = bits - 1;
    return bits;
}
///=========================================================================/
///=========================================================================/
/**
 * @brief random_int
 * @param size_num
 * @return
 */
uint16_t random_int(uint16_t size_num){

    uint16_t rand_num = random_rand();

    if(size_num == 0){
                
        return 0;
    }

    uint16_t val = (65535 / size_num);
    
    if(rand_num < val){
        
        PRINTF("Hereee: %u\n", rand_num);
        return 0;
	
    }else{
      
        uint16_t k;
	
        for(k = 1; k < size_num; k++){
	  
            if (rand_num >= k*val && rand_num <= (k+1)*val){
	      
                return k;
            }
        }
    }
    return 0;
}
///=========================================================================/
///=========================================================================/
/**
 * @brief random_int
 * @param size_num
 * @return
 */
uint16_t randomint_between(uint16_t num_a, uint16_t num_b){

    ///Compute the difference so as we have the size of
    /// the sample...
    uint16_t size_num = num_b - num_a;

    ///generate a random value between 0 and 65535
    uint16_t rand_num = random_rand();

    if(size_num == 0){
        COOJA_DEBUG_PRINTF("DC1:%u\n",size_num);
        return 0;
    }

    uint16_t val = (65535 / size_num);
    if ( rand_num < val){
        return num_a;
    }else{
        uint16_t k;
        for(k = 1; k < size_num; k++){
            if (rand_num >= k*val && rand_num <= (k+1)*val){
                return (num_a + k);
            }
        }
    }
    return 0;
}
///=========================================================================/
///=========================================================================/
/**
 * @brief calc_sqrt: babylon method for calculating the square root.
 * @param num2comp : the value to compute the square root..
 * @return         : the square root value of 'num2comp'
 */
uint16_t calc_sqrt(const uint16_t num2comp){
    uint8_t i = 0;

    uint16_t x1 = 0, sqrt_val = 100;

    for(i = 0; i < 15; i++){
        sqrt_val = (sqrt_val + num2comp/sqrt_val)/2;

        if(i > 0 && x1 == sqrt_val){
            break;
        }else{
            x1 = sqrt_val;
        }
    }

    return  sqrt_val;
}
///=========================================================================/
///=========================================================================/
