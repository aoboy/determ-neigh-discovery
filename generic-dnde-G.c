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
#define CDEBUG 0
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
#define TS                  (25*ONE_MSEC)
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
static const uint8_t periods_vec [] = {133, 133, 167, 150, 133, 100}; //x100
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

///=========================================================================/
///=========================================================================/
static void on(void){
  if(radio_is_on_flag == 0) {
    radio_is_on_flag = 1;
    NETSTACK_RADIO.on();
  }
}
///=========================================================================/
///=========================================================================/
/**
 * @brief off
 */
static void off(void){
  if(radio_is_on_flag == 1) {
    radio_is_on_flag = 0;
    NETSTACK_RADIO.off();
  }
}
///=========================================================================/
///=========================================================================/
/**
 * @brief divide_round
 * @param n
 * @param d
 * @return
 */
static uint16_t divide_round(const uint16_t n, const uint16_t d){
 return ((n+d/2)/d);
}
///=========================================================================/
///=========================================================================/
/**
 * @brief energy_usage
 * @return
 */
uint16_t energy_usage(){
    return energy_counter + (extra_probe_counter+1)/2 - extra_probe_counter;
}

///=========================================================================/
///=========================================================================/
uint8_t energy_usage_period(){
  return energy_per_period + (extra_probe_per_period + 1)/2;
}
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
  
  random_slot = 1 +  random_rand()%period_length;
  
  Tup_bound = (period_length*period_length)/4; 
  
  //random_slot = random_slot + random_rand()%Tup_bound ;
  
  /*if (rimeaddr_node_addr.u8[0] < 130){
    random_slot = random_slot + random_rand()%period_length;
  }else{
    random_slot = random_slot + Tup_bound + random_rand()%period_length;
  }*/
  //enable random offset wait flag..
  rand_offset_wait_flag = 1;
  node_slots_offset     = random_slot;
  slot_upperBound       = 10000-node_slots_offset;
  
  /*uint16_t random_slot = 1 + random_int(T_period_lenghth);
  node_slots_offset    = random_slot;
  
  slot_counter_upperB = T_period_lenghth;
  slot_counter_upperB = random_slot + slot_counter_upperB;
  
  rtimer_clock_t startTime = RTIMER_NOW() + random_slot*TS;*/
  
  rtimer_clock_t startTime = RTIMER_NOW() + 2;

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
  
    /*int k = 0, k1 = random_int(100);
  
    for(k = 0; k < 100; k++){
        random_rand();
    }*/

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
