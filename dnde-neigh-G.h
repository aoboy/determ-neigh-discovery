/**
 * ------------------------------------------------------------------------
 * KTH Royal Institute of Technology
 *   School of Electrical Engineering
 *      Dept. of Automatic Control
 * ------------------------------------------------------------------------
 * @author: António Gonga < gonga@ee.kth.se>, PhD candidate
 * @date:   March 9th 2015
 * @file:   dnde-neigh.h 
 * @brief:  header file contains functions prototypes for manipulating the 
 *          neighbors list
 * ------------------------------------------------------------------------ 
 * @info: Auxiliary function prototypes file for a generic deterministic 
 *        neighbor discovery with epidemics.
 */

#ifndef DNDE_NEIGH_H
#define DNDE_NEIGH_H

#include "contiki.h"
#include "lib/random.h"

#define MAX_REPEAT   100 //25
///=========================================================================/
///=========================================================================/
struct data_item_t{
    uint8_t node_id; //this allows for 256IDS
    uint8_t dc_hopc; //duty_cycle.hopcount
    uint8_t offset;  //the offset of a specific neighbor..
    uint8_t period;
};

struct data_item_tmp{
    uint8_t node_id; //this allows for 256IDS
    uint8_t dc_hopc; //duty_cycle.hopcount
    uint8_t offset;  //the offset of a specific neighbor..
    //uint8_t period;
};

#define DATA_ITEM_LEN sizeof(struct data_item_t)
///=========================================================================/
///=========================================================================/
#if CONF_NETWORK_SIZE != 0
#define MAX_NETWORK_SIZE  (sizeof(struct data_item_t))*CONF_NETWORK_SIZE
static uint8_t num_neighbors = CONF_NETWORK_SIZE;
#else
static uint8_t num_neighbors = 2;
#define MAX_NETWORK_SIZE  2
#endif //CONF_NETWORK_CLIQUE_SIZE != 0
///=========================================================================/
///=========================================================================/
#define MAX_TRIES_H2_NEIGH  2
///=========================================================================/
///=========================================================================/
struct nodelist_item{
    struct nodelist_item *next;
    uint8_t  node_id;    //nodeID of neighbor j
    uint8_t  hopcount;   //0000.yyyy = {hopcount}
    uint8_t  period;     //period of node and related to its j
    uint8_t  j_factor;   //used to compute offset to any neighbor j
    uint8_t  max_txhop2; //max number of attempts to a hop 2 neighbor..
    int16_t  offset;     //offset to neighbor j
    uint8_t  offsetj;    // offset to send to neighbors
    uint16_t t_anchor;   //anchor time when overlap occured
    uint16_t tknown;     //time when neighbor j became known by node i
    uint16_t tconfirmed; //time when neighbor j was confirmed by node i
};
///#if CONF_NETWORK_SIZE != 0 && 
#if CONF_NETWORK_SIZE < 25
#define NETWORK_PAYLOAD_SIZE CONF_NETWORK_SIZE*DATA_ITEM_LEN
#else  //CONF_NETWORK_SIZE
#define NETWORK_PAYLOAD_SIZE 25*DATA_ITEM_LEN
#endif //CONF_NETWORK_SIZE
///=========================================================================/
///=========================================================================/
typedef struct{
    uint8_t type;
    uint8_t offset;
    uint8_t period;
    uint8_t data[NETWORK_PAYLOAD_SIZE];
}data_packet_t;
///=========================================================================/
///=========================================================================/
#define DATAPKT_HDR_LEN (3*sizeof(uint8_t))
#define DATA_PACKET_SIZE sizeof(data_packet_t)
///=========================================================================/
///=========================================================================/
enum{
  SYNCH_PKT    = 0,
  DATA_PKT     = 0x01,
  ANCHOR_PKT   = (0x80 | DATA_PKT),
  PROBE_PKT    = (0x40 | DATA_PKT),
  HOPC_MASK    = 0x0F,
  DCYCLE_MASK  = 0xF0  
};
///=========================================================================/
///=========================================================================/
uint16_t get_num_iterations();
uint16_t get_discovery_time();
uint16_t get_anchor_time();
uint8_t  get_node_period();
uint8_t  get_node_dutycycle();
///=========================================================================/
///=========================================================================/
void neighs_init();

struct nodelist_item *neighs_get(uint8_t nodeid);

struct nodelist_item *neighs_getlist();

void    neighs_add_itself();

uint8_t neighs_num_nodes();

uint8_t neighs_all_found();

void    neighs_remove(uint8_t nodeid);

void    neighs_flush_all();

///=========================================================================/
///=========================================================================/
uint8_t  neighs_xhops(uint8_t xhops);

uint8_t  neighs_h2_indirect();

void     neighs_jfactor_update();

uint16_t time_neighbor_anchor(struct nodelist_item *n_item);

uint8_t  isthere_anchor(uint8_t hops_away, uint16_t curr_time);

///=========================================================================/
///=========================================================================/

uint8_t neighs_register(data_packet_t *pkt_hdr, int, uint8_t);

uint8_t neighs_add2payload(uint8_t *data_ptr, uint8_t, uint8_t);
///=========================================================================/
///=========================================================================/

uint8_t  log2_n(uint16_t n);

uint16_t random_int(uint16_t size_num);

uint16_t randomint_between(uint16_t num_a, uint16_t num_b);

uint16_t calc_sqrt(const uint16_t );
///=========================================================================/
///=========================================================================/

void start_protocol(uint8_t );

///=========================================================================/
///=========================================================================/
#if CONF_CHANNEL_SIZE != 0
#define CHANNEL_SIZE CONF_CHANNEL_SIZE
static  uint8_t num_channels = CHANNEL_SIZE;
#else   //CONF_CHANNEL_POOL_SIZE
#define CHANNEL_SIZE    1
static  uint8_t num_channels = CHANNEL_SIZE;
#endif //CONF_CHANNEL_POOL_SIZE
///=========================================================================/
///=========================================================================/
#if HOPCOUNT_FILTER_NDISC != 0
#define MAX_HOPCOUNT HOPCOUNT_FILTER_NDISC
/*    #if CHANNEL_POOL_SIZE == 1
        #define MAX_HOPCOUNT  1
    #else //CHANNEL_POOL_SIZE == 1
        #define MAX_HOPCOUNT HOPCOUNT_FILTER_NDISC
    #endif //CHANNEL_POOL_SIZE == 1
*/
#else //HOPCOUNT_FILTER_NDISC != 0
    #define MAX_HOPCOUNT 15
#endif //CONF_MAX_HOPCOUNT != 0
///=========================================================================/
///=========================================================================/

#endif //DNDE_NEIGH_H

