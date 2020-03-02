// -------- User Header file ----------- //

#include "stdint.h"
#include "stdlib.h"
#include "stdbool.h"
#include "log.h"
#include "math.h"

#define APP_LOCALIZE_NODE_NUMBER        3
#define APP_RSSI_SAMPLE_NUMBER          20
#define SAMPLE_RATE                     150  
#define RSSI_AT_1M                      -67

#define x_90                            370
#define y_90                            0
#define x_100                           50
#define y_100                           0 
#define x_80                            180
#define y_80                            660

//uint32_t address_pool[3] = {80, 100, 90};

typedef struct
{
    uint32_t node_address;
    uint32_t x;
    uint32_t y;
    uint32_t distance;
} localize_node_info;

typedef struct 
{
    uint32_t x;
    uint32_t y;
} Position;

uint32_t printRssiVector(int32_t * vector, uint32_t address);

void rssiGET_timer();

bool is_address_in_buffer(uint32_t *buffer, uint32_t arr_size, uint32_t address);

void add_address_to_buffer(uint32_t *buffer, uint32_t arr_size, uint32_t address);
  
Position getPosition(//uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint32_t x3, uint32_t y3
                        localize_node_info *node);

void assign_position(localize_node_info *nodeinfo);

bool is_command_success(const char* str, const char* verify);