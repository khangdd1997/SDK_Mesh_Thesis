#include "localization.h"

bool is_command_success(const char* str, const char* verify)
{
    
}

 uint32_t printRssiVector(int32_t * vector, uint32_t address)
{
    uint8_t i;
    uint32_t distance = 0;
    int32_t average = 0;
    for (i = 0; i < APP_RSSI_SAMPLE_NUMBER; i++) {
        average += vector[i];
        vector[i] = 0;
    }
    average /= (int32_t)APP_RSSI_SAMPLE_NUMBER;
    distance = (uint32_t)(pow(10, (RSSI_AT_1M - (average))/30.0) * 100.0);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Average RSSI of %d: %d\n", address, average);    

    return distance;
}



 bool is_address_in_buffer(uint32_t *buffer, uint32_t arr_size, uint32_t address) 
{    
    for (uint8_t i = 0; i < arr_size; i++)
    {
        if (buffer[i] == address)
        {
            return true;
        }
    }
    return false;
}

void add_address_to_buffer(uint32_t *buffer, uint32_t arr_size, uint32_t address) 
{
    for (uint8_t i = 0; i < arr_size; i++)
    {
        if (buffer[i] == 0)
        {
            buffer[i] = address;
                        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Add address: %d\n", buffer[i]);

            break;
        }
    }
}

Position getPosition(//uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint32_t x3, uint32_t y3
                        localize_node_info *node)
{
    // Check if there is any unassigned node

    int32_t x1 = node[0].x;
    int32_t y1 = node[0].y;
    int32_t x2 = node[1].x;
    int32_t y2 = node[1].y;
    int32_t x3 = node[2].x;
    int32_t y3 = node[2].x;
    int32_t r1 = node[0].distance;
    int32_t r2 = node[1].distance;
    int32_t r3 = node[2].distance;

    float a1, b1, c1, a2, b2, c2;
    
    a1 = -2*x1 + 2*x2;
    b1 = -2*y1 + 2*y2;
    c1 = (r1*r1 - r2*r2 - x1*x1 + x2*x2 - y1*y1 + y2*y2);
    a2 = -2*x2 + 2*x3;
    b2 = -2*y2 + 2*y3;
    c2 = (r2*r2 - r3*r3 - x2*x2 + x3*x3 - y2*y2 + y3*y3); 
    
    Position pos;
    pos.x = (uint32_t)((c1*b2 - c2*b1)/(a1*b2 - a2*b1));
    pos.y = (uint32_t)((c2*a1 - c1*a2)/(a1*b2 - a2*b1));

    return pos;
}

void assign_position(localize_node_info *nodeinfo)
{
    switch(nodeinfo->node_address) 
    {
        case 80:
            nodeinfo->x = x_80;
            nodeinfo->y = y_80;
            break;
        case 100:
            nodeinfo->x = x_100;
            nodeinfo->y = y_100;
            break;
        case 90:
            nodeinfo->x = x_90;
            nodeinfo->y = y_90;
            break;
    }
}