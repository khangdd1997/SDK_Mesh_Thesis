static uint32_t printRssiVector(int32_t * vector, uint32_t address)
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

static void rssiGET_timer()
{
        rssiGET_set_params_t rssiGET_params;
        model_transition_t transition_params;
        transition_params.delay_ms = APP_CONFIG_ONOFF_DELAY_MS;
        transition_params.transition_time_ms = APP_CONFIG_ONOFF_TRANSITION_TIME_MS;
        uint8_t tid = 0;
    
        rssiGET_params.reverse = 0;
        rssiGET_params.tid = tid++;
        (void)rssiGET_client_set_unack(&m_clients[1], &rssiGET_params, &transition_params, 0); 
}

static bool is_address_in_buffer(uint32_t *buffer, uint32_t arr_size, uint32_t address) 
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

static void add_address_to_buffer(uint32_t *buffer, uint32_t arr_size, uint32_t address) 
{
    for (uint8_t i = 0; i < arr_size; i++)
    {
        if (buffer[i] == 0)
        {
            buffer[i] = address;
            break;
        }
    }
}