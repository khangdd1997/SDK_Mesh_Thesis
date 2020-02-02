//#include "application_beacon.h"
//
//#define STATIC_AUTH_DATA        {0x6E, 0x6F, 0x72, 0x64, 0x69, 0x63, 0x5F, 0x65, 0x78, 0x61, 0x6D, 0x70, 0x6C, 0x65, 0x5F, 0x31}
//#define ADVERTISER_BUFFER_SIZE  (64)
//
///** Single advertiser instance. May periodically transmit one packet at a time. */
//static advertiser_t m_advertiser;
//
//static uint8_t      m_adv_buffer[ADVERTISER_BUFFER_SIZE];
//static bool         m_device_provisioned;
//
//
//static void rx_cb(const nrf_mesh_adv_packet_rx_data_t * p_rx_data)
//{
//    LEDS_OFF(BSP_LED_0_MASK);  /* @c LED_RGB_RED_MASK on pca10031 */
//    char msg[128];
//    (void) sprintf(msg, "RX [@%u]: RSSI: %3d ADV TYPE: %x ADDR: [%02x:%02x:%02x:%02x:%02x:%02x]",
//                   p_rx_data->p_metadata->params.scanner.timestamp,
//                   p_rx_data->p_metadata->params.scanner.rssi,
//                   p_rx_data->adv_type,
//                   p_rx_data->p_metadata->params.scanner.adv_addr.addr[0],
//                   p_rx_data->p_metadata->params.scanner.adv_addr.addr[1],
//                   p_rx_data->p_metadata->params.scanner.adv_addr.addr[2],
//                   p_rx_data->p_metadata->params.scanner.adv_addr.addr[3],
//                   p_rx_data->p_metadata->params.scanner.adv_addr.addr[4],
//                   p_rx_data->p_metadata->params.scanner.adv_addr.addr[5]);
//    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, msg, p_rx_data->p_payload, p_rx_data->length);
//    LEDS_ON(BSP_LED_0_MASK);  /* @c LED_RGB_RED_MASK on pca10031 */
//}
//
//void adv_init(void)
//{
//    /* Start listening for incoming packets */
//    nrf_mesh_rx_cb_set(rx_cb);
//
//    advertiser_instance_init(&m_advertiser, NULL, m_adv_buffer, ADVERTISER_BUFFER_SIZE);
//}
//
//void adv_start(void)
//{
//    /* Let scanner accept Complete Local Name AD Type. */
//    bearer_adtype_add(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME);
//
//    advertiser_enable(&m_advertiser);
//    static const uint8_t adv_data[] =
//    {
//        0x11, /* AD data length (including type, but not itself) */
//        BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, /* AD data type (Complete local name) */
//        'N',  /* AD data payload (Name of device) */
//        'o',
//        'r',
//        'd',
//        'i',
//        'c',
//        ' ',
//        'S',
//        'e',
//        'm',
//        'i',
//        ' ',
//        'M',
//        'e',
//        's',
//        'h'
//    };
//
//    /* Allocate packet */
//    adv_packet_t * p_packet = advertiser_packet_alloc(&m_advertiser, sizeof(adv_data));
//    if (p_packet)
//    {
//        /* Construct packet contents */
//        memcpy(p_packet->packet.payload, adv_data, sizeof(adv_data));
//        /* Repeat forever */
//        p_packet->config.repeats = ADVERTISER_REPEAT_INFINITE;
//
//        advertiser_packet_send(&m_advertiser, p_packet);
//    }
//
//}

