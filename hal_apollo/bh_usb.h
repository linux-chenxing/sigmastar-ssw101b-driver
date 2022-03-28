
#ifndef ATBM_APOLLO_BH_USB_H
#define ATBM_APOLLO_BH_USB_H


#define USB_BLOCK_SIZE (512)
#ifdef CONFIG_USB_AGGR_URB_TX
#define RX_BUFFER_SIZE 12*1024
#else
#define RX_BUFFER_SIZE 4000//(1024*4-100)
#endif
#define RX_LONG_BUFFER_SIZE 4000

#define TX_LONG_BUFFER_SIZE (1024*4)
#ifdef CONFIG_USE_DMA_ADDR_BUFFER
#define TX_BUFFER_SIZE 2048
#else
#define TX_BUFFER_SIZE 1680
#endif //CONFIG_USE_DMA_ADDR_BUFFER

#endif//ATBM_APOLLO_BH_USB_H
