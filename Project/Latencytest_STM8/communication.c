#include "communication.h"
#include "configuration.h"
#include <string.h>

void
init_com_interface(uint32_t baudrate) {
    /* Initialize UART communication interface with 81N on given baudrate (TX only) */
    UART2_DeInit();
    UART2_Init(baudrate, 
        UART2_WORDLENGTH_8D, 
        UART2_STOPBITS_1, 
        UART2_PARITY_NO, 
        UART2_SYNCMODE_CLOCK_DISABLE, 
        UART2_MODE_TXRX_ENABLE);
			   
  /* Enable receive interrupt */
  UART2_ITConfig(UART2_IT_RXNE_OR, ENABLE);
  UART2_ITConfig(UART2_IT_RXNE_OR, ENABLE);

  UART2_Cmd(ENABLE);
}

void
com_send(const char* str) {
  uint32_t len = strlen(str);
  for(uint32_t s = 0; s < len && s < COM_MAX_STRLEN; s++) {
    
    UART2->DR = (unsigned char) str[s];

    while (UART2_GetFlagStatus(UART2_FLAG_TXE) == RESET) {
      nop();
    }
  }
}