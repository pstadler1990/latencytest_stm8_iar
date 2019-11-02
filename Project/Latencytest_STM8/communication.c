#include "communication.h"
#include "configuration.h"
#include <string.h>

void
init_com_interface(uint32_t baudrate) {
  /* Initialize UART communication interface with 81N on given baudrate (TX only) */
  UART1_DeInit();
  UART1_Init(baudrate, UART1_WORDLENGTH_8D,
                     UART1_STOPBITS_1, UART1_PARITY_NO,
                     UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);
			   
  /* Enable receive interrupt */
  UART1_ITConfig(UART1_IT_RXNE_OR, ENABLE);

  UART1_Cmd(ENABLE);
}

void
com_send(const char* str) {
  uint32_t len = strlen(str);
  for(uint32_t s = 0; s < len && s < COM_MAX_STRLEN; s++) {
    while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET) {
      nop();
    }
    UART1->DR = str[s];
  }
}