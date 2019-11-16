#ifndef _PDLIB_LL
#define _PDLIB_LL

//#define PART_LM4F120H5QR

#if defined(PART_LM4F120H5QR)
  void NRF24L01_SetInterface(unsigned long ulCEBase, unsigned long ulCEPin, unsigned long ulCEPeriph, unsigned long ulCSNBase, unsigned long ulCSNPin, unsigned long ulCSNPeriph, unsigned char ucSSIIndex);
  void NRF24L01_LL_ConfigureSPIInterface();
  unsigned char pdlibSPI_ReceiveDataBlocking();
  unsigned int pdlibSPI_ReceiveDataNonBlocking(char *pcData);
  #ifdef NRF24L01_CONF_INTERRUPT_PIN
    void NRF24L01_LL_InterruptInit(unsigned long ulIRQBase, unsigned long ulIRQPin, unsigned long ulIRQPeriph, unsigned long ulInterrupt);
  #endif
#else
  void NRF24L01_LL_ConfigureSPIInterface();
  #ifdef NRF24L01_CONF_INTERRUPT_PIN
    void NRF24L01_LL_InterruptInit();
  #endif
#endif

unsigned char NRF24L01_LL_TransferByte(unsigned char ucData);
void NRF24L01_LL_CEHigh();
void NRF24L01_LL_CELow();
void NRF24L01_LL_CSNHigh();
void NRF24L01_LL_CSNLow();


#endif //#ifndef _PDLIB_LL