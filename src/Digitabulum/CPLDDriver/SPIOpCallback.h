#ifndef __SPI_BUS_OP_CALLBACK_H__
#define __SPI_BUS_OP_CALLBACK_H__

#include "SPIBusOp.h"

/*
* This is an interface class that implements a callback path from the SPI ISR back to a
*   device that asked for the operation.
* If a device wants to put operations into the SPI queue, it must either implement this
*   interface, or delegate its callback duties to a class that does.
*/
class SPIOpCallback {
  public:
    // The SPI driver will call this fxn when the bus op finishes.
    virtual int8_t spi_op_callback(SPIBusOp*) =0;
    
    /* 
    */
    virtual int8_t queue_spi_job(SPIBusOp*) =0;

};


#endif

