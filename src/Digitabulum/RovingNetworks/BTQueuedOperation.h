/*
File:   BTQueuedOperation.h
Author: J. Ian Lindsay
Date:   2014.05.22

Copyright 2016 Manuvr, Inc

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.


*/

#ifndef __DIGITABULUM_BT_Q_OP_H__
#define __DIGITABULUM_BT_Q_OP_H__

#include <Kernel.h>
#include <Drivers/BusQueue/BusQueue.h>
#include <DataStructures/StringBuilder.h>

// Forward declarations...
class RNBase;

/*
* This is the class that represents an item in the work queue.
*/
class BTQueuedOperation : public BusOp {
  public:
    StringBuilder data;       // Might need a raw buffer on the way to DMA...

    BTQueuedOperation();
    BTQueuedOperation(BusOpcode nu_op);
    BTQueuedOperation(BusOpcode nu_op, BusOpCallback* requester);
    BTQueuedOperation(BusOpcode nu_op, StringBuilder* nu_data);
    /* Specialized constructor for direct buffer spec. */
    BTQueuedOperation(BusOpcode nu_op, unsigned char *nu_data, uint16_t nu_len);

    virtual ~BTQueuedOperation();

    /* Mandatory overrides from the BusOp interface... */
    //XferFault advance();
    XferFault begin();
    void wipe();
    void printDebug(StringBuilder*);

    int8_t markComplete();
    /**
    * This will mark the bus operation complete with a given error code.
    * Overriden for simplicity. Marks the operation with failure code NO_REASON.
    *
    * @return 0 on success. Non-zero on failure.
    */
    inline int8_t abort() {    return abort(XferFault::NO_REASON); }
    int8_t abort(XferFault);

    void set_data(BusOpcode, StringBuilder*);


  private:
};


#endif  // __DIGITABULUM_BT_Q_OP_H__
