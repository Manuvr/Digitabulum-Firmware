/*
File:   ManuLegendPipe.h
Author: J. Ian Lindsay
Date:   2017.08.18

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

#ifndef __DIGITABULUM_MANU_LEGEND_PIPE_H_
#define __DIGITABULUM_MANU_LEGEND_PIPE_H_

#include <Kernel.h>
#include "ManuLegend.h"

// Forward dec
class SensorFrame;

/*
* Flags for Legend state and control (not data).
*/
#define  LEGENDPIPE_FLAGS_LEGEND_ACTIVE       0x01   // If set, this ManuLegend will be included in the sum.
#define  LEGENDPIPE_FLAGS_LEGEND_SATISFIED    0x02   // If set, this ManuLegend is getting everything it needs.
#define  LEGENDPIPE_FLAGS_LEGEND_STABLE       0x04   // This ManuLegend is stable and ready for operation.
#define  LEGENDPIPE_FLAGS_LEGEND_CHANGE_PEND  0x08   // We have a pending change to the ManuLegend from outside.
#define  LEGENDPIPE_FLAGS_LEGEND_CHANGE_SENT  0x10   // Change notice has been sent to counterparty.
#define  LEGENDPIPE_FLAGS_LEGEND_DECOUPLE_SEQ 0x80   // If set, frame seq will be independently-tracked here.

#define  LEGENDPIPE_FLAGS_SHOULD_ACCEPT_MASK  (LEGENDPIPE_FLAGS_LEGEND_STABLE | LEGENDPIPE_FLAGS_LEGEND_ACTIVE)

/*
* Supported options for encoding Frames.
*/
enum class ManuEncoding : uint8_t {
  LOG    = 0,
  CBOR   = 1,
  OSC    = 2,
  MANUVR = 3
};


/*
* Class for connecting Manu results to the outside world.
*/
class ManuLegendPipe : public ManuLegend, public BufferPipe {
  public:
    ManuLegendPipe(ManuEncoding);
    ManuLegendPipe() : ManuLegendPipe(ManuEncoding::LOG) {};
    ~ManuLegendPipe();

    /* Override from BufferPipe. */

    void printDebug(StringBuilder*);

    int8_t offer(SensorFrame*);
    void broadcast_legend();

    /* Used to enable or disable the Legend's activity. */
    inline bool active() {              return (_flags & LEGENDPIPE_FLAGS_LEGEND_ACTIVE);          };
    inline void active(bool en) {       _flags = (en) ? (_flags | LEGENDPIPE_FLAGS_LEGEND_ACTIVE) : (_flags & ~(LEGENDPIPE_FLAGS_LEGEND_ACTIVE));  };

    inline bool satisfied() {           return (_flags & LEGENDPIPE_FLAGS_LEGEND_SATISFIED);       };
    inline bool stable() {              return (_flags & LEGENDPIPE_FLAGS_LEGEND_STABLE);          };
    inline bool changeSent() {          return (_flags & LEGENDPIPE_FLAGS_LEGEND_CHANGE_SENT);     };
    inline bool changePending() {       return (_flags & LEGENDPIPE_FLAGS_LEGEND_CHANGE_PEND);     };

    inline bool decoupleSeq() {         return (_flags & LEGENDPIPE_FLAGS_LEGEND_DECOUPLE_SEQ);     };
    inline void decoupleSeq(bool en) {  _flags = (en) ? (_flags | LEGENDPIPE_FLAGS_LEGEND_DECOUPLE_SEQ) : (_flags & ~(LEGENDPIPE_FLAGS_LEGEND_DECOUPLE_SEQ));  };

    inline ManuEncoding encoding() {        return _encoding;   };
    inline void encoding(ManuEncoding e) {  _encoding = e;      };


    static const char* encoding_label(ManuEncoding);


  protected:
    const char* pipeName();


  private:
    // TODO: We are ultimately going to form this class into a BufferPipe once that class
    //   has some better asynchronicity and self-reporting properties.
    EventReceiver* _owner  = nullptr;  // The owner of this ManuLegend.

    uint32_t _local_seq    = 0;
    uint32_t _ms_last_send = 0;
    uint32_t _ms_interval  = 100;
    uint8_t  _flags        = 0;
    ManuEncoding _encoding = ManuEncoding::MANUVR;

    inline void satisfied(bool en) {  _flags = (en) ? (_flags | LEGENDPIPE_FLAGS_LEGEND_SATISFIED) : (_flags & ~(LEGENDPIPE_FLAGS_LEGEND_SATISFIED)); };
    inline void stable(bool en) {     _flags = (en) ? (_flags | LEGENDPIPE_FLAGS_LEGEND_STABLE)    : (_flags & ~(LEGENDPIPE_FLAGS_LEGEND_STABLE));    };
    inline void changeSent(bool en) {     _flags = (en) ? (_flags | LEGENDPIPE_FLAGS_LEGEND_CHANGE_SENT) : (_flags & ~(LEGENDPIPE_FLAGS_LEGEND_CHANGE_SENT));  };
    inline void changePending(bool en) {  _flags = (en) ? (_flags | LEGENDPIPE_FLAGS_LEGEND_CHANGE_PEND) : (_flags & ~(LEGENDPIPE_FLAGS_LEGEND_CHANGE_PEND));  };

    inline bool should_accept() {     return (_flags & LEGENDPIPE_FLAGS_SHOULD_ACCEPT_MASK);  };
};

#endif  // __DIGITABULUM_MANU_LEGEND_PIPE_H_
