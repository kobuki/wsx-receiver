#ifndef RFM69WSX_H
#define RFM69WSX_H

#include <RFM69.h>
#include <RFM69wsx.h>

class RFM69wsx : public RFM69 {
public:
    static int16_t FEI;

    bool initialize(uint32_t freq);
    void interruptHandler();
    bool receiveDone();
};

#endif // RFM69WSX_H