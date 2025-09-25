#pragma once

#include <AHRS_MessageQueueBase.h>
#include <BlackboxMessageQueue.h>

class BlackboxMessageQueueAHRS : public AHRS_MessageQueueBase {
public:
    explicit BlackboxMessageQueueAHRS(BlackboxMessageQueue& blackboxMessageQueue) :
        _blackboxMessageQueue(blackboxMessageQueue) {}
    virtual uint32_t append(uint32_t timeMicroseconds, const xyz_t& gyroRPS, const xyz_t& gyroRPS_unfiltered, const xyz_t& acc, const Quaternion& orientation) override;
private:
    BlackboxMessageQueue _blackboxMessageQueue;
};
