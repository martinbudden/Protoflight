#pragma once

#include <AHRS_MessageQueueBase.h>
#include <BlackboxMessageQueue.h>

//!!TODO: add Quaternion orientation to AHRS_MessageQueueBase
class BlackboxMessageQueueAHRS : public AHRS_MessageQueueBase {
public:
    explicit BlackboxMessageQueueAHRS(BlackboxMessageQueue& blackboxMessageQueue) :
        _blackboxMessageQueue(blackboxMessageQueue) {}
    virtual uint32_t append(uint32_t timeMicroseconds, const xyz_t& gyroRPS, const xyz_t& gyroRPS_unfiltered, const xyz_t& acc) override;
private:
    BlackboxMessageQueue _blackboxMessageQueue;
};
