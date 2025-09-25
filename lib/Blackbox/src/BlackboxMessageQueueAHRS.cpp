#include "BlackboxMessageQueueAHRS.h"
#include <BlackboxMessageQueue.h>


uint32_t BlackboxMessageQueueAHRS::append(uint32_t timeMicroseconds, const xyz_t& gyroRPS, const xyz_t& gyroRPS_unfiltered, const xyz_t& acc, const Quaternion& orientation)
{
    struct BlackboxMessageQueue::queue_item_t queueItem {
        timeMicroseconds,
        gyroRPS,
        gyroRPS_unfiltered,
        acc,
        orientation
    };

    return _blackboxMessageQueue.SEND_IF_NOT_FULL(queueItem); // cppcheck-suppress knownConditionTrueFalse
}
