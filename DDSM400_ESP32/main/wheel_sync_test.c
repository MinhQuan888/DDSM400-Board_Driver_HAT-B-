#include "wheel_sync.h"

#include <assert.h>

int main(void)
{
    wheel_sync_state equal = {0};
    wheel_sync_reset(&equal);
    for (int i = 0; i < 8; ++i) assert(wheel_sync_update(&equal, 20.0f, 20.0f));
    assert(equal.id2_scale == 1.0f && equal.id1_scale == 1.0f);

    wheel_sync_state uneven = {0};
    wheel_sync_reset(&uneven);
    for (int i = 0; i < 5; ++i) assert(wheel_sync_update(&uneven, 25.0f, 15.0f));
    assert(uneven.id2_scale < 1.0f && uneven.id1_scale == 1.0f);
    assert(uneven.id2_scale >= 0.70f && uneven.id1_scale <= 1.0f);

    wheel_sync_state stalled = {0};
    wheel_sync_reset(&stalled);
    assert(wheel_sync_update(&stalled, 20.0f, 0.0f));
    assert(!wheel_sync_update(&stalled, 20.0f, 0.0f));

    wheel_sync_state reversed = {0};
    wheel_sync_reset(&reversed);
    assert(!wheel_sync_update(&reversed, 20.0f, -4.0f));

    wheel_sync_state idle = {0};
    wheel_sync_reset(&idle);
    for (int i = 0; i < 3; ++i) assert(wheel_sync_update(&idle, 0.0f, 0.0f));
    assert(!wheel_sync_update(&idle, 0.0f, 0.0f));
    return 0;
}
