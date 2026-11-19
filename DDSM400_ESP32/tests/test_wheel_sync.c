#include "wheel_sync.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    wheel_sync_state sync;
    wheel_sync_reset(&sync);
    assert(sync.id2_scale == 1.0f && sync.id1_scale == 1.0f);

    /* ID2 is intrinsically 25% faster at the same input. */
    for (unsigned i = 0; i < 40; ++i) {
        assert(wheel_sync_update(&sync, 20.0f * sync.id2_scale,
                                16.0f * sync.id1_scale));
    }
    assert(fabsf(20.0f * sync.id2_scale - 16.0f * sync.id1_scale) < 0.9f);
    assert(sync.id2_scale < 1.0f && sync.id1_scale <= 1.0f);

    wheel_sync_reset(&sync);
    for (unsigned i = 0; i < 40; ++i) {
        assert(wheel_sync_update(&sync, 16.0f * sync.id2_scale,
                                20.0f * sync.id1_scale));
    }
    assert(fabsf(16.0f * sync.id2_scale - 20.0f * sync.id1_scale) < 0.9f);
    assert(sync.id1_scale < 1.0f && sync.id2_scale <= 1.0f);

    wheel_sync_reset(&sync);
    assert(wheel_sync_update(&sync, 20.0f, 0.0f));
    assert(!wheel_sync_update(&sync, 20.0f, 0.0f));

    puts("wheel sync tests: PASS");
    return 0;
}
