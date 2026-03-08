#ifndef CORE_HARDWARE_CC_H_
#define CORE_HARDWARE_CC_H_

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

#define CC_PACKED_BEGIN
#define CC_PACKED_END
#define CC_PACKED __attribute__((packed))
#define CC_ALIGNED(n) __attribute__((aligned(n)))
#define CC_ASSERT(exp) assert(exp)
#define CC_STATIC_ASSERT(exp) _Static_assert(exp, "")
#define CC_DEPRECATED __attribute__((deprecated))

#endif
