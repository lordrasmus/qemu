#ifndef IA64_TARGET_SIGNAL_H
#define IA64_TARGET_SIGNAL_H

#include "../generic/signal.h"

/* arch/ia64/include/uapi/asm/signal.h, the typo included */
#undef TARGET_MINSIGSTKSZ
#define TARGET_MINSIGSTKSZ      131027

#define TARGET_ARCH_HAS_SIGTRAMP_PAGE 1

#endif /* IA64_TARGET_SIGNAL_H */
