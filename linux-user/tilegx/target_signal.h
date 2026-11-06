#ifndef TILEGX_TARGET_SIGNAL_H
#define TILEGX_TARGET_SIGNAL_H

#include "../generic/signal.h"

/* arch/tile/include/uapi/asm/signal.h */
#define TARGET_SA_RESTORER      0x04000000

#define TARGET_ARCH_HAS_SIGTRAMP_PAGE 1

#endif /* TILEGX_TARGET_SIGNAL_H */
