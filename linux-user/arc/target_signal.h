#ifndef ARC_TARGET_SIGNAL_H
#define ARC_TARGET_SIGNAL_H

/* arch/arc/include/uapi/asm/signal.h */
#define TARGET_SA_RESTORER 0x04000000

#include "../generic/signal.h"

/* The handler returns through the libc's sa_restorer. */
#define TARGET_ARCH_HAS_SIGTRAMP_PAGE 0

#endif /* ARC_TARGET_SIGNAL_H */
