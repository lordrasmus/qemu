#ifndef IA64_TARGET_SYSCALL_H
#define IA64_TARGET_SYSCALL_H

#define UNAME_MACHINE "ia64"
#define UNAME_MINIMUM_RELEASE "2.6.32"

/* the offset of mmap2 is in units of the 16 KB page */
#define MMAP_SHIFT 14

#define TARGET_MCL_CURRENT 1
#define TARGET_MCL_FUTURE  2
#define TARGET_MCL_ONFAULT 4

#endif
