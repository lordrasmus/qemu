/*
 * IA-64 virtual machine: what the board and its EFI boot services share
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef HW_IA64_VIRT_H
#define HW_IA64_VIRT_H

#include "cpu.h"

/* Physical layout */
/*
 * The firmware lives outside of RAM: arch/ia64/mm/contig.c registers all
 * of [0, top of RAM) and would hand out firmware pages inside it.
 */
#define FW_BASE         0x000000fe00000000ull   /* firmware, 1 MB */
#define FW_PAL          (FW_BASE + 0x00000)     /* EFI_PAL_CODE, 64 KB */
#define FW_CODE         (FW_BASE + 0x10000)     /* runtime code */
#define FW_FDESC        (FW_BASE + 0x18000)     /* function descriptors */
#define FW_EFI          (FW_BASE + 0x20000)     /* runtime data */
#define FW_SAL          (FW_BASE + 0x21000)
/*
 * The ACPI tables live below 4 GB (Windows' acpi.sys of 2001 follows the
 * 32-bit pointers only), outside RAM like the rest of the firmware.
 */
#define FW_ACPI         0xfeb00000ull
#define FW_ACPI_SIZE    0x10000
#define FW_DATA_END     (FW_BASE + 0x30000)
#define FW_BOOTPARAM    (FW_BASE + 0x30000)     /* loader data */
#define FW_CMDLINE      (FW_BASE + 0x31000)
#define FW_MEMMAP       (FW_BASE + 0x32000)
#define FW_IVT          (FW_BASE + 0x40000)     /* 32 KB, see efi.c */
#define FW_END          (FW_BASE + 0x100000)

/* The legacy area of a PC, kept for the x86 emulator of Windows' HAL */
#define VGA_WINDOW_BASE 0xa0000
#define VGA_WINDOW_SIZE 0x20000
#define VGA_ROM_BASE    0xc0000
#define VGA_ROM_SIZE    0x10000
#define VGA_STUB        0xcf100                 /* real mode cf10:0000 */

/* EFI system table and its neighbours in the runtime data */
#define FW_SYSTAB       (FW_EFI)
#define FW_VENDOR       (FW_EFI + 0x100)
#define FW_RUNTIME      (FW_EFI + 0x200)
#define FW_CONFIG       (FW_EFI + 0x300)        /* up to 128 entries */
#define FW_CONFIG_MAX   128
#define FW_FPSWA        (FW_EFI + 0xf00)        /* the FPSWA interface */

/* Firmware call numbers: break.i 0x1c0000 + n, n < 256 */
enum {
    FW_CALL_PAL = 0,
    FW_CALL_SAL = 1,
    FW_CALL_EFI = 2,    /* + runtime service index */
    FW_CALL_BOOT = 16,  /* reset entry of the EFI boot manager */
    FW_CALL_IMAGE_RETURN = 17,
    FW_CALL_BS = 18,    /* + boot service index */
    FW_CALL_PROTO = 64, /* protocol functions, see efi.c */
    FW_CALL_IVT = 255,  /* every vector of the firmware's IVT */
};

/* The stub and the function descriptor of firmware call n */
#define FW_STUB(n)      (FW_CODE + 16 * (n))
#define FW_FD(n)        (FW_FDESC + 16 * (n))

enum {
    EFI_GET_TIME, EFI_SET_TIME, EFI_GET_WAKEUP_TIME, EFI_SET_WAKEUP_TIME,
    EFI_SET_VIRTUAL_ADDRESS_MAP, EFI_CONVERT_POINTER, EFI_GET_VARIABLE,
    EFI_GET_NEXT_VARIABLE, EFI_SET_VARIABLE, EFI_GET_NEXT_HIGH_MONO_COUNT,
    EFI_RESET_SYSTEM, EFI_UPDATE_CAPSULE, EFI_QUERY_CAPSULE_CAPS,
    EFI_QUERY_VARIABLE_INFO,
    EFI_NUM_SERVICES
};

#define EFI_ERR(n)              (0x8000000000000000ull | (n))
#define EFI_SUCCESS             0
#define EFI_LOAD_ERROR          EFI_ERR(1)
#define EFI_INVALID_PARAMETER   EFI_ERR(2)
#define EFI_UNSUPPORTED         EFI_ERR(3)
#define EFI_BAD_BUFFER_SIZE     EFI_ERR(4)
#define EFI_BUFFER_TOO_SMALL    EFI_ERR(5)
#define EFI_NOT_READY           EFI_ERR(6)
#define EFI_DEVICE_ERROR        EFI_ERR(7)
#define EFI_WRITE_PROTECTED     EFI_ERR(8)
#define EFI_OUT_OF_RESOURCES    EFI_ERR(9)
#define EFI_VOLUME_CORRUPTED    EFI_ERR(10)
#define EFI_NO_MEDIA            EFI_ERR(12)
#define EFI_MEDIA_CHANGED       EFI_ERR(13)
#define EFI_NOT_FOUND           EFI_ERR(14)
#define EFI_ACCESS_DENIED       EFI_ERR(15)
#define EFI_ABORTED             EFI_ERR(21)

/* EFI memory types and attributes */
enum {
    EFI_RESERVED = 0, EFI_LOADER_CODE = 1, EFI_LOADER_DATA = 2,
    EFI_BS_CODE = 3, EFI_BS_DATA = 4, EFI_RUNTIME_CODE = 5,
    EFI_RUNTIME_DATA = 6, EFI_CONVENTIONAL = 7, EFI_UNUSABLE = 8,
    EFI_ACPI_RECLAIM = 9, EFI_ACPI_NVS = 10, EFI_MMIO = 11,
    EFI_MMIO_PORT = 12, EFI_PAL_CODE = 13, EFI_MAX_MEMORY_TYPE = 14,
};
#define EFI_MEM_UC          (1ull << 0)
#define EFI_MEM_WC          (1ull << 1)
#define EFI_MEM_WT          (1ull << 2)
#define EFI_MEM_WB          (1ull << 3)
#define EFI_MEM_RUNTIME     (1ull << 63)

typedef struct MemDesc {
    uint32_t type;
    uint64_t start, end, attr;
} MemDesc;

/* virt.c */
int virt_fw_memmap(MemDesc *md);
void virt_com1_init(void);
int virt_com1_getc(void);
void virt_pci_assign(void);
hwaddr fw_pa(CPUIA64State *env, uint64_t va, MMUAccessType type);
void fw_ret(CPUIA64State *env, uint64_t r8, uint64_t r9, uint64_t r10,
            uint64_t r11);

/* efi.c: boot services and variables */
void efi_boot_call(CPUIA64State *env, unsigned n);
uint64_t efi_get_variable(CPUIA64State *env);
uint64_t efi_get_next_variable(CPUIA64State *env);
uint64_t efi_set_variable(CPUIA64State *env);
void efi_virtual_address_change(uint64_t phys_start, uint64_t virt_start);
bool efi_svam_begin(CPUIA64State *env, uint64_t map, uint64_t size,
                    uint64_t dsize);
uint64_t efi_convert_pointer(CPUIA64State *env);
void virt_svam_complete(CPUIA64State *env);

#endif
