/*
 * IA-64 virtual machine: EFI 1.10 boot services
 *
 * Enough of an EFI firmware to start an OS loader from a CD or a disk:
 * memory allocation, a handle database, PE32+ images with IA-64
 * relocations, the console on COM1, block I/O on every block backend,
 * a read-only FAT driver (the El Torito image of a CD, EFI system
 * partitions), events with timers, and variables.
 *
 * Like the runtime services in virt.c every function is a stub that
 * breaks into qemu; images run in physical mode on the stack the boot
 * manager or their caller gave them.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"
#include "qemu/timer.h"
#include "qemu/ctype.h"
#include "cpu.h"
#include "hw/core/boards.h"
#include "chardev/char.h"
#include "system/system.h"
#include "system/runstate.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "system/block-backend.h"
#include <zlib.h>
#include "virt.h"

#define PAGE            4096ull
#define EVT_SIGNAL_VIRTUAL_ADDRESS_CHANGE 0x60000202u
#define EFI_FILE_MODE_WRITE     2

static bool efi_debug;

#define DPRINTF(fmt, ...) do { \
        if (efi_debug) { \
            fprintf(stderr, "efi: " fmt, ## __VA_ARGS__); \
        } \
    } while (0)

/*
 * GUIDs in their binary form
 */
typedef struct Guid {
    uint8_t b[16];
} Guid;

#define GUID(a, b, c, d0, d1, d2, d3, d4, d5, d6, d7) {{ \
        (a) & 0xff, ((a) >> 8) & 0xff, ((a) >> 16) & 0xff, (a) >> 24, \
        (b) & 0xff, (b) >> 8, (c) & 0xff, (c) >> 8, \
        d0, d1, d2, d3, d4, d5, d6, d7 }}

static const Guid guid_loaded_image =
    GUID(0x5b1b31a1, 0x9562, 0x11d2, 0x8e, 0x3f, 0x00, 0xa0, 0xc9, 0x69, 0x72, 0x3b);
static const Guid guid_device_path =
    GUID(0x09576e91, 0x6d3f, 0x11d2, 0x8e, 0x39, 0x00, 0xa0, 0xc9, 0x69, 0x72, 0x3b);
static const Guid guid_block_io =
    GUID(0x964e5b21, 0x6459, 0x11d2, 0x8e, 0x39, 0x00, 0xa0, 0xc9, 0x69, 0x72, 0x3b);
static const Guid guid_disk_io =
    GUID(0xce345171, 0xba0b, 0x11d2, 0x8e, 0x4f, 0x00, 0xa0, 0xc9, 0x69, 0x72, 0x3b);
static const Guid guid_simple_fs =
    GUID(0x964e5b22, 0x6459, 0x11d2, 0x8e, 0x39, 0x00, 0xa0, 0xc9, 0x69, 0x72, 0x3b);
static const Guid guid_file_info =
    GUID(0x09576e92, 0x6d3f, 0x11d2, 0x8e, 0x39, 0x00, 0xa0, 0xc9, 0x69, 0x72, 0x3b);
static const Guid guid_fs_info =
    GUID(0x09576e93, 0x6d3f, 0x11d2, 0x8e, 0x39, 0x00, 0xa0, 0xc9, 0x69, 0x72, 0x3b);
static const Guid guid_text_out =
    GUID(0x387477c2, 0x69c7, 0x11d2, 0x8e, 0x39, 0x00, 0xa0, 0xc9, 0x69, 0x72, 0x3b);
static const Guid guid_text_in =
    GUID(0x387477c1, 0x69c7, 0x11d2, 0x8e, 0x39, 0x00, 0xa0, 0xc9, 0x69, 0x72, 0x3b);
static const Guid guid_global_var =
    GUID(0x8be4df61, 0x93ca, 0x11d2, 0xaa, 0x0d, 0x00, 0xe0, 0x98, 0x03, 0x2b, 0x8c);
static const Guid guid_fpswa =
    GUID(0xc41b6531, 0x97b9, 0x11d3, 0x9a, 0x29, 0x00, 0x90, 0x27, 0x3f, 0xc1, 0x4d);
static const Guid guid_vt100 =
    GUID(0xdfa66065, 0xb419, 0x11d3, 0x9a, 0x2d, 0x00, 0x90, 0x27, 0x3f, 0xc1, 0x4d);

static void guid_str(const Guid *g, char *buf)
{
    sprintf(buf, "%08x-%04x-%04x-%02x%02x-%02x%02x%02x%02x%02x%02x",
            ldl_le_p(g->b), lduw_le_p(g->b + 4), lduw_le_p(g->b + 6),
            g->b[8], g->b[9], g->b[10], g->b[11], g->b[12], g->b[13],
            g->b[14], g->b[15]);
}

/*
 * Boot service and protocol function numbers
 */
enum {
    BS_RAISE_TPL, BS_RESTORE_TPL, BS_ALLOCATE_PAGES, BS_FREE_PAGES,
    BS_GET_MEMORY_MAP, BS_ALLOCATE_POOL, BS_FREE_POOL, BS_CREATE_EVENT,
    BS_SET_TIMER, BS_WAIT_FOR_EVENT, BS_SIGNAL_EVENT, BS_CLOSE_EVENT,
    BS_CHECK_EVENT, BS_INSTALL_PROTOCOL, BS_REINSTALL_PROTOCOL,
    BS_UNINSTALL_PROTOCOL, BS_HANDLE_PROTOCOL, BS_RESERVED,
    BS_REGISTER_PROTOCOL_NOTIFY, BS_LOCATE_HANDLE, BS_LOCATE_DEVICE_PATH,
    BS_INSTALL_CONFIG_TABLE, BS_LOAD_IMAGE, BS_START_IMAGE, BS_EXIT,
    BS_UNLOAD_IMAGE, BS_EXIT_BOOT_SERVICES, BS_GET_NEXT_MONOTONIC_COUNT,
    BS_STALL, BS_SET_WATCHDOG_TIMER, BS_CONNECT_CONTROLLER,
    BS_DISCONNECT_CONTROLLER, BS_OPEN_PROTOCOL, BS_CLOSE_PROTOCOL,
    BS_OPEN_PROTOCOL_INFORMATION, BS_PROTOCOLS_PER_HANDLE,
    BS_LOCATE_HANDLE_BUFFER, BS_LOCATE_PROTOCOL,
    BS_INSTALL_MULTIPLE, BS_UNINSTALL_MULTIPLE, BS_CALCULATE_CRC32,
    BS_COPY_MEM, BS_SET_MEM,
    BS_NUM
};

enum {
    P_TEXTIN_RESET = FW_CALL_PROTO, P_TEXTIN_READ_KEY,
    P_TEXTOUT_RESET, P_TEXTOUT_OUTPUT, P_TEXTOUT_TEST, P_TEXTOUT_QUERY_MODE,
    P_TEXTOUT_SET_MODE, P_TEXTOUT_SET_ATTR, P_TEXTOUT_CLEAR,
    P_TEXTOUT_SET_CURSOR, P_TEXTOUT_ENABLE_CURSOR,
    P_BLKIO_RESET, P_BLKIO_READ, P_BLKIO_WRITE, P_BLKIO_FLUSH,
    P_DISKIO_READ, P_DISKIO_WRITE,
    P_SFS_OPEN_VOLUME,
    P_FILE_OPEN, P_FILE_CLOSE, P_FILE_DELETE, P_FILE_READ, P_FILE_WRITE,
    P_FILE_GET_POS, P_FILE_SET_POS, P_FILE_GET_INFO, P_FILE_SET_INFO,
    P_FILE_FLUSH,
    P_IMAGE_UNLOAD,
    P_FPSWA,
    P_NOTIFY_RETURN,    /* return of an event notification function */
    P_END
};

QEMU_BUILD_BUG_ON(FW_CALL_BS + BS_NUM > FW_CALL_PROTO);
QEMU_BUILD_BUG_ON(P_END > 256);

/*
 * State
 */
typedef struct MemRange {
    uint64_t start, end;
    uint32_t type;
} MemRange;

#define MM_MAX          512

typedef struct Proto {
    Guid guid;
    uint64_t iface;
} Proto;

#define PROTO_MAX       16

typedef struct Handle {
    uint64_t addr;
    int n;
    Proto p[PROTO_MAX];
} Handle;

#define HANDLE_MAX      128

/* What StartImage has to restore when the image returns or exits */
typedef struct CallSnap {
    uint64_t gr[128];
    uint32_t nat[128];
    uint32_t pr[64];
    uint64_t br[8];
    IA64FReg fr[32];
    uint64_t cfm;
    uint64_t ar_rsc, ar_bspstore, ar_rnat, ar_unat, ar_fpsr;
    uint64_t ar_pfs, ar_lc, ar_ec;
    uint64_t rse[IA64_NUM_STACKED];
    uint32_t rse_nat[IA64_NUM_STACKED];
    int32_t rse_ndirty;
    uint64_t ip;
    uint32_t ri;
} CallSnap;

typedef struct Image {
    bool used, started, boot;
    uint64_t handle, base, pages, entry;
    uint32_t subsystem;
    uint32_t size;
    uint8_t *reloc;             /* a runtime driver's, for its new address */
    uint32_t reloc_size;
    uint8_t *loaded;            /* and the image as it was loaded */
    uint64_t exit_size_ptr, exit_data_ptr;
    CallSnap snap;
} Image;

#define IMAGE_MAX       16

typedef struct FatVol {
    BlockBackend *blk;
    uint64_t base;          /* byte offset of the volume */
    int type;               /* 12, 16 or 32 */
    uint32_t bps, csize;
    uint64_t fat_off, root_off, data_off;
    uint32_t root_size, root_clus, nclus;
    uint64_t total_bytes;
} FatVol;

typedef struct FatNode {
    uint32_t clus, size;
    uint8_t attr;
    bool fixed_root;        /* FAT12/16 root directory */
    uint16_t name[256];
} FatNode;

typedef struct DevPath {
    uint8_t b[1024];
    int n;
} DevPath;

typedef struct Disk {
    BlockBackend *blk;
    uint64_t base;          /* byte offset of this view */
    uint32_t bsize;
    uint64_t nblocks;
    bool cdrom, partition, ro;
    uint64_t handle, blkio, media, diskio, sfs, dpath;
    DevPath dp;             /* host copy of the device path */
    bool has_fat;
    FatVol fat;
} Disk;

#define DISK_MAX        32

typedef struct File {
    bool used;
    uint64_t addr;
    Disk *disk;
    FatNode node;
    uint64_t pos;
} File;

#define FILE_MAX        64

typedef struct Event {
    bool used, signaled, periodic, is_key;
    uint64_t addr;
    uint32_t type;
    uint64_t notify, context;   /* notification function, its argument */
    int64_t trigger, period;    /* QEMU_CLOCK_VIRTUAL ns, 0 = no timer */
} Event;

#define EVENT_MAX       64

typedef struct Var {
    bool used;
    uint16_t name[128];
    Guid guid;
    uint32_t attr;
    uint32_t size;
    uint8_t *data;
} Var;

#define VAR_MAX         128

typedef struct EfiState {
    CPUIA64State *env;
    bool exited;            /* ExitBootServices was called */
    uint64_t tpl;
    MemRange mm[MM_MAX];
    int mm_n;
    uint64_t map_key;
    uint64_t obj_next, obj_end;
    Handle handles[HANDLE_MAX];
    int nhandles;
    Image images[IMAGE_MAX];
    Image *active[IMAGE_MAX];
    int nactive;
    Disk disks[DISK_MAX];
    int ndisks;
    File files[FILE_MAX];
    Event events[EVENT_MAX];
    uint64_t file_proto;    /* the template EFI_FILE */
    uint64_t conin, conout, conout_mode, con_handle, key_event;
    int col, row;
    uint8_t inbuf[16];
    int inlen;
    bool autokey;
    uint64_t monotonic;
    int ncfg;
    bool dead;              /* nothing to boot; waiting for the shutdown */
    int boot_next;          /* the BootOrder position to try next */
    int boot_once;          /* BootNext + 1, or 0 */
    /* SetVirtualAddressMap: the map, and the notification in progress */
    struct { uint64_t phys, virt, size; } vmap[64];
    int nvmap;
    int notify_next;
    CallSnap notify_snap;
} EfiState;

static EfiState efi;
static Var vars[VAR_MAX];      /* survive a reset, like NVRAM */
/* only the very first boot gets the key the CD's boot program asks for */
static bool autokey_done;

/*
 * Guest memory: during boot services pointers are physical.
 */
static hwaddr pa(uint64_t va, MMUAccessType type)
{
    return fw_pa(efi.env, va, type);
}

#define rpa(va) pa(va, MMU_DATA_LOAD)
#define wpa(va) pa(va, MMU_DATA_STORE)

static uint8_t gld8(uint64_t va)
{
    return address_space_ldub(&address_space_memory, rpa(va),
                              MEMTXATTRS_UNSPECIFIED, NULL);
}

static uint16_t gld16(uint64_t va)
{
    return address_space_lduw_le(&address_space_memory, rpa(va),
                                 MEMTXATTRS_UNSPECIFIED, NULL);
}

static uint32_t gld32(uint64_t va)
{
    return address_space_ldl_le(&address_space_memory, rpa(va),
                                MEMTXATTRS_UNSPECIFIED, NULL);
}

static uint64_t gld64(uint64_t va)
{
    return address_space_ldq_le(&address_space_memory, rpa(va),
                                MEMTXATTRS_UNSPECIFIED, NULL);
}

static void gst8(uint64_t va, uint8_t v)
{
    address_space_stb(&address_space_memory, wpa(va), v,
                      MEMTXATTRS_UNSPECIFIED, NULL);
}

static void gst16(uint64_t va, uint16_t v)
{
    address_space_stw_le(&address_space_memory, wpa(va), v,
                         MEMTXATTRS_UNSPECIFIED, NULL);
}

static void gst32(uint64_t va, uint32_t v)
{
    address_space_stl_le(&address_space_memory, wpa(va), v,
                         MEMTXATTRS_UNSPECIFIED, NULL);
}

static void gst64(uint64_t va, uint64_t v)
{
    address_space_stq_le(&address_space_memory, wpa(va), v,
                         MEMTXATTRS_UNSPECIFIED, NULL);
}

/* Buffers of a virtually addressed caller: translate page by page */
static void gread(uint64_t va, void *buf, uint64_t len)
{
    while (len) {
        uint64_t n = MIN(len, PAGE - (va & (PAGE - 1)));

        address_space_read(&address_space_memory, rpa(va),
                           MEMTXATTRS_UNSPECIFIED, buf, n);
        va += n;
        buf = (uint8_t *)buf + n;
        len -= n;
    }
}

/* Fault in an output buffer before the first write of a call */
static void gprobe(uint64_t va, uint64_t len)
{
    while (len) {
        uint64_t n = MIN(len, PAGE - (va & (PAGE - 1)));

        wpa(va);
        va += n;
        len -= n;
    }
}

static void gwrite(uint64_t va, const void *buf, uint64_t len)
{
    while (len) {
        uint64_t n = MIN(len, PAGE - (va & (PAGE - 1)));

        address_space_write(&address_space_memory, wpa(va),
                            MEMTXATTRS_UNSPECIFIED, buf, n);
        va += n;
        buf = (const uint8_t *)buf + n;
        len -= n;
    }
}

static void gzero(uint64_t va, uint64_t len)
{
    static const uint8_t zero[4096];

    while (len) {
        uint64_t n = MIN(len, sizeof(zero));

        gwrite(va, zero, n);
        va += n;
        len -= n;
    }
}

static Guid gguid(uint64_t va)
{
    Guid g;

    gread(va, g.b, 16);
    return g;
}

/* A NUL-terminated UCS-2 string; returns its length */
static int gstr16(uint64_t va, uint16_t *buf, int max)
{
    int n = 0;

    while (n < max - 1) {
        uint16_t c = gld16(va + 2 * n);

        if (!c) {
            break;
        }
        buf[n++] = c;
    }
    buf[n] = 0;
    return n;
}

static int str16len(const uint16_t *s)
{
    int n = 0;

    while (s[n]) {
        n++;
    }
    return n;
}

static void str16_ascii(const uint16_t *s, char *buf, int max)
{
    int i;

    for (i = 0; s[i] && i < max - 1; i++) {
        buf[i] = s[i] < 0x80 ? s[i] : '?';
    }
    buf[i] = 0;
}

static void ascii_str16(const char *s, uint16_t *buf)
{
    while ((*buf++ = (uint8_t)*s++)) {
    }
}

/* Argument i of the call: r32.., from the ninth on the memory stack */
static uint64_t arg(int i)
{
    CPUIA64State *env = efi.env;

    if (i < 8) {
        return env->gr[32 + i];
    }
    return gld64(env->gr[12] + 16 + 8 * (i - 8));
}

/*
 * Memory map
 */
static int mm_cmp(const void *a, const void *b)
{
    const MemRange *x = a, *y = b;

    return x->start < y->start ? -1 : x->start > y->start;
}

/* Retype [start, end), which lies within RAM. */
static void mm_set(uint64_t start, uint64_t end, uint32_t type)
{
    MemRange out[MM_MAX + 2];
    int n = 0;

    for (int i = 0; i < efi.mm_n; i++) {
        MemRange r = efi.mm[i];

        if (r.end <= start || r.start >= end) {
            out[n++] = r;
            continue;
        }
        if (r.start < start) {
            out[n++] = (MemRange){ r.start, start, r.type };
        }
        if (r.end > end) {
            out[n++] = (MemRange){ end, r.end, r.type };
        }
    }
    out[n++] = (MemRange){ start, end, type };
    qsort(out, n, sizeof(out[0]), mm_cmp);

    efi.mm_n = 0;
    for (int i = 0; i < n; i++) {
        MemRange *last = efi.mm_n ? &efi.mm[efi.mm_n - 1] : NULL;

        if (last && last->type == out[i].type && last->end == out[i].start) {
            last->end = out[i].end;
        } else {
            assert(efi.mm_n < MM_MAX);
            efi.mm[efi.mm_n++] = out[i];
        }
    }
    efi.map_key++;
}

static bool mm_is_free(uint64_t start, uint64_t end)
{
    uint64_t at = start;

    for (int i = 0; i < efi.mm_n && at < end; i++) {
        MemRange *r = &efi.mm[i];

        if (r->start <= at && at < r->end) {
            if (r->type != EFI_CONVENTIONAL) {
                return false;
            }
            at = r->end;
        }
    }
    return at >= end;
}

/* AllocatePages: 0 any, 1 below *addr, 2 at *addr */
static uint64_t mm_alloc(int how, uint32_t type, uint64_t pages,
                         uint64_t *addr)
{
    uint64_t size = pages * PAGE;
    uint64_t max = how == 1 ? *addr : UINT64_MAX;

    if (!pages) {
        return EFI_INVALID_PARAMETER;
    }
    if (how == 2) {
        if ((*addr & (PAGE - 1)) || !mm_is_free(*addr, *addr + size)) {
            return EFI_NOT_FOUND;
        }
        mm_set(*addr, *addr + size, type);
        return EFI_SUCCESS;
    }
    /* top down, as most firmware does */
    for (int i = efi.mm_n - 1; i >= 0; i--) {
        MemRange *r = &efi.mm[i];
        uint64_t top = r->end;

        if (r->type != EFI_CONVENTIONAL) {
            continue;
        }
        if (max != UINT64_MAX && top > max + 1) {
            top = (max + 1) & ~(PAGE - 1);
        }
        if (top < r->start + size) {
            continue;
        }
        *addr = (top - size) & ~(PAGE - 1);
        mm_set(*addr, *addr + size, type);
        return EFI_SUCCESS;
    }
    return EFI_OUT_OF_RESOURCES;
}

static void mm_free(uint64_t addr, uint64_t pages)
{
    mm_set(addr, addr + pages * PAGE, EFI_CONVENTIONAL);
}

/* Boot services pool: whole pages behind a 16-byte header */
#define POOL_MAGIC      0x4c4f4f50494645ull     /* "EFIPOOL" */

static uint64_t pool_alloc(uint32_t type, uint64_t size)
{
    uint64_t pages = (size + 16 + PAGE - 1) / PAGE;
    uint64_t addr;

    if (mm_alloc(0, type, pages, &addr) != EFI_SUCCESS) {
        return 0;
    }
    gst64(addr, POOL_MAGIC);
    gst64(addr + 8, pages);
    return addr + 16;
}

static uint64_t pool_free(uint64_t buf)
{
    if (!buf || gld64(buf - 16) != POOL_MAGIC) {
        return EFI_INVALID_PARAMETER;
    }
    gst64(buf - 16, 0);
    mm_free(buf - 16, gld64(buf - 8));
    return EFI_SUCCESS;
}

/* Firmware objects that live until ExitBootServices */
static uint64_t obj_alloc(uint64_t size)
{
    uint64_t addr;

    size = ROUND_UP(size, 16);
    if (efi.obj_next + size > efi.obj_end) {
        uint64_t pages = MAX(16, (size + PAGE - 1) / PAGE);

        if (mm_alloc(0, EFI_BS_DATA, pages, &addr) != EFI_SUCCESS) {
            error_report("ia64 efi: out of memory for firmware objects");
            exit(EXIT_FAILURE);
        }
        efi.obj_next = addr;
        efi.obj_end = addr + pages * PAGE;
    }
    addr = efi.obj_next;
    efi.obj_next += size;
    gzero(addr, size);
    return addr;
}

/* A table of function descriptors for calls first..first+n-1 */
static uint64_t obj_functions(uint64_t at, int first, int n)
{
    for (int i = 0; i < n; i++) {
        gst64(at + 8 * i, FW_FD(first + i));
    }
    return at;
}

/*
 * Handles
 */
static Handle *handle_find(uint64_t addr)
{
    for (int i = 0; i < efi.nhandles; i++) {
        if (efi.handles[i].addr == addr && addr) {
            return &efi.handles[i];
        }
    }
    return NULL;
}

static Handle *handle_new(void)
{
    Handle *h;

    if (efi.nhandles == HANDLE_MAX) {
        error_report("ia64 efi: out of handles");
        return NULL;
    }
    h = &efi.handles[efi.nhandles++];
    memset(h, 0, sizeof(*h));
    h->addr = obj_alloc(16);
    gst64(h->addr, 0x454c444e4148ull);          /* "HANDLE" */
    return h;
}

static Proto *proto_find(Handle *h, const Guid *g)
{
    for (int i = 0; h && i < h->n; i++) {
        if (!memcmp(&h->p[i].guid, g, sizeof(*g))) {
            return &h->p[i];
        }
    }
    return NULL;
}

static uint64_t proto_install(Handle *h, const Guid *g, uint64_t iface)
{
    if (proto_find(h, g)) {
        return EFI_INVALID_PARAMETER;
    }
    if (h->n == PROTO_MAX) {
        return EFI_OUT_OF_RESOURCES;
    }
    h->p[h->n++] = (Proto){ *g, iface };
    return EFI_SUCCESS;
}

static uint64_t proto_uninstall(Handle *h, const Guid *g, uint64_t iface)
{
    Proto *p = proto_find(h, g);

    if (!p || p->iface != iface) {
        return EFI_NOT_FOUND;
    }
    memmove(p, p + 1, (h->p + h->n - (p + 1)) * sizeof(*p));
    h->n--;
    return EFI_SUCCESS;
}

/*
 * Device paths
 */
static void dp_node(DevPath *d, uint8_t type, uint8_t sub, const void *data,
                    int len)
{
    d->b[d->n] = type;
    d->b[d->n + 1] = sub;
    stw_le_p(d->b + d->n + 2, len + 4);
    memcpy(d->b + d->n + 4, data, len);
    d->n += len + 4;
}

static void dp_acpi(DevPath *d, uint32_t hid, uint32_t uid)
{
    uint8_t b[8];

    stl_le_p(b, hid);
    stl_le_p(b + 4, uid);
    dp_node(d, 2, 1, b, 8);
}

static void dp_pci(DevPath *d, uint8_t dev, uint8_t fn)
{
    uint8_t b[2] = { fn, dev };

    dp_node(d, 1, 1, b, 2);
}

static void dp_file(DevPath *d, const uint16_t *name)
{
    int len = str16len(name);
    uint8_t b[512];

    for (int i = 0; i <= len && 2 * i < sizeof(b) - 2; i++) {
        stw_le_p(b + 2 * i, name[i]);
    }
    dp_node(d, 4, 4, b, 2 * (len + 1));
}

static void dp_uart(DevPath *d)
{
    uint8_t uart[15] = { 0 };

    stq_le_p(uart + 4, 115200);
    uart[12] = 8;               /* data bits */
    uart[13] = 1;               /* no parity */
    uart[14] = 1;               /* one stop bit */
    dp_node(d, 3, 14, uart, sizeof(uart));
    dp_node(d, 3, 10, guid_vt100.b, 16);
}

static void dp_end(DevPath *d)
{
    dp_node(d, 0x7f, 0xff, NULL, 0);
}

static uint64_t dp_store(const DevPath *d)
{
    uint64_t addr = obj_alloc(d->n);

    gwrite(addr, d->b, d->n);
    return addr;
}

/* Length of a guest device path without its end node */
static int gdp_len(uint64_t va)
{
    int n = 0;

    while (n < 4096) {
        uint8_t type = gld8(va + n);
        uint16_t len = gld16(va + n + 2);

        if ((type & 0x7f) == 0x7f || len < 4) {
            break;
        }
        n += len;
    }
    return n;
}

/*
 * Images
 */
static void snap_save(CPUIA64State *env, CallSnap *s)
{
    memcpy(s->gr, env->gr, sizeof(s->gr));
    memcpy(s->nat, env->nat, sizeof(s->nat));
    memcpy(s->pr, env->pr, sizeof(s->pr));
    memcpy(s->br, env->br, sizeof(s->br));
    memcpy(s->fr, env->fr, sizeof(s->fr));
    s->cfm = env->cfm;
    s->ar_rsc = env->ar[AR_RSC];
    s->ar_bspstore = env->ar[AR_BSPSTORE];
    s->ar_rnat = env->ar[AR_RNAT];
    s->ar_unat = env->ar[AR_UNAT];
    s->ar_fpsr = env->ar[AR_FPSR];
    s->ar_pfs = env->ar[AR_PFS];
    s->ar_lc = env->ar[AR_LC];
    s->ar_ec = env->ar[AR_EC];
    memcpy(s->rse, env->rse, sizeof(s->rse));
    memcpy(s->rse_nat, env->rse_nat, sizeof(s->rse_nat));
    s->rse_ndirty = env->rse_ndirty;
    s->ip = env->ip;
    s->ri = env->ri;
}

static void snap_restore(CPUIA64State *env, const CallSnap *s)
{
    memcpy(env->gr, s->gr, sizeof(s->gr));
    memcpy(env->nat, s->nat, sizeof(s->nat));
    memcpy(env->pr, s->pr, sizeof(s->pr));
    memcpy(env->br, s->br, sizeof(s->br));
    memcpy(env->fr, s->fr, sizeof(s->fr));
    env->cfm = s->cfm;
    env->ar[AR_RSC] = s->ar_rsc;
    env->ar[AR_BSPSTORE] = s->ar_bspstore;
    env->ar[AR_RNAT] = s->ar_rnat;
    env->ar[AR_UNAT] = s->ar_unat;
    env->ar[AR_FPSR] = s->ar_fpsr;
    env->ar[AR_PFS] = s->ar_pfs;
    env->ar[AR_LC] = s->ar_lc;
    env->ar[AR_EC] = s->ar_ec;
    memcpy(env->rse, s->rse, sizeof(s->rse));
    memcpy(env->rse_nat, s->rse_nat, sizeof(s->rse_nat));
    env->rse_ndirty = s->rse_ndirty;
    env->ip = s->ip;
    env->ri = s->ri;
}

static Image *image_find(uint64_t handle)
{
    for (int i = 0; i < IMAGE_MAX; i++) {
        if (efi.images[i].used && efi.images[i].handle == handle) {
            return &efi.images[i];
        }
    }
    return NULL;
}

/* Field i of an MLX bundle */
static uint64_t bundle_get(const uint8_t *b, int pos, int len)
{
    uint64_t v = 0;

    for (int i = 0; i < len; i++) {
        int bit = pos + i;

        v |= (uint64_t)((b[bit >> 3] >> (bit & 7)) & 1) << i;
    }
    return v;
}

static void bundle_put(uint8_t *b, int pos, int len, uint64_t v)
{
    for (int i = 0; i < len; i++) {
        int bit = pos + i;

        b[bit >> 3] = (b[bit >> 3] & ~(1 << (bit & 7))) |
                      (((v >> i) & 1) << (bit & 7));
    }
}

/* movl's imm64: slot 1 is imm41, slot 2 (from bit 87) holds the rest */
static uint64_t movl_get(const uint8_t *b)
{
    return bundle_get(b, 87 + 13, 7) |
           bundle_get(b, 87 + 27, 9) << 7 |
           bundle_get(b, 87 + 22, 5) << 16 |
           bundle_get(b, 87 + 21, 1) << 21 |
           bundle_get(b, 46, 41) << 22 |
           bundle_get(b, 87 + 36, 1) << 63;
}

static void movl_put(uint8_t *b, uint64_t v)
{
    bundle_put(b, 87 + 13, 7, v);
    bundle_put(b, 87 + 27, 9, v >> 7);
    bundle_put(b, 87 + 22, 5, v >> 16);
    bundle_put(b, 87 + 21, 1, v >> 21);
    bundle_put(b, 46, 41, v >> 22);
    bundle_put(b, 87 + 36, 1, v >> 63);
}

/*
 * Apply the base relocations RELOC of an image IMG loaded DELTA away.  With
 * ORIG, the image as loaded, a place its driver has changed since, by
 * ConvertPointer, stays as it is.
 */
static void pe_relocate(uint8_t *img, uint32_t size, const uint8_t *reloc,
                        uint32_t reloc_size, uint64_t delta,
                        const uint8_t *orig)
{
    for (uint32_t p = 0; p + 8 <= reloc_size;) {
        uint32_t page = ldl_le_p(reloc + p);
        uint32_t bsize = ldl_le_p(reloc + p + 4);

        if (bsize < 8 || bsize > reloc_size - p) {
            break;
        }
        for (uint32_t i = 8; i + 2 <= bsize; i += 2) {
            uint16_t e = lduw_le_p(reloc + p + i);
            uint32_t at = page + (e & 0xfff);

            switch (e >> 12) {
            case 0:
                break;
            case 3:     /* HIGHLOW */
                if (at + 4 <= size &&
                    (!orig || !memcmp(img + at, orig + at, 4))) {
                    stl_le_p(img + at, ldl_le_p(img + at) + delta);
                }
                break;
            case 9:     /* IA64_IMM64: the movl bundle */
                at &= ~15;
                if (at + 16 <= size &&
                    (!orig || !memcmp(img + at, orig + at, 16))) {
                    movl_put(img + at, movl_get(img + at) + delta);
                }
                break;
            case 10:    /* DIR64 */
                if (at + 8 <= size &&
                    (!orig || !memcmp(img + at, orig + at, 8))) {
                    stq_le_p(img + at, ldq_le_p(img + at) + delta);
                }
                break;
            default:
                warn_report("ia64 efi: relocation type %d", e >> 12);
                break;
            }
        }
        p += bsize;
    }
}

/* Load a PE32+ image for IA-64; returns an EFI status */
static uint64_t pe_load(const uint8_t *f, uint64_t len, Image *im)
{
    uint32_t pe, o, nsec, optsz, size, hdrsize, entry;
    uint64_t imagebase, base, delta;
    uint32_t reloc_rva = 0, reloc_size = 0;
    uint16_t subsystem;
    uint8_t *img;
    uint32_t type;

    if (len < 0x40 || lduw_le_p(f) != 0x5a4d) {
        return EFI_LOAD_ERROR;
    }
    pe = ldl_le_p(f + 0x3c);
    if (pe + 24 + 112 > len || ldl_le_p(f + pe) != 0x4550 ||
        lduw_le_p(f + pe + 4) != 0x200) {
        return EFI_LOAD_ERROR;
    }
    nsec = lduw_le_p(f + pe + 6);
    optsz = lduw_le_p(f + pe + 20);
    o = pe + 24;
    if (lduw_le_p(f + o) != 0x20b) {
        return EFI_LOAD_ERROR;
    }
    entry = ldl_le_p(f + o + 16);
    imagebase = ldq_le_p(f + o + 24);
    size = ldl_le_p(f + o + 56);
    hdrsize = ldl_le_p(f + o + 60);
    subsystem = lduw_le_p(f + o + 68);
    if (ldl_le_p(f + o + 108) > 5) {
        reloc_rva = ldl_le_p(f + o + 112 + 8 * 5);
        reloc_size = ldl_le_p(f + o + 112 + 8 * 5 + 4);
    }
    if (hdrsize > len || hdrsize > size) {
        return EFI_LOAD_ERROR;
    }

    switch (subsystem) {
    case 10:
        type = EFI_LOADER_CODE;
        break;
    case 11:
        type = EFI_BS_CODE;
        break;
    case 12:
        type = EFI_RUNTIME_CODE;
        break;
    default:
        return EFI_UNSUPPORTED;
    }

    img = g_malloc0(size);
    memcpy(img, f, hdrsize);
    for (uint32_t i = 0; i < nsec; i++) {
        const uint8_t *s = f + o + optsz + 40 * i;
        uint32_t va = ldl_le_p(s + 12);
        uint32_t raw = ldl_le_p(s + 16);
        uint32_t rawp = ldl_le_p(s + 20);

        if (s + 40 > f + len) {
            g_free(img);
            return EFI_LOAD_ERROR;
        }
        if (va >= size) {
            continue;
        }
        raw = MIN(raw, size - va);
        if (rawp > len || raw > len - rawp) {
            g_free(img);
            return EFI_LOAD_ERROR;
        }
        memcpy(img + va, f + rawp, raw);
    }

    /* at the preferred address if possible, as Intel's EFI does */
    im->pages = (size + PAGE - 1) / PAGE;
    base = imagebase;
    if ((imagebase < 1 * MiB || (imagebase & (PAGE - 1)) ||
         mm_alloc(2, type, im->pages, &base) != EFI_SUCCESS) &&
        mm_alloc(0, type, im->pages, &base) != EFI_SUCCESS) {
        g_free(img);
        return EFI_OUT_OF_RESOURCES;
    }
    delta = base - imagebase;

    if (reloc_size && reloc_rva < size && reloc_size <= size - reloc_rva) {
        pe_relocate(img, size, img + reloc_rva, reloc_size, delta, NULL);
        if (subsystem == 12) {
            im->reloc = g_memdup2(img + reloc_rva, reloc_size);
            im->reloc_size = reloc_size;
            im->loaded = g_memdup2(img, size);
        }
    }

    gwrite(base, img, size);
    if (im->pages * PAGE > size) {
        gzero(base + size, im->pages * PAGE - size);
    }
    g_free(img);

    im->base = base;
    im->entry = base + entry;
    im->subsystem = subsystem;
    im->size = size;
    DPRINTF("image at 0x%" PRIx64 " size 0x%x entry 0x%" PRIx64
            " subsystem %d\n", base, size, im->entry, subsystem);
    return EFI_SUCCESS;
}

/* Make an image handle with its LoadedImage protocol */
static uint64_t image_create(const uint8_t *f, uint64_t len, uint64_t parent,
                             uint64_t device, uint64_t filepath,
                             const void *options, uint32_t options_size,
                             uint64_t *handle)
{
    Image *im = NULL;
    Handle *h;
    uint64_t li, st;
    uint32_t code_type, data_type;

    for (int i = 0; i < IMAGE_MAX; i++) {
        if (!efi.images[i].used) {
            im = &efi.images[i];
            break;
        }
    }
    if (!im) {
        return EFI_OUT_OF_RESOURCES;
    }
    memset(im, 0, sizeof(*im));
    st = pe_load(f, len, im);
    if (st != EFI_SUCCESS) {
        return st;
    }
    h = handle_new();
    if (!h) {
        mm_free(im->base, im->pages);
        g_clear_pointer(&im->reloc, g_free);
        g_clear_pointer(&im->loaded, g_free);
        return EFI_OUT_OF_RESOURCES;
    }
    im->used = true;
    im->handle = h->addr;

    switch (im->subsystem) {
    case 11:
        code_type = EFI_BS_CODE;
        data_type = EFI_BS_DATA;
        break;
    case 12:
        code_type = EFI_RUNTIME_CODE;
        data_type = EFI_RUNTIME_DATA;
        break;
    default:
        code_type = EFI_LOADER_CODE;
        data_type = EFI_LOADER_DATA;
        break;
    }

    li = obj_alloc(96);
    gst32(li, 0x1000);
    gst64(li + 8, parent);
    gst64(li + 16, FW_SYSTAB);
    gst64(li + 24, device);
    gst64(li + 32, filepath);
    if (options_size) {
        uint64_t opt = obj_alloc(options_size);

        gwrite(opt, options, options_size);
        gst32(li + 48, options_size);
        gst64(li + 56, opt);
    }
    gst64(li + 64, im->base);
    gst64(li + 72, im->pages * PAGE);
    gst32(li + 80, code_type);
    gst32(li + 84, data_type);
    gst64(li + 88, FW_FD(P_IMAGE_UNLOAD));
    proto_install(h, &guid_loaded_image, li);
    *handle = h->addr;
    return EFI_SUCCESS;
}

/*
 * Call the image's entry point as if StartImage had called it: the
 * caller's frame becomes the image's, with ar.pfs describing it, so that
 * the image's br.ret leaves the frame as it was.  It returns to
 * FW_CALL_IMAGE_RETURN.
 */
static void image_enter(CPUIA64State *env, Image *im)
{
    uint64_t code = gld64(im->entry), gp = gld64(im->entry + 8);

    if ((env->cfm & 0x7f) < 2) {
        env->cfm = 2;
    }
    env->gr[32] = im->handle;
    env->gr[33] = FW_SYSTAB;
    env->nat[32] = env->nat[33] = 0;
    env->ar[AR_PFS] = (env->cfm & ((1ull << 38) - 1)) |
                      ((env->ar[AR_EC] & 0x3f) << 52);
    env->gr[1] = gp;
    env->nat[1] = 0;
    env->br[0] = FW_STUB(FW_CALL_IMAGE_RETURN);
    env->ip = code & ~15ull;
    env->ri = 0;
    im->started = true;
    efi.active[efi.nactive++] = im;
}

static void image_unload(Image *im)
{
    Handle *h = handle_find(im->handle);

    mm_free(im->base, im->pages);
    g_clear_pointer(&im->reloc, g_free);
    g_clear_pointer(&im->loaded, g_free);
    if (h) {
        h->n = 0;
    }
    im->used = false;
}

static void efi_boot(CPUIA64State *env);

/* The image returned or called Exit: back to its StartImage */
static void image_finish(CPUIA64State *env, Image *im, uint64_t status)
{
    while (efi.nactive && efi.active[efi.nactive - 1] != im) {
        efi.active[--efi.nactive]->started = false;
    }
    if (efi.nactive) {
        efi.nactive--;
    }
    im->started = false;
    DPRINTF("image 0x%" PRIx64 " finished: 0x%" PRIx64 "\n", im->base,
            status);
    if (im->boot) {
        DPRINTF("boot application returned 0x%" PRIx64 "\n", status);
        image_unload(im);
        efi_boot(env);
        return;
    }
    snap_restore(env, &im->snap);
    env->gr[8] = status;
    env->nat[8] = 0;
    if (im->exit_size_ptr) {
        gst64(im->exit_size_ptr, 0);
    }
    /* an application leaves; a driver that succeeded stays resident */
    if (im->subsystem == 10 || status != EFI_SUCCESS) {
        image_unload(im);
    }
}

/*
 * Console on COM1: VT100 out, keys in
 */
static void con_write(const char *s, int len)
{
    Chardev *chr = serial_hd(0);

    if (chr) {
        qemu_chr_write_all(chr, (const uint8_t *)s, len);
    }
}

static void con_printf(const char *fmt, ...) G_GNUC_PRINTF(1, 2);

static void con_printf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    con_write(buf, MIN(n, sizeof(buf) - 1));
}

static char con_char(uint16_t c)
{
    if (c >= 0x20 && c < 0x7f) {
        return c;
    }
    switch (c) {
    case 0x2500: case 0x2550:               /* horizontal lines */
        return '-';
    case 0x2502: case 0x2551:               /* vertical lines */
        return '|';
    case 0x250c: case 0x2510: case 0x2514: case 0x2518:
    case 0x251c: case 0x2524: case 0x252c: case 0x2534: case 0x253c:
    case 0x2554: case 0x2557: case 0x255a: case 0x255d:
    case 0x2552: case 0x2553: case 0x2555: case 0x2556:
    case 0x2558: case 0x2559: case 0x255b: case 0x255c:
    case 0x255e: case 0x255f: case 0x2560: case 0x2561: case 0x2562:
    case 0x2563: case 0x2564: case 0x2565: case 0x2566: case 0x2567:
    case 0x2568: case 0x2569: case 0x256a: case 0x256b: case 0x256c:
        return '+';
    case 0x2588: case 0x2591: case 0x2592: case 0x2593:
        return '#';
    case 0x25b2: return '^';
    case 0x25ba: return '>';
    case 0x25bc: return 'v';
    case 0x25c4: return '<';
    }
    return '?';
}

static void con_output(uint64_t str)
{
    char buf[256];
    int n = 0;

    for (int i = 0; i < 65536; i++) {
        uint16_t c = gld16(str + 2 * i);

        if (!c) {
            break;
        }
        switch (c) {
        case '\r':
            buf[n++] = '\r';
            efi.col = 0;
            break;
        case '\n':
            buf[n++] = '\n';
            efi.row = MIN(efi.row + 1, 24);
            break;
        case '\b':
            buf[n++] = '\b';
            efi.col = MAX(efi.col - 1, 0);
            break;
        default:
            buf[n++] = con_char(c);
            if (++efi.col == 80) {
                efi.col = 0;
                efi.row = MIN(efi.row + 1, 24);
            }
            break;
        }
        if (n >= sizeof(buf) - 1) {
            con_write(buf, n);
            n = 0;
        }
    }
    con_write(buf, n);
    gst32(efi.conout_mode + 12, efi.col);
    gst32(efi.conout_mode + 16, efi.row);
}

static void con_poll(void)
{
    int c;

    while (efi.inlen < sizeof(efi.inbuf) && (c = virt_com1_getc()) >= 0) {
        efi.inbuf[efi.inlen++] = c;
    }
}

static void con_consume(int n)
{
    memmove(efi.inbuf, efi.inbuf + n, efi.inlen - n);
    efi.inlen -= n;
}

/* One key from the input: scan code and character, false if none */
static bool con_key(uint16_t *scan, uint16_t *uc)
{
    static const struct {
        const char *seq;
        uint16_t scan;
    } seqs[] = {
        { "\033[A", 1 }, { "\033[B", 2 }, { "\033[C", 3 }, { "\033[D", 4 },
        { "\033[H", 5 }, { "\033[F", 6 }, { "\033[1~", 5 }, { "\033[4~", 6 },
        { "\033[2~", 7 }, { "\033[3~", 8 }, { "\033[5~", 9 },
        { "\033[6~", 10 },
        { "\033OP", 11 }, { "\033OQ", 12 }, { "\033OR", 13 }, { "\033OS", 14 },
        { "\033[11~", 11 }, { "\033[12~", 12 }, { "\033[13~", 13 },
        { "\033[14~", 14 }, { "\033[15~", 15 }, { "\033[17~", 16 },
        { "\033[18~", 17 }, { "\033[19~", 18 }, { "\033[20~", 19 },
        { "\033[21~", 20 },
        /* VT100+ function keys */
        { "\0331", 11 }, { "\0332", 12 }, { "\0333", 13 }, { "\0334", 14 },
        { "\0335", 15 }, { "\0336", 16 }, { "\0337", 17 }, { "\0338", 18 },
        { "\0339", 19 }, { "\0330", 20 },
    };

    con_poll();
    if (!efi.inlen) {
        return false;
    }
    *scan = 0;
    *uc = 0;
    if (efi.inbuf[0] == 0x1b) {
        for (int i = 0; i < ARRAY_SIZE(seqs); i++) {
            int len = strlen(seqs[i].seq);

            if (efi.inlen >= len && !memcmp(efi.inbuf, seqs[i].seq, len)) {
                *scan = seqs[i].scan;
                con_consume(len);
                return true;
            }
        }
        *scan = 0x17;               /* escape */
        con_consume(1);
        return true;
    }
    switch (efi.inbuf[0]) {
    case '\n':
        *uc = '\r';
        break;
    case 0x7f:
        *uc = '\b';
        break;
    default:
        *uc = efi.inbuf[0];
        break;
    }
    con_consume(1);
    return true;
}

static bool con_key_ready(void)
{
    con_poll();
    return efi.inlen > 0;
}

static const uint8_t ansi_color[8] = { 0, 4, 2, 6, 1, 5, 3, 7 };

static void textout_call(CPUIA64State *env, unsigned n)
{
    uint64_t st = EFI_SUCCESS;

    switch (n) {
    case P_TEXTOUT_RESET:
        con_printf("\033[0m\033[2J\033[H");
        efi.col = efi.row = 0;
        break;
    case P_TEXTOUT_OUTPUT:
        con_output(arg(1));
        break;
    case P_TEXTOUT_TEST:
        break;
    case P_TEXTOUT_QUERY_MODE:
        if (arg(1) != 0) {
            st = EFI_UNSUPPORTED;
            break;
        }
        gst64(arg(2), 80);
        gst64(arg(3), 25);
        break;
    case P_TEXTOUT_SET_MODE:
        st = arg(1) == 0 ? EFI_SUCCESS : EFI_UNSUPPORTED;
        break;
    case P_TEXTOUT_SET_ATTR: {
        uint64_t a = arg(1);

        con_printf("\033[0;%s%d;%dm", (a & 8) ? "1;" : "",
                   30 + ansi_color[a & 7], 40 + ansi_color[(a >> 4) & 7]);
        gst32(efi.conout_mode + 8, a);
        break;
    }
    case P_TEXTOUT_CLEAR:
        con_printf("\033[2J\033[H");
        efi.col = efi.row = 0;
        gst32(efi.conout_mode + 12, 0);
        gst32(efi.conout_mode + 16, 0);
        break;
    case P_TEXTOUT_SET_CURSOR:
        if (arg(1) >= 80 || arg(2) >= 25) {
            st = EFI_UNSUPPORTED;
            break;
        }
        efi.col = arg(1);
        efi.row = arg(2);
        con_printf("\033[%d;%dH", efi.row + 1, efi.col + 1);
        gst32(efi.conout_mode + 12, efi.col);
        gst32(efi.conout_mode + 16, efi.row);
        break;
    case P_TEXTOUT_ENABLE_CURSOR:
        con_printf(arg(1) ? "\033[?25h" : "\033[?25l");
        gst8(efi.conout_mode + 20, !!arg(1));
        break;
    }
    fw_ret(env, st, 0, 0, 0);
}

/*
 * Block devices
 */
static int blk_io(Disk *d, uint64_t off, void *buf, uint64_t len, bool wr)
{
    int ret;

    if (wr) {
        ret = blk_pwrite(d->blk, d->base + off, len, buf, 0);
    } else {
        ret = blk_pread(d->blk, d->base + off, len, buf, 0);
    }
    return ret;
}

/* Copy between a disk and guest memory in pieces */
static uint64_t disk_xfer(Disk *d, uint64_t off, uint64_t va, uint64_t len,
                          bool wr)
{
    uint8_t *buf = g_malloc(MIN(len, 1 * MiB));
    uint64_t st = EFI_SUCCESS;

    while (len) {
        uint64_t n = MIN(len, 1 * MiB);

        if (wr) {
            gread(va, buf, n);
        }
        if (blk_io(d, off, buf, n, wr) < 0) {
            st = EFI_DEVICE_ERROR;
            break;
        }
        if (!wr) {
            gwrite(va, buf, n);
        }
        off += n;
        va += n;
        len -= n;
    }
    g_free(buf);
    return st;
}

static Disk *disk_by(uint64_t iface, int which)
{
    for (int i = 0; i < efi.ndisks; i++) {
        Disk *d = &efi.disks[i];
        uint64_t v = which == 0 ? d->blkio : which == 1 ? d->diskio : d->sfs;

        if (v == iface) {
            return d;
        }
    }
    return NULL;
}

static void blkio_call(CPUIA64State *env, unsigned n)
{
    Disk *d = disk_by(arg(0), 0);
    uint64_t st = EFI_SUCCESS;

    if (!d) {
        fw_ret(env, EFI_INVALID_PARAMETER, 0, 0, 0);
        return;
    }
    switch (n) {
    case P_BLKIO_READ:
    case P_BLKIO_WRITE: {
        uint64_t lba = arg(2), size = arg(3), buf = arg(4);
        bool wr = n == P_BLKIO_WRITE;

        if (wr && d->ro) {
            st = EFI_WRITE_PROTECTED;
        } else if (size % d->bsize) {
            st = EFI_BAD_BUFFER_SIZE;
        } else if (lba > d->nblocks || size / d->bsize > d->nblocks - lba) {
            st = EFI_INVALID_PARAMETER;
        } else {
            st = disk_xfer(d, lba * d->bsize, buf, size, wr);
        }
        DPRINTF("blkio %s lba 0x%" PRIx64 " size 0x%" PRIx64 " -> 0x%"
                PRIx64 "\n", wr ? "write" : "read", lba, size, st);
        break;
    }
    case P_BLKIO_RESET:
    case P_BLKIO_FLUSH:
        break;
    }
    fw_ret(env, st, 0, 0, 0);
}

static void diskio_call(CPUIA64State *env, unsigned n)
{
    Disk *d = disk_by(arg(0), 1);
    uint64_t off = arg(2), size = arg(3), buf = arg(4);
    uint64_t st;
    bool wr = n == P_DISKIO_WRITE;

    if (!d) {
        st = EFI_INVALID_PARAMETER;
    } else if (wr && d->ro) {
        st = EFI_WRITE_PROTECTED;
    } else if (off > d->nblocks * d->bsize ||
               size > d->nblocks * d->bsize - off) {
        st = EFI_INVALID_PARAMETER;
    } else {
        st = disk_xfer(d, off, buf, size, wr);
    }
    fw_ret(env, st, 0, 0, 0);
}

/*
 * FAT, read only
 */
static int fat_read_bytes(FatVol *v, uint64_t off, void *buf, uint64_t len)
{
    return blk_pread(v->blk, v->base + off, len, buf, 0);
}

static bool fat_mount(FatVol *v, BlockBackend *blk, uint64_t base,
                      uint64_t avail)
{
    uint8_t b[512];
    uint32_t bps, spc, res, nfats, roots, tot, fatsz, rootsecs, datasecs;

    if (blk_pread(blk, base, 512, b, 0) < 0 || lduw_le_p(b + 510) != 0xaa55) {
        return false;
    }
    bps = lduw_le_p(b + 11);
    spc = b[13];
    res = lduw_le_p(b + 14);
    nfats = b[16];
    roots = lduw_le_p(b + 17);
    tot = lduw_le_p(b + 19) ? lduw_le_p(b + 19) : ldl_le_p(b + 32);
    fatsz = lduw_le_p(b + 22) ? lduw_le_p(b + 22) : ldl_le_p(b + 36);
    if ((bps != 512 && bps != 1024 && bps != 2048 && bps != 4096) ||
        !spc || (spc & (spc - 1)) || !res || !nfats || !fatsz || !tot) {
        return false;
    }
    rootsecs = (roots * 32 + bps - 1) / bps;
    if (res + nfats * fatsz + rootsecs >= tot) {
        return false;
    }
    datasecs = tot - res - nfats * fatsz - rootsecs;

    v->blk = blk;
    v->base = base;
    v->bps = bps;
    v->csize = bps * spc;
    v->nclus = datasecs / spc;
    v->type = v->nclus < 4085 ? 12 : v->nclus < 65525 ? 16 : 32;
    v->fat_off = (uint64_t)res * bps;
    v->root_off = v->fat_off + (uint64_t)nfats * fatsz * bps;
    v->root_size = roots * 32;
    v->data_off = v->root_off + (uint64_t)rootsecs * bps;
    v->root_clus = v->type == 32 ? ldl_le_p(b + 44) : 0;
    v->total_bytes = (uint64_t)tot * bps;
    if (avail && v->total_bytes > avail) {
        return false;
    }
    DPRINTF("FAT%d at 0x%" PRIx64 ": %u clusters of %u bytes\n", v->type,
            base, v->nclus, v->csize);
    return true;
}

static uint32_t fat_next(FatVol *v, uint32_t c)
{
    uint8_t b[4] = { 0 };
    uint32_t e;

    switch (v->type) {
    case 12:
        fat_read_bytes(v, v->fat_off + c + c / 2, b, 2);
        e = lduw_le_p(b);
        e = c & 1 ? e >> 4 : e & 0xfff;
        return e >= 0xff8 ? 0 : e;
    case 16:
        fat_read_bytes(v, v->fat_off + 2 * c, b, 2);
        c = lduw_le_p(b);
        return c >= 0xfff8 ? 0 : c;
    default:
        fat_read_bytes(v, v->fat_off + 4 * (uint64_t)c, b, 4);
        c = ldl_le_p(b) & 0x0fffffff;
        return c >= 0x0ffffff8 ? 0 : c;
    }
}

/* Read from a file or directory; returns the bytes read */
static uint64_t fat_read(FatVol *v, const FatNode *n, uint64_t pos,
                         uint8_t *buf, uint64_t len)
{
    uint64_t done = 0;
    uint32_t c;

    if (n->fixed_root) {
        if (pos >= v->root_size) {
            return 0;
        }
        len = MIN(len, v->root_size - pos);
        return fat_read_bytes(v, v->root_off + pos, buf, len) < 0 ? 0 : len;
    }
    if (!(n->attr & 0x10)) {
        if (pos >= n->size) {
            return 0;
        }
        len = MIN(len, n->size - pos);
    }
    c = n->clus;
    for (uint64_t skip = pos / v->csize; skip && c >= 2; skip--) {
        c = fat_next(v, c);
    }
    pos %= v->csize;
    while (len && c >= 2 && c < v->nclus + 2) {
        uint64_t k = MIN(len, v->csize - pos);
        uint64_t off = v->data_off + (uint64_t)(c - 2) * v->csize + pos;

        if (fat_read_bytes(v, off, buf + done, k) < 0) {
            break;
        }
        done += k;
        len -= k;
        pos = 0;
        c = fat_next(v, c);
    }
    return done;
}

/*
 * The directory entry at or after *pos, with its long name; *pos ends
 * behind it.  False at the end of the directory.
 */
static bool fat_dir_next(FatVol *v, const FatNode *dir, uint64_t *pos,
                         FatNode *out)
{
    uint16_t lfn[260];
    bool have_lfn = false;

    while (1) {
        uint8_t e[32];

        if (fat_read(v, dir, *pos, e, 32) != 32 || e[0] == 0) {
            return false;
        }
        *pos += 32;
        if (e[0] == 0xe5) {
            have_lfn = false;
            continue;
        }
        if ((e[11] & 0x3f) == 0x0f) {
            int seq = (e[0] & 0x1f) - 1;
            static const int off[13] = { 1, 3, 5, 7, 9, 14, 16, 18, 20, 22,
                                         24, 28, 30 };

            if (seq < 0 || seq >= 20) {
                continue;
            }
            if (e[0] & 0x40) {
                memset(lfn, 0, sizeof(lfn));
                have_lfn = true;
            }
            for (int i = 0; i < 13; i++) {
                uint16_t ch = lduw_le_p(e + off[i]);

                lfn[seq * 13 + i] = ch == 0xffff ? 0 : ch;
            }
            continue;
        }
        if (e[11] & 0x08) {         /* volume label */
            have_lfn = false;
            continue;
        }
        memset(out, 0, sizeof(*out));
        out->attr = e[11];
        out->size = ldl_le_p(e + 28);
        out->clus = lduw_le_p(e + 26) | (lduw_le_p(e + 20) << 16);
        if (have_lfn) {
            memcpy(out->name, lfn, sizeof(out->name) - 2);
        } else {
            int k = 0;

            for (int i = 0; i < 8 && e[i] != ' '; i++) {
                out->name[k++] = (e[12] & 0x08) ? qemu_tolower(e[i]) : e[i];
            }
            if (e[8] != ' ') {
                out->name[k++] = '.';
                for (int i = 8; i < 11 && e[i] != ' '; i++) {
                    out->name[k++] = (e[12] & 0x10) ? qemu_tolower(e[i])
                                                    : e[i];
                }
            }
            out->name[k] = 0;
        }
        return true;
    }
}

static FatNode fat_root(FatVol *v)
{
    FatNode n = { .attr = 0x10 };

    if (v->type == 32) {
        n.clus = v->root_clus;
    } else {
        n.fixed_root = true;
    }
    return n;
}

static bool name_eq(const uint16_t *a, const uint16_t *b, int blen)
{
    int i;

    for (i = 0; i < blen; i++) {
        uint16_t x = a[i], y = b[i];

        if (!x) {
            return false;
        }
        if (x < 0x80) {
            x = qemu_toupper(x);
        }
        if (y < 0x80) {
            y = qemu_toupper(y);
        }
        if (x != y) {
            return false;
        }
    }
    return a[i] == 0;
}

/* Resolve a path relative to dir ('\' separated); false if missing */
static bool fat_lookup(FatVol *v, const FatNode *dir, const uint16_t *path,
                       FatNode *out)
{
    FatNode cur = *dir;

    if (*path == '\\') {
        cur = fat_root(v);
    }
    while (*path) {
        const uint16_t *end;
        uint64_t pos = 0;
        FatNode e;
        bool found = false;

        while (*path == '\\') {
            path++;
        }
        if (!*path) {
            break;
        }
        for (end = path; *end && *end != '\\'; end++) {
        }
        if (end - path == 1 && path[0] == '.') {
            path = end;
            continue;
        }
        if (!(cur.attr & 0x10)) {
            return false;
        }
        while (fat_dir_next(v, &cur, &pos, &e)) {
            if (name_eq(e.name, path, end - path)) {
                found = true;
                break;
            }
        }
        if (!found) {
            return false;
        }
        if ((e.attr & 0x10) && e.clus == 0) {
            e = fat_root(v);        /* ".." of a first level directory */
        }
        cur = e;
        path = end;
    }
    *out = cur;
    return true;
}

/*
 * Files: an EFI_FILE per open file, its revision followed by the
 * function table
 */
static File *file_by(uint64_t addr)
{
    for (int i = 0; i < FILE_MAX; i++) {
        if (efi.files[i].used && efi.files[i].addr == addr) {
            return &efi.files[i];
        }
    }
    return NULL;
}

static File *file_new(Disk *d, const FatNode *node)
{
    for (int i = 0; i < FILE_MAX; i++) {
        File *f = &efi.files[i];

        if (!f->used) {
            if (!f->addr) {
                f->addr = obj_alloc(8 + 8 * 10);
                gst64(f->addr, 0x00010000);
                obj_functions(f->addr + 8, P_FILE_OPEN, 10);
            }
            f->used = true;
            f->disk = d;
            f->node = *node;
            f->pos = 0;
            return f;
        }
    }
    return NULL;
}

/* EFI_FILE_INFO for node into buf; returns its size */
static uint64_t file_info(const FatNode *n, uint8_t *buf, uint64_t max)
{
    int len = str16len(n->name);
    uint64_t size = 80 + 2 * (len + 1);

    if (size > max) {
        return size;
    }
    memset(buf, 0, size);
    stq_le_p(buf, size);
    stq_le_p(buf + 8, (n->attr & 0x10) ? 0 : n->size);
    stq_le_p(buf + 16, (n->attr & 0x10) ? 0 : n->size);
    stq_le_p(buf + 72, n->attr & 0x37);
    for (int i = 0; i <= len; i++) {
        stw_le_p(buf + 80 + 2 * i, n->name[i]);
    }
    return size;
}

static void file_call(CPUIA64State *env, unsigned n)
{
    File *f = file_by(arg(0));
    uint64_t st = EFI_SUCCESS;

    if (!f) {
        fw_ret(env, EFI_INVALID_PARAMETER, 0, 0, 0);
        return;
    }
    switch (n) {
    case P_FILE_OPEN: {
        uint16_t name[260];
        FatNode node;
        File *nf;
        char s[260];

        gstr16(arg(2), name, ARRAY_SIZE(name));
        str16_ascii(name, s, sizeof(s));
        if (arg(3) & EFI_FILE_MODE_WRITE) {
            st = EFI_WRITE_PROTECTED;
        } else if (!fat_lookup(&f->disk->fat, &f->node, name, &node)) {
            st = EFI_NOT_FOUND;
        } else if (!(nf = file_new(f->disk, &node))) {
            st = EFI_OUT_OF_RESOURCES;
        } else {
            gst64(arg(1), nf->addr);
        }
        DPRINTF("open '%s' -> 0x%" PRIx64 "\n", s, st);
        break;
    }
    case P_FILE_CLOSE:
        f->used = false;
        break;
    case P_FILE_DELETE:
        f->used = false;
        st = EFI_WRITE_PROTECTED;
        break;
    case P_FILE_READ: {
        uint64_t size = gld64(arg(1)), buf = arg(2);

        if (f->node.attr & 0x10) {
            FatNode e;
            uint64_t pos = f->pos;
            uint8_t info[600];
            uint64_t isz;

            do {
                if (!fat_dir_next(&f->disk->fat, &f->node, &pos, &e)) {
                    gst64(arg(1), 0);
                    goto out;
                }
            } while ((e.name[0] == '.' && !e.name[1]) ||
                     (e.name[0] == '.' && e.name[1] == '.' && !e.name[2]));
            isz = file_info(&e, info, sizeof(info));
            if (isz > size) {
                gst64(arg(1), isz);
                st = EFI_BUFFER_TOO_SMALL;
                break;
            }
            gwrite(buf, info, isz);
            gst64(arg(1), isz);
            f->pos = pos;
        } else {
            uint8_t *tmp = g_malloc(MIN(size, 16 * MiB) + 1);
            uint64_t done = 0;

            while (done < size) {
                uint64_t k = MIN(size - done, 16 * MiB);
                uint64_t got = fat_read(&f->disk->fat, &f->node, f->pos,
                                        tmp, k);

                gwrite(buf + done, tmp, got);
                done += got;
                f->pos += got;
                if (got < k) {
                    break;
                }
            }
            g_free(tmp);
            gst64(arg(1), done);
        }
        break;
    }
    case P_FILE_WRITE:
    case P_FILE_SET_INFO:
        st = EFI_WRITE_PROTECTED;
        break;
    case P_FILE_GET_POS:
        if (f->node.attr & 0x10) {
            st = EFI_UNSUPPORTED;
        } else {
            gst64(arg(1), f->pos);
        }
        break;
    case P_FILE_SET_POS: {
        uint64_t pos = arg(1);

        if (f->node.attr & 0x10) {
            if (pos) {
                st = EFI_UNSUPPORTED;
            } else {
                f->pos = 0;
            }
        } else {
            f->pos = pos == UINT64_MAX ? f->node.size : pos;
        }
        break;
    }
    case P_FILE_GET_INFO: {
        Guid g = gguid(arg(1));
        uint64_t size = gld64(arg(2)), buf = arg(3);
        uint8_t info[600];
        uint64_t isz;

        if (!memcmp(&g, &guid_file_info, 16)) {
            isz = file_info(&f->node, info, sizeof(info));
        } else if (!memcmp(&g, &guid_fs_info, 16)) {
            FatVol *v = &f->disk->fat;

            isz = 36 + 2;
            memset(info, 0, isz);
            stq_le_p(info, isz);
            info[8] = 1;                            /* read only */
            stq_le_p(info + 16, v->total_bytes);
            stq_le_p(info + 24, 0);
            stl_le_p(info + 32, v->csize);
        } else {
            st = EFI_UNSUPPORTED;
            break;
        }
        if (isz > size) {
            st = EFI_BUFFER_TOO_SMALL;
        } else {
            gwrite(buf, info, isz);
        }
        gst64(arg(2), isz);
        break;
    }
    case P_FILE_FLUSH:
        break;
    }
out:
    fw_ret(env, st, 0, 0, 0);
}

/* Read a whole file into a host buffer */
static uint8_t *fat_slurp(Disk *d, const uint16_t *path, uint64_t *len)
{
    FatNode root = fat_root(&d->fat), node;
    uint8_t *buf;

    if (!fat_lookup(&d->fat, &root, path, &node) || (node.attr & 0x10)) {
        return NULL;
    }
    buf = g_malloc(node.size + 1);
    *len = fat_read(&d->fat, &node, 0, buf, node.size);
    if (*len != node.size) {
        g_free(buf);
        return NULL;
    }
    return buf;
}

/*
 * Disks: one handle for the device, one per partition
 */
static Disk *disk_new(BlockBackend *blk, uint64_t base, uint32_t bsize,
                      uint64_t nblocks, const DevPath *dp, bool cdrom,
                      bool partition)
{
    Disk *d;
    Handle *h;

    if (efi.ndisks == DISK_MAX || !(h = handle_new())) {
        return NULL;
    }
    d = &efi.disks[efi.ndisks++];
    memset(d, 0, sizeof(*d));
    d->blk = blk;
    d->base = base;
    d->bsize = bsize;
    d->nblocks = nblocks;
    d->cdrom = cdrom;
    d->partition = partition;
    d->ro = cdrom || !blk_is_writable(blk);
    d->handle = h->addr;

    d->media = obj_alloc(32);
    gst32(d->media, 1);                         /* media id */
    gst8(d->media + 4, cdrom);                  /* removable */
    gst8(d->media + 5, 1);                      /* present */
    gst8(d->media + 6, partition);
    gst8(d->media + 7, d->ro);
    gst32(d->media + 12, bsize);
    gst32(d->media + 16, 0);
    gst64(d->media + 24, nblocks - 1);

    d->blkio = obj_alloc(48);
    gst64(d->blkio, 0x00010000);
    gst64(d->blkio + 8, d->media);
    obj_functions(d->blkio + 16, P_BLKIO_RESET, 4);

    d->diskio = obj_alloc(24);
    gst64(d->diskio, 0x00010000);
    obj_functions(d->diskio + 8, P_DISKIO_READ, 2);

    d->dpath = dp_store(dp);
    d->dp = *dp;
    proto_install(h, &guid_device_path, d->dpath);
    proto_install(h, &guid_block_io, d->blkio);
    proto_install(h, &guid_disk_io, d->diskio);

    if (fat_mount(&d->fat, blk, base, nblocks * bsize)) {
        d->has_fat = true;
        d->sfs = obj_alloc(16);
        gst64(d->sfs, 0x00010000);
        gst64(d->sfs + 8, FW_FD(P_SFS_OPEN_VOLUME));
        proto_install(h, &guid_simple_fs, d->sfs);
    }
    return d;
}

/* The El Torito boot catalog's EFI entry: start in 2048 byte blocks */
static bool eltorito_efi(BlockBackend *blk, uint64_t *start, uint64_t *len)
{
    uint8_t b[2048];
    uint32_t cat;

    if (blk_pread(blk, 17 * 2048, 2048, b, 0) < 0 || b[0] != 0 ||
        memcmp(b + 1, "CD001", 5) ||
        memcmp(b + 7, "EL TORITO SPECIFICATION", 23)) {
        return false;
    }
    cat = ldl_le_p(b + 0x47);
    if (blk_pread(blk, cat * 2048ull, 2048, b, 0) < 0 || b[0] != 1) {
        return false;
    }
    /* validation entry, then the default entry and sections */
    for (int i = 32; i + 32 <= 2048;) {
        uint8_t *e = b + i;
        bool efi_entry = b[1] == 0xef && i == 32;

        if (e[0] == 0x90 || e[0] == 0x91) {     /* section header */
            int n = lduw_le_p(e + 2);

            if (e[1] == 0xef && n && e[32] == 0x88) {
                e += 32;
                efi_entry = true;
            } else {
                i += 32 * (1 + n);
                if (e[0] == 0x91) {
                    break;
                }
                continue;
            }
        } else if (i > 32) {
            break;
        }
        if (efi_entry && e[0] == 0x88) {
            *start = ldl_le_p(e + 8);
            *len = lduw_le_p(e + 6) * 512ull;
            return true;
        }
        i += 32;
    }
    return false;
}

/* A partition: its handle's device path ends in an HD() node */
static void partition_new(BlockBackend *blk, const DevPath *disk, int num,
                          uint64_t start, uint64_t blocks,
                          const uint8_t sig[16], int mbr_type, int sig_type)
{
    DevPath dp = *disk;
    uint8_t hd[38] = { 0 };

    stl_le_p(hd, num);
    stq_le_p(hd + 4, start);
    stq_le_p(hd + 12, blocks);
    memcpy(hd + 20, sig, 16);
    hd[36] = mbr_type;
    hd[37] = sig_type;
    dp_node(&dp, 4, 1, hd, sizeof(hd));
    dp_end(&dp);
    DPRINTF("partition %d at %" PRIu64 ", %" PRIu64 " blocks\n", num, start,
            blocks);
    disk_new(blk, start * 512, 512, blocks, &dp, false, true);
}

/* GPT, else MBR partitions of a 512-byte-block disk */
static void partitions_init(BlockBackend *blk, uint64_t blocks,
                            const DevPath *disk)
{
    uint8_t b[512];

    if (blk_pread(blk, 512, 512, b, 0) >= 0 && !memcmp(b, "EFI PART", 8)) {
        uint64_t lba = ldq_le_p(b + 72);
        uint32_t n = ldl_le_p(b + 80), esize = ldl_le_p(b + 84);
        static const uint8_t unused[16];

        for (uint32_t i = 0; i < MIN(n, 128) && esize >= 128; i++) {
            uint8_t e[128];

            if (blk_pread(blk, lba * 512 + (uint64_t)i * esize, 128, e,
                          0) < 0 || !memcmp(e, unused, 16)) {
                continue;
            }
            partition_new(blk, disk, i + 1, ldq_le_p(e + 32),
                          ldq_le_p(e + 40) - ldq_le_p(e + 32) + 1, e + 16, 2,
                          2);
        }
        return;
    }
    if (blk_pread(blk, 0, 512, b, 0) >= 0 && lduw_le_p(b + 510) == 0xaa55) {
        for (int i = 0; i < 4; i++) {
            const uint8_t *e = b + 446 + 16 * i;
            uint8_t sig[16] = { 0 };

            if (!e[4] || e[4] == 0xee || !ldl_le_p(e + 12)) {
                continue;
            }
            memcpy(sig, b + 440, 4);
            partition_new(blk, disk, i + 1, ldl_le_p(e + 8), ldl_le_p(e + 12),
                          sig, 1, 1);
        }
    }
}

static void disks_init(void)
{
    BlockBackend *blk = NULL;
    int cd = 0, hd = 0;

    while ((blk = blk_next(blk))) {
        uint8_t b[2048];
        int64_t size;
        DevPath dp = { .n = 0 };

        if (!blk_is_inserted(blk) || (size = blk_getlength(blk)) <= 0) {
            continue;
        }
        dp_acpi(&dp, 0x0a0341d0, 0);            /* PNP0A03 */
        if (blk_pread(blk, 16 * 2048, 2048, b, 0) >= 0 && b[0] == 1 &&
            !memcmp(b + 1, "CD001", 5)) {
            uint64_t start, len;
            uint8_t ata[4] = { 1, cd & 1, 0, 0 };   /* secondary channel */
            DevPath part;

            dp_pci(&dp, 1, 1);
            dp_node(&dp, 3, 1, ata, 4);
            part = dp;
            dp_end(&dp);
            disk_new(blk, 0, 2048, size / 2048, &dp, true, false);
            cd++;

            if (eltorito_efi(blk, &start, &len)) {
                uint8_t cdnode[20];
                FatVol v;

                /* the catalog's sector count is often 1: ask the FAT */
                if (len < 2048 && fat_mount(&v, blk, start * 2048, 0)) {
                    len = v.total_bytes;
                }
                len = ROUND_UP(len, 2048);
                stl_le_p(cdnode, 0);
                stq_le_p(cdnode + 4, start);
                stq_le_p(cdnode + 12, len / 2048);
                dp_node(&part, 4, 2, cdnode, 20);
                dp_end(&part);
                disk_new(blk, start * 2048, 2048, len / 2048, &part, true,
                         true);
            }
        } else {
            uint8_t scsi[4] = { hd, 0, 0, 0 };
            DevPath part;

            dp_pci(&dp, 2, 0);
            dp_node(&dp, 3, 2, scsi, 4);
            part = dp;
            dp_end(&dp);
            disk_new(blk, 0, 512, size / 512, &dp, false, false);
            partitions_init(blk, size / 512, &part);
            hd++;
        }
    }
}

/*
 * Events
 */
static Event *event_by(uint64_t addr)
{
    for (int i = 0; i < EVENT_MAX; i++) {
        if (efi.events[i].used && efi.events[i].addr == addr) {
            return &efi.events[i];
        }
    }
    return NULL;
}

static Event *event_new(uint32_t type)
{
    for (int i = 0; i < EVENT_MAX; i++) {
        Event *e = &efi.events[i];

        if (!e->used) {
            if (!e->addr) {
                e->addr = obj_alloc(16);
            }
            e->used = true;
            e->type = type;
            e->signaled = e->periodic = e->is_key = false;
            e->trigger = 0;
            e->notify = e->context = 0;
            return e;
        }
    }
    return NULL;
}

static bool event_ready(Event *e)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (e->signaled) {
        return true;
    }
    if (e->is_key) {
        return con_key_ready();
    }
    if (e->trigger && now >= e->trigger) {
        e->trigger = e->periodic ? now + e->period : 0;
        e->signaled = true;
        return true;
    }
    return false;
}

/*
 * Variables
 */
static Var *var_find(const uint16_t *name, const Guid *g)
{
    for (int i = 0; i < VAR_MAX; i++) {
        Var *v = &vars[i];

        if (v->used && !memcmp(&v->guid, g, 16) &&
            !memcmp(v->name, name, 2 * (str16len(name) + 1))) {
            return v;
        }
    }
    return NULL;
}

static bool var_set(const uint16_t *name, const Guid *g, uint32_t attr,
                    const void *data, uint32_t size)
{
    Var *v = var_find(name, g);

    if (v && !size) {
        g_free(v->data);
        v->used = false;
        return true;
    }
    if (!size) {
        return true;
    }
    if (!v) {
        for (int i = 0; i < VAR_MAX; i++) {
            if (!vars[i].used) {
                v = &vars[i];
                break;
            }
        }
        if (!v || str16len(name) >= ARRAY_SIZE(v->name)) {
            return false;
        }
        v->used = true;
        v->data = NULL;
        memcpy(v->name, name, 2 * (str16len(name) + 1));
        v->guid = *g;
    }
    g_free(v->data);
    v->data = g_memdup2(data, size);
    v->size = size;
    v->attr = attr;
    return true;
}

/*
 * Non-volatile variables go to the file named by IA64_NVRAM, rewritten on
 * every change: per variable the name's length, the name, the GUID, the
 * attributes, the data's size and the data.
 */
static void nvram_save(void)
{
    const char *file = getenv("IA64_NVRAM");
    FILE *f;

    if (!file || !(f = fopen(file, "wb"))) {
        return;
    }
    for (int i = 0; i < VAR_MAX; i++) {
        Var *v = &vars[i];
        uint16_t nl;

        if (!v->used || !(v->attr & 1)) {
            continue;
        }
        nl = str16len(v->name);
        fwrite(&nl, 2, 1, f);
        fwrite(v->name, 2, nl, f);
        fwrite(v->guid.b, 16, 1, f);
        fwrite(&v->attr, 4, 1, f);
        fwrite(&v->size, 4, 1, f);
        fwrite(v->data, 1, v->size, f);
    }
    fclose(f);
}

static void nvram_load(void)
{
    static bool loaded;
    const char *file = getenv("IA64_NVRAM");
    FILE *f;
    uint16_t nl;

    if (loaded || !file || !(f = fopen(file, "rb"))) {
        return;
    }
    loaded = true;
    while (fread(&nl, 2, 1, f) == 1 && nl < 128) {
        uint16_t name[128];
        Guid g;
        uint32_t attr, size;
        uint8_t *data;

        if (fread(name, 2, nl, f) != nl || fread(g.b, 16, 1, f) != 1 ||
            fread(&attr, 4, 1, f) != 1 || fread(&size, 4, 1, f) != 1 ||
            size > 64 * KiB) {
            break;
        }
        name[nl] = 0;
        data = g_malloc(size + 1);
        if (fread(data, 1, size, f) == size) {
            var_set(name, &g, attr, data, size);
        }
        g_free(data);
    }
    fclose(f);
}

static void var_set_ascii(const char *name, const void *data, uint32_t size)
{
    uint16_t n[64];

    ascii_str16(name, n);
    var_set(n, &guid_global_var, 7, data, size);
}

/*
 * A call that faults on a pointer is repeated, so the variable services
 * fault in every output before they write the first one.
 */
uint64_t efi_get_variable(CPUIA64State *env)
{
    uint16_t name[128];
    Guid g;
    Var *v;
    uint64_t size;
    char s[128];

    efi.env = env;
    gstr16(arg(0), name, ARRAY_SIZE(name));
    g = gguid(arg(1));
    v = var_find(name, &g);
    str16_ascii(name, s, sizeof(s));
    DPRINTF("GetVariable %s%s\n", s, v ? "" : ": not found");
    if (!v) {
        return EFI_NOT_FOUND;
    }
    size = gld64(arg(3));
    if (efi_debug) {                    /* TEMP: TLB against page tables */
        hwaddr tp = pa(arg(3), MMU_DATA_LOAD);
        hwaddr wp = ia64_fw_translate(env, arg(3));

        fprintf(stderr, "efi: size at %" PRIx64 ": tlb pa %" PRIx64 " = %"
                PRIx64 ", walk pa %" PRIx64 " = %" PRIx64 "\n", arg(3), tp,
                address_space_ldq_le(&address_space_memory, tp,
                                     MEMTXATTRS_UNSPECIFIED, NULL), wp,
                wp == -1 ? 0 : address_space_ldq_le(&address_space_memory, wp,
                                                    MEMTXATTRS_UNSPECIFIED, NULL));
    }
    DPRINTF("GetVariable %s: size %" PRIu64 " of %u, attr %" PRIx64 " data %"
            PRIx64 " from %" PRIx64 "\n", s, size, v->size, arg(2), arg(4),
            env->br[0]);
    if (size >= v->size && !arg(4)) {
        if (efi_debug) {                /* TEMP: who asks */
            fprintf(stderr, "efi: NULL data, callers:");
            for (int k = env->rse_ndirty - 1; k >= 0 &&
                 k >= env->rse_ndirty - 64; k--) {
                uint64_t x = env->rse[k];

                if (x >= 0xe000000083000000ull && x < 0xe000000083800000ull) {
                    fprintf(stderr, " [%d]%" PRIx64, env->rse_ndirty - k, x);
                }
            }
            fprintf(stderr, "\n");
        }
        return EFI_INVALID_PARAMETER;
    }
    gprobe(arg(3), 8);
    if (arg(2)) {
        gprobe(arg(2), 4);
    }
    if (size >= v->size) {
        gprobe(arg(4), v->size);
    }
    gst64(arg(3), v->size);
    if (arg(2)) {
        gst32(arg(2), v->attr);
    }
    if (size < v->size) {
        return EFI_BUFFER_TOO_SMALL;
    }
    gwrite(arg(4), v->data, v->size);
    return EFI_SUCCESS;
}

uint64_t efi_get_next_variable(CPUIA64State *env)
{
    uint16_t name[128];
    uint64_t size;
    Guid g;
    int i = 0;
    Var *v;

    efi.env = env;
    size = gld64(arg(0));
    gstr16(arg(1), name, ARRAY_SIZE(name));
    g = gguid(arg(2));
    if (name[0]) {
        v = var_find(name, &g);
        if (!v) {
            return EFI_NOT_FOUND;
        }
        i = v - vars + 1;
    }
    for (; i < VAR_MAX; i++) {
        if (vars[i].used) {
            uint64_t need = 2 * (str16len(vars[i].name) + 1);

            gprobe(arg(0), 8);
            if (size >= need) {
                gprobe(arg(1), need);
                gprobe(arg(2), 16);
            }
            gst64(arg(0), need);
            if (size < need) {
                return EFI_BUFFER_TOO_SMALL;
            }
            for (int k = 0; k < need / 2; k++) {
                gst16(arg(1) + 2 * k, vars[i].name[k]);
            }
            gwrite(arg(2), vars[i].guid.b, 16);
            return EFI_SUCCESS;
        }
    }
    return EFI_NOT_FOUND;
}

uint64_t efi_set_variable(CPUIA64State *env)
{
    uint16_t name[128];
    Guid g;
    uint64_t size;
    g_autofree uint8_t *data = NULL;
    bool ok;
    char s[128];

    efi.env = env;
    gstr16(arg(0), name, ARRAY_SIZE(name));
    g = gguid(arg(1));
    size = arg(3);
    if (size > 64 * KiB) {
        return EFI_OUT_OF_RESOURCES;
    }
    str16_ascii(name, s, sizeof(s));
    DPRINTF("SetVariable %s size %" PRIu64 "\n", s, size);
    data = g_malloc(size + 1);
    gread(arg(4), data, size);
    ok = var_set(name, &g, arg(2), data, size);
    if (ok) {
        nvram_save();
    }
    return ok ? EFI_SUCCESS : EFI_OUT_OF_RESOURCES;
}

/*
 * Boot services
 */
static uint64_t bs_locate(int type, const Guid *g, uint64_t *out, int max,
                          int *n)
{
    *n = 0;
    for (int i = 0; i < efi.nhandles; i++) {
        Handle *h = &efi.handles[i];

        if (!h->n || (type == 2 && !proto_find(h, g))) {
            continue;
        }
        if (type == 1) {
            continue;
        }
        if (*n < max) {
            out[*n] = h->addr;
        }
        (*n)++;
    }
    return *n ? EFI_SUCCESS : EFI_NOT_FOUND;
}

static uint64_t bs_install_config(const Guid *g, uint64_t table)
{
    int i;

    for (i = 0; i < efi.ncfg; i++) {
        if (!memcmp(gguid(FW_CONFIG + 24 * i).b, g, 16)) {
            break;
        }
    }
    if (!table) {
        if (i == efi.ncfg) {
            return EFI_NOT_FOUND;
        }
        for (; i + 1 < efi.ncfg; i++) {
            uint8_t e[24];

            gread(FW_CONFIG + 24 * (i + 1), e, 24);
            gwrite(FW_CONFIG + 24 * i, e, 24);
        }
        efi.ncfg--;
    } else {
        if (i == FW_CONFIG_MAX) {
            return EFI_OUT_OF_RESOURCES;
        }
        gwrite(FW_CONFIG + 24 * i, g->b, 16);
        gst64(FW_CONFIG + 24 * i + 16, table);
        if (i == efi.ncfg) {
            efi.ncfg++;
        }
    }
    gst64(FW_SYSTAB + 104, efi.ncfg);
    return EFI_SUCCESS;
}

/* Find the handle with protocol g whose device path prefixes *dp */
static uint64_t bs_locate_device_path(const Guid *g, uint64_t dpp,
                                      uint64_t devp)
{
    uint64_t dp = gld64(dpp);
    int len = gdp_len(dp), best = -1;
    Handle *found = NULL;
    uint8_t *path = g_malloc(len + 1);

    gread(dp, path, len);
    for (int i = 0; i < efi.nhandles; i++) {
        Handle *h = &efi.handles[i];
        Proto *p = proto_find(h, &guid_device_path);
        int hl;
        uint8_t *hp;

        if (!p || !proto_find(h, g)) {
            continue;
        }
        hl = gdp_len(p->iface);
        if (hl > len || hl <= best) {
            continue;
        }
        hp = g_malloc(hl + 1);
        gread(p->iface, hp, hl);
        if (!memcmp(hp, path, hl)) {
            best = hl;
            found = h;
        }
        g_free(hp);
    }
    g_free(path);
    if (!found) {
        return EFI_NOT_FOUND;
    }
    gst64(dpp, dp + best);
    gst64(devp, found->addr);
    return EFI_SUCCESS;
}

static uint64_t bs_load_image(uint64_t parent, uint64_t fp, uint64_t src,
                              uint64_t srclen, uint64_t handlep)
{
    uint64_t device = 0, rest = fp, st, len = 0, filepath = 0;
    uint8_t *buf = NULL;
    Handle *h;

    if (fp) {
        uint64_t devp = obj_alloc(16);

        gst64(devp, fp);
        if (bs_locate_device_path(&guid_simple_fs, devp, devp + 8) ==
            EFI_SUCCESS) {
            rest = gld64(devp);
            device = gld64(devp + 8);
        } else if (bs_locate_device_path(&guid_block_io, devp, devp + 8) ==
                   EFI_SUCCESS) {
            rest = gld64(devp);
            device = gld64(devp + 8);
        }
    }
    if (src) {
        buf = g_malloc(srclen + 1);
        gread(src, buf, srclen);
        len = srclen;
    } else {
        uint16_t path[512];
        int n = 0;
        Disk *d = NULL;

        h = handle_find(device);
        for (int i = 0; h && i < efi.ndisks; i++) {
            if (efi.disks[i].handle == device && efi.disks[i].has_fat) {
                d = &efi.disks[i];
            }
        }
        if (!d) {
            return EFI_NOT_FOUND;
        }
        /* concatenate the file path nodes */
        for (uint64_t at = rest; gld8(at) == 4 && gld8(at + 1) == 4;
             at += gld16(at + 2)) {
            int k = (gld16(at + 2) - 4) / 2;

            if (n && path[n - 1] != '\\') {
                path[n++] = '\\';
            }
            for (int i = 0; i < k && n < ARRAY_SIZE(path) - 1; i++) {
                uint16_t c = gld16(at + 4 + 2 * i);

                if (!c) {
                    break;
                }
                path[n++] = c;
            }
        }
        path[n] = 0;
        buf = fat_slurp(d, path, &len);
        if (!buf) {
            return EFI_NOT_FOUND;
        }
    }
    if (rest) {
        int l = gdp_len(rest);

        filepath = obj_alloc(l + 4);
        for (int i = 0; i < l; i++) {
            gst8(filepath + i, gld8(rest + i));
        }
        gst32(filepath + l, 0x0004ff7f);
    }
    st = image_create(buf, len, parent, device, filepath, NULL, 0, &len);
    g_free(buf);
    if (st == EFI_SUCCESS) {
        gst64(handlep, len);
    }
    return st;
}

static void bs_call(CPUIA64State *env, unsigned n)
{
    uint64_t st = EFI_SUCCESS;

    if (efi.exited) {
        fw_ret(env, EFI_UNSUPPORTED, 0, 0, 0);
        return;
    }
    switch (n) {
    case BS_RAISE_TPL: {
        uint64_t old = efi.tpl;

        efi.tpl = arg(0);
        st = old;
        break;
    }
    case BS_RESTORE_TPL:
        efi.tpl = arg(0);
        break;
    case BS_ALLOCATE_PAGES: {
        uint64_t addr = gld64(arg(3));

        if (arg(1) >= EFI_MAX_MEMORY_TYPE && arg(1) < 0x80000000ull) {
            st = EFI_INVALID_PARAMETER;
            break;
        }
        st = mm_alloc(arg(0), arg(1), arg(2), &addr);
        if (st == EFI_SUCCESS) {
            gst64(arg(3), addr);
        }
        DPRINTF("AllocatePages(%d, %d, 0x%" PRIx64 ") = 0x%" PRIx64
                " -> 0x%" PRIx64 "\n", (int)arg(0), (int)arg(1), arg(2),
                addr, st);
        break;
    }
    case BS_FREE_PAGES:
        mm_free(arg(0), arg(1));
        break;
    case BS_GET_MEMORY_MAP: {
        MemDesc md[MM_MAX + 16];
        uint64_t size = gld64(arg(0)), map = arg(1);
        int nd = 0;

        for (int i = 0; i < efi.mm_n; i++) {
            md[nd++] = (MemDesc){ efi.mm[i].type, efi.mm[i].start,
                                  efi.mm[i].end,
                                  EFI_MEM_UC | EFI_MEM_WC | EFI_MEM_WT |
                                  EFI_MEM_WB };
            if (efi.mm[i].type == EFI_RUNTIME_CODE ||
                efi.mm[i].type == EFI_RUNTIME_DATA) {
                md[nd - 1].attr |= EFI_MEM_RUNTIME;
            }
        }
        nd += virt_fw_memmap(md + nd);
        gst64(arg(0), 40 * nd);
        if (arg(2)) {
            gst64(arg(2), efi.map_key);
        }
        gst64(arg(3), 40);
        gst32(arg(4), 1);
        if (size < 40 * nd) {
            st = EFI_BUFFER_TOO_SMALL;
            break;
        }
        for (int i = 0; i < nd; i++) {
            uint64_t d = map + 40 * i;

            gst32(d, md[i].type);
            gst32(d + 4, 0);
            gst64(d + 8, md[i].start);
            gst64(d + 16, 0);
            gst64(d + 24, (md[i].end - md[i].start) / PAGE);
            gst64(d + 32, md[i].attr);
        }
        break;
    }
    case BS_ALLOCATE_POOL: {
        uint64_t buf = pool_alloc(arg(0), arg(1));

        if (!buf) {
            st = EFI_OUT_OF_RESOURCES;
        } else {
            gst64(arg(2), buf);
        }
        break;
    }
    case BS_FREE_POOL:
        st = pool_free(arg(0));
        break;
    case BS_CREATE_EVENT: {
        Event *e = event_new(arg(0));

        if (!e) {
            st = EFI_OUT_OF_RESOURCES;
        } else {
            e->notify = arg(2);
            e->context = arg(3);
            if (e->notify && e->type != EVT_SIGNAL_VIRTUAL_ADDRESS_CHANGE) {
                warn_report_once("ia64 efi: event notification functions "
                                 "are only called on the virtual address "
                                 "change");
            }
            gst64(arg(4), e->addr);
        }
        break;
    }
    case BS_SET_TIMER: {
        Event *e = event_by(arg(0));
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        int64_t ns = arg(2) * 100;

        if (!e) {
            st = EFI_INVALID_PARAMETER;
            break;
        }
        e->signaled = false;
        e->periodic = arg(1) == 1;
        e->period = MAX(ns, 1000);
        e->trigger = arg(1) == 0 ? 0 : now + (e->periodic ? e->period : ns);
        if (arg(1) == 2 && !ns) {
            e->trigger = now;
        }
        break;
    }
    case BS_WAIT_FOR_EVENT: {
        uint64_t num = arg(0), evs = arg(1);
        bool key = false;

        for (uint64_t i = 0; i < num; i++) {
            Event *e = event_by(gld64(evs + 8 * i));

            if (!e) {
                gst64(arg(2), i);
                st = EFI_INVALID_PARAMETER;
                goto done;
            }
            if (event_ready(e)) {
                e->signaled = false;
                gst64(arg(2), i);
                goto done;
            }
            key |= e->is_key;
        }
        /*
         * The CD's boot program asks to press a key and boots the next
         * entry when none comes: answer the first such wait with Enter.
         */
        if (key && !autokey_done) {
            autokey_done = true;
            efi.inbuf[efi.inlen++] = '\r';
        }
        /* nothing yet: wait by running the call again */
        env->ip = FW_STUB(FW_CALL_BS + BS_WAIT_FOR_EVENT);
        env->ri = 0;
        return;
    }
    case BS_SIGNAL_EVENT: {
        Event *e = event_by(arg(0));

        if (e) {
            e->signaled = true;
        }
        break;
    }
    case BS_CLOSE_EVENT: {
        Event *e = event_by(arg(0));

        if (e && !e->is_key) {
            e->used = false;
        }
        break;
    }
    case BS_CHECK_EVENT: {
        Event *e = event_by(arg(0));

        if (!e) {
            st = EFI_INVALID_PARAMETER;
        } else if (event_ready(e)) {
            e->signaled = false;
        } else {
            st = EFI_NOT_READY;
        }
        break;
    }
    case BS_INSTALL_PROTOCOL: {
        Handle *h = handle_find(gld64(arg(0)));
        Guid g = gguid(arg(1));

        if (!h) {
            if (gld64(arg(0))) {
                st = EFI_INVALID_PARAMETER;
                break;
            }
            h = handle_new();
            gst64(arg(0), h->addr);
        }
        st = proto_install(h, &g, arg(3));
        break;
    }
    case BS_REINSTALL_PROTOCOL: {
        Handle *h = handle_find(arg(0));
        Guid g = gguid(arg(1));
        Proto *p = proto_find(h, &g);

        if (!p || p->iface != arg(2)) {
            st = EFI_NOT_FOUND;
        } else {
            p->iface = arg(3);
        }
        break;
    }
    case BS_UNINSTALL_PROTOCOL: {
        Guid g = gguid(arg(1));

        st = proto_uninstall(handle_find(arg(0)), &g, arg(2));
        break;
    }
    case BS_HANDLE_PROTOCOL:
    case BS_OPEN_PROTOCOL: {
        Guid g = gguid(arg(1));
        Proto *p = proto_find(handle_find(arg(0)), &g);
        char s[40];

        guid_str(&g, s);
        DPRINTF("%s(0x%" PRIx64 ", %s) -> %s\n",
                n == BS_HANDLE_PROTOCOL ? "HandleProtocol" : "OpenProtocol",
                arg(0), s, p ? "found" : "unsupported");
        if (!p) {
            st = EFI_UNSUPPORTED;
        } else if (arg(2)) {
            gst64(arg(2), p->iface);
        }
        break;
    }
    case BS_REGISTER_PROTOCOL_NOTIFY:
        gst64(arg(2), obj_alloc(16));
        break;
    case BS_LOCATE_HANDLE: {
        Guid g = arg(1) ? gguid(arg(1)) : (Guid){};
        uint64_t size = gld64(arg(3)), hs[HANDLE_MAX];
        int found;
        char s[40];

        st = bs_locate(arg(0), &g, hs, HANDLE_MAX, &found);
        guid_str(&g, s);
        DPRINTF("LocateHandle(%d, %s) -> %d\n", (int)arg(0), s, found);
        if (st != EFI_SUCCESS) {
            break;
        }
        gst64(arg(3), 8 * found);
        if (size < 8 * found) {
            st = EFI_BUFFER_TOO_SMALL;
            break;
        }
        for (int i = 0; i < found; i++) {
            gst64(arg(4) + 8 * i, hs[i]);
        }
        break;
    }
    case BS_LOCATE_HANDLE_BUFFER: {
        Guid g = arg(1) ? gguid(arg(1)) : (Guid){};
        uint64_t hs[HANDLE_MAX], buf;
        int found;

        st = bs_locate(arg(0), &g, hs, HANDLE_MAX, &found);
        if (st != EFI_SUCCESS) {
            break;
        }
        buf = pool_alloc(EFI_BS_DATA, 8 * found);
        for (int i = 0; i < found; i++) {
            gst64(buf + 8 * i, hs[i]);
        }
        gst64(arg(3), found);
        gst64(arg(4), buf);
        break;
    }
    case BS_LOCATE_DEVICE_PATH: {
        Guid g = gguid(arg(0));

        st = bs_locate_device_path(&g, arg(1), arg(2));
        break;
    }
    case BS_LOCATE_PROTOCOL: {
        Guid g = gguid(arg(0));
        uint64_t hs[HANDLE_MAX];
        int found;
        char s[40];

        guid_str(&g, s);
        st = bs_locate(2, &g, hs, HANDLE_MAX, &found);
        DPRINTF("LocateProtocol(%s) -> %d\n", s, found);
        if (st == EFI_SUCCESS) {
            gst64(arg(2), proto_find(handle_find(hs[0]), &g)->iface);
        }
        break;
    }
    case BS_INSTALL_CONFIG_TABLE: {
        Guid g = gguid(arg(0));

        st = bs_install_config(&g, arg(1));
        break;
    }
    case BS_LOAD_IMAGE:
        st = bs_load_image(arg(1), arg(2), arg(3), arg(4), arg(5));
        DPRINTF("LoadImage -> 0x%" PRIx64 "\n", st);
        break;
    case BS_START_IMAGE: {
        Image *im = image_find(arg(0));

        if (!im || im->started) {
            st = EFI_INVALID_PARAMETER;
            break;
        }
        im->exit_size_ptr = arg(1);
        im->exit_data_ptr = arg(2);
        DPRINTF("StartImage 0x%" PRIx64 "\n", im->base);
        snap_save(env, &im->snap);
        image_enter(env, im);
        return;
    }
    case BS_EXIT: {
        Image *im = image_find(arg(0));

        if (!im || !im->started) {
            st = EFI_INVALID_PARAMETER;
            break;
        }
        image_finish(env, im, arg(1));
        return;
    }
    case BS_UNLOAD_IMAGE: {
        Image *im = image_find(arg(0));

        if (!im || im->started) {
            st = EFI_INVALID_PARAMETER;
        } else {
            image_unload(im);
        }
        break;
    }
    case BS_EXIT_BOOT_SERVICES:
        if (arg(1) != efi.map_key) {
            st = EFI_INVALID_PARAMETER;
            DPRINTF("ExitBootServices: stale map key\n");
            break;
        }
        DPRINTF("ExitBootServices\n");
        efi.exited = true;
        break;
    case BS_GET_NEXT_MONOTONIC_COUNT:
        gst64(arg(0), ++efi.monotonic);
        break;
    case BS_STALL:
    case BS_SET_WATCHDOG_TIMER:
    case BS_CONNECT_CONTROLLER:
    case BS_DISCONNECT_CONTROLLER:
    case BS_CLOSE_PROTOCOL:
        break;
    case BS_OPEN_PROTOCOL_INFORMATION:
        gst64(arg(2), 0);
        gst64(arg(3), 0);
        break;
    case BS_PROTOCOLS_PER_HANDLE: {
        Handle *h = handle_find(arg(0));
        uint64_t buf;

        if (!h) {
            st = EFI_INVALID_PARAMETER;
            break;
        }
        buf = pool_alloc(EFI_BS_DATA, 8 * h->n + 16 * h->n);
        for (int i = 0; i < h->n; i++) {
            uint64_t g = buf + 8 * h->n + 16 * i;

            gwrite(g, h->p[i].guid.b, 16);
            gst64(buf + 8 * i, g);
        }
        gst64(arg(1), buf);
        gst64(arg(2), h->n);
        break;
    }
    case BS_INSTALL_MULTIPLE:
    case BS_UNINSTALL_MULTIPLE: {
        Handle *h = handle_find(gld64(arg(0)));

        if (!h && n == BS_INSTALL_MULTIPLE && !gld64(arg(0))) {
            h = handle_new();
            gst64(arg(0), h->addr);
        }
        if (!h) {
            st = EFI_INVALID_PARAMETER;
            break;
        }
        for (int i = 1; arg(i) && i < 32; i += 2) {
            Guid g = gguid(arg(i));

            st = n == BS_INSTALL_MULTIPLE ?
                 proto_install(h, &g, arg(i + 1)) :
                 proto_uninstall(h, &g, arg(i + 1));
            if (st != EFI_SUCCESS) {
                break;
            }
        }
        break;
    }
    case BS_CALCULATE_CRC32: {
        uint64_t len = arg(1);
        uint8_t *buf = g_malloc(len + 1);

        gread(arg(0), buf, len);
        gst32(arg(2), crc32(0, buf, len));
        g_free(buf);
        break;
    }
    case BS_COPY_MEM: {
        uint64_t len = arg(2);
        uint8_t *buf = g_malloc(len + 1);

        gread(arg(1), buf, len);
        gwrite(arg(0), buf, len);
        g_free(buf);
        break;
    }
    case BS_SET_MEM: {
        uint64_t len = arg(1);
        uint8_t *buf = g_malloc(len + 1);

        memset(buf, arg(2), len);
        gwrite(arg(0), buf, len);
        g_free(buf);
        break;
    }
    default:
        st = EFI_UNSUPPORTED;
        break;
    }
done:
    fw_ret(env, st, 0, 0, 0);
}

static void proto_call(CPUIA64State *env, unsigned n)
{
    uint64_t st = EFI_SUCCESS;

    if (n >= P_TEXTOUT_RESET && n <= P_TEXTOUT_ENABLE_CURSOR) {
        textout_call(env, n);
        return;
    }
    if (n >= P_BLKIO_RESET && n <= P_BLKIO_FLUSH) {
        blkio_call(env, n);
        return;
    }
    if (n == P_DISKIO_READ || n == P_DISKIO_WRITE) {
        diskio_call(env, n);
        return;
    }
    if (n >= P_FILE_OPEN && n <= P_FILE_FLUSH) {
        file_call(env, n);
        return;
    }
    switch (n) {
    case P_TEXTIN_RESET:
        con_poll();
        efi.inlen = 0;
        break;
    case P_TEXTIN_READ_KEY: {
        uint16_t scan, uc;

        if (!con_key(&scan, &uc)) {
            st = EFI_NOT_READY;
            break;
        }
        gst16(arg(1), scan);
        gst16(arg(1) + 2, uc);
        break;
    }
    case P_SFS_OPEN_VOLUME: {
        Disk *d = disk_by(arg(0), 2);
        FatNode root;
        File *f;

        if (!d) {
            st = EFI_INVALID_PARAMETER;
            break;
        }
        root = fat_root(&d->fat);
        f = file_new(d, &root);
        if (!f) {
            st = EFI_OUT_OF_RESOURCES;
            break;
        }
        gst64(arg(1), f->addr);
        break;
    }
    case P_IMAGE_UNLOAD:
        st = EFI_UNSUPPORTED;
        break;
    case P_FPSWA:
        /* no software assistance: the FPU computes every case itself */
        warn_report_once("ia64 efi: FPSWA called");
        fw_ret(env, -1, 0, 0, 0);
        return;
    default:
        st = EFI_UNSUPPORTED;
        break;
    }
    fw_ret(env, st, 0, 0, 0);
}

/*
 * SetVirtualAddressMap.  As in Intel's EFI, the notification functions of
 * EVT_SIGNAL_VIRTUAL_ADDRESS_CHANGE events run first, in physical mode,
 * and convert their drivers' pointers with ConvertPointer; then runtime
 * drivers are relocated to their new addresses.  The notification
 * functions are the caller's to run: each returns to P_NOTIFY_RETURN, and
 * the call completes after the last one.
 */
static Event *notify_find(int from)
{
    for (int i = from; i < EVENT_MAX; i++) {
        Event *e = &efi.events[i];

        if (e->used && e->notify &&
            e->type == EVT_SIGNAL_VIRTUAL_ADDRESS_CHANGE) {
            efi.notify_next = i + 1;
            return e;
        }
    }
    return NULL;
}

static void notify_enter(CPUIA64State *env, Event *e)
{
    uint64_t code = gld64(e->notify), gp = gld64(e->notify + 8);

    DPRINTF("virtual address change: notify 0x%" PRIx64 "\n", code);
    if ((env->cfm & 0x7f) < 2) {
        env->cfm = 2;
    }
    env->gr[32] = e->addr;
    env->gr[33] = e->context;
    env->nat[32] = env->nat[33] = 0;
    env->ar[AR_PFS] = (env->cfm & ((1ull << 38) - 1)) |
                      ((env->ar[AR_EC] & 0x3f) << 52);
    env->gr[1] = gp;
    env->nat[1] = 0;
    env->br[0] = FW_STUB(P_NOTIFY_RETURN);
    env->ip = code & ~15ull;
    env->ri = 0;
}

/* Begin SetVirtualAddressMap; false when it can complete right away */
bool efi_svam_begin(CPUIA64State *env, uint64_t map, uint64_t size,
                    uint64_t dsize)
{
    Event *e;

    efi.env = env;
    efi.nvmap = 0;
    for (uint64_t off = 0; dsize >= 40 && off + dsize <= size &&
         efi.nvmap < ARRAY_SIZE(efi.vmap); off += dsize) {
        uint32_t attr_hi = gld32(map + off + 36);

        if (attr_hi & 0x80000000) {             /* EFI_MEMORY_RUNTIME */
            efi.vmap[efi.nvmap].phys = gld64(map + off + 8);
            efi.vmap[efi.nvmap].virt = gld64(map + off + 16);
            efi.vmap[efi.nvmap].size = gld64(map + off + 24) * PAGE;
            efi.nvmap++;
        }
    }
    e = notify_find(0);
    if (!e) {
        return false;
    }
    snap_save(env, &efi.notify_snap);
    notify_enter(env, e);
    return true;
}

static void notify_return(CPUIA64State *env)
{
    Event *e = notify_find(efi.notify_next);

    snap_restore(env, &efi.notify_snap);
    if (e) {
        notify_enter(env, e);
    } else {
        virt_svam_complete(env);
    }
}

static int64_t vmap_delta(uint64_t pa)
{
    for (int i = 0; i < efi.nvmap; i++) {
        if (pa >= efi.vmap[i].phys && pa - efi.vmap[i].phys < efi.vmap[i].size) {
            return efi.vmap[i].virt - efi.vmap[i].phys;
        }
    }
    return 0;
}

/* ConvertPointer(DebugDisposition, Address) */
uint64_t efi_convert_pointer(CPUIA64State *env)
{
    uint64_t p, d;

    efi.env = env;
    p = gld64(arg(1));
    if (!p) {
        return (arg(0) & 1) ? EFI_SUCCESS : EFI_INVALID_PARAMETER;
    }
    for (int i = 0; i < efi.nvmap; i++) {
        if (p >= efi.vmap[i].phys && p - efi.vmap[i].phys < efi.vmap[i].size) {
            d = efi.vmap[i].virt - efi.vmap[i].phys;
            gst64(arg(1), p + d);
            return EFI_SUCCESS;
        }
    }
    return EFI_NOT_FOUND;
}

/*
 * The firmware's runtime part moved: so does the FPSWA entry, and runtime
 * drivers follow their memory.
 */
void efi_virtual_address_change(uint64_t phys_start, uint64_t virt_start)
{
    address_space_stq_le(&address_space_memory, FW_FPSWA + 8,
                         FW_FD(P_FPSWA) - phys_start + virt_start,
                         MEMTXATTRS_UNSPECIFIED, NULL);
    for (int i = 0; i < IMAGE_MAX; i++) {
        Image *im = &efi.images[i];
        int64_t delta = im->used ? vmap_delta(im->base) : 0;
        uint8_t *img;

        if (!delta || !im->reloc) {
            continue;
        }
        DPRINTF("runtime image at 0x%" PRIx64 " moves by 0x%" PRIx64 "\n",
                im->base, delta);
        img = g_malloc(im->size);
        address_space_read(&address_space_memory, im->base,
                           MEMTXATTRS_UNSPECIFIED, img, im->size);
        pe_relocate(img, im->size, im->reloc, im->reloc_size, delta,
                    im->loaded);
        address_space_write(&address_space_memory, im->base,
                            MEMTXATTRS_UNSPECIFIED, img, im->size);
        g_free(img);
    }
}

/*
 * Boot manager
 */
static void systab_crc(uint64_t table)
{
    uint32_t size = gld32(table + 12);
    uint8_t *buf = g_malloc(size);

    gst32(table + 16, 0);
    gread(table, buf, size);
    gst32(table + 16, crc32(0, buf, size));
    g_free(buf);
}

static void efi_init(CPUIA64State *env)
{
    MachineState *ms = MACHINE(qdev_get_machine());
    Handle *h;
    uint64_t bs, fpswa;
    DevPath con = { .n = 0 };
    uint8_t opt[256];
    uint16_t desc[16];
    uint16_t boot0 = 0;
    int ol;

    efi_debug = getenv("IA64_EFI_DEBUG") != NULL;
    memset(&efi, 0, sizeof(efi));
    efi.env = env;
    efi.tpl = 4;                                /* TPL_APPLICATION */
    efi.mm[0] = (MemRange){ 0, ms->ram_size, EFI_CONVENTIONAL };
    efi.mm_n = 1;
    /*
     * Real mode vectors, BIOS data, VGA window and ROM: see virt.c.  Not
     * the whole legacy area up to 1 MB: setupldr then lays its data out
     * differently and the kernel touches some above the 64 MB that the
     * loader's translation registers cover once it reuses dtr[4].
     */
    mm_set(0, PAGE, EFI_RESERVED);
    mm_set(VGA_WINDOW_BASE, VGA_ROM_BASE + VGA_ROM_SIZE, EFI_RESERVED);
    efi.ncfg = gld64(FW_SYSTAB + 104);

    virt_com1_init();
    virt_pci_assign();

    /* console: COM1 */
    h = handle_new();
    efi.con_handle = h->addr;
    efi.conin = obj_alloc(24);
    obj_functions(efi.conin, P_TEXTIN_RESET, 2);
    {
        Event *e = event_new(0x100);            /* EVT_NOTIFY_WAIT */

        e->is_key = true;
        efi.key_event = e->addr;
        gst64(efi.conin + 16, e->addr);
    }
    efi.conout_mode = obj_alloc(24);
    gst32(efi.conout_mode, 1);                  /* MaxMode */
    gst32(efi.conout_mode + 8, 0x07);
    gst8(efi.conout_mode + 20, 1);
    efi.conout = obj_alloc(80);
    obj_functions(efi.conout, P_TEXTOUT_RESET, 9);
    gst64(efi.conout + 72, efi.conout_mode);
    dp_acpi(&con, 0x050141d0, 0);               /* PNP0501 */
    dp_uart(&con);
    dp_end(&con);
    proto_install(h, &guid_device_path, dp_store(&con));
    proto_install(h, &guid_text_in, efi.conin);
    proto_install(h, &guid_text_out, efi.conout);

    /*
     * Floating point software assistance: a runtime interface, the OS keeps
     * it and SetVirtualAddressMap converts its pointer.
     */
    h = handle_new();
    fpswa = FW_FPSWA;
    gst32(fpswa, 0x00010000);
    gst32(fpswa + 4, 0);
    gst64(fpswa + 8, FW_FD(P_FPSWA));
    proto_install(h, &guid_fpswa, fpswa);

    disks_init();

    /* boot services table */
    bs = obj_alloc(24 + 8 * BS_NUM);
    gst64(bs, 0x56524553544f4f42ull);           /* "BOOTSERV" */
    gst32(bs + 8, (1 << 16) | 10);
    gst32(bs + 12, 24 + 8 * BS_NUM);
    obj_functions(bs + 24, FW_CALL_BS, BS_NUM);
    systab_crc(bs);

    gst64(FW_SYSTAB + 40, efi.con_handle);
    gst64(FW_SYSTAB + 48, efi.conin);
    gst64(FW_SYSTAB + 56, efi.con_handle);
    gst64(FW_SYSTAB + 64, efi.conout);
    gst64(FW_SYSTAB + 72, efi.con_handle);
    gst64(FW_SYSTAB + 80, efi.conout);
    gst64(FW_SYSTAB + 96, bs);
    systab_crc(FW_SYSTAB);
    systab_crc(FW_RUNTIME);

    /* variables a firmware sets up */
    nvram_load();
    var_set_ascii("ConOut", con.b, con.n);
    var_set_ascii("ConIn", con.b, con.n);
    var_set_ascii("ErrOut", con.b, con.n);
    var_set_ascii("ConOutDev", con.b, con.n);
    var_set_ascii("ConInDev", con.b, con.n);
    var_set_ascii("ErrOutDev", con.b, con.n);
    var_set_ascii("Lang", "eng", 3);
    {
        uint16_t timeout = 0;

        var_set_ascii("Timeout", &timeout, 2);
    }
    /* Boot0000: the first CD */
    for (int i = 0; i < efi.ndisks; i++) {
        Disk *d = &efi.disks[i];
        int dl;

        if (!d->cdrom || d->partition) {
            continue;
        }
        stl_le_p(opt, 1);                       /* LOAD_OPTION_ACTIVE */
        ascii_str16("CD-ROM", desc);
        ol = 6;
        for (int k = 0; k <= str16len(desc); k++) {
            stw_le_p(opt + ol, desc[k]);
            ol += 2;
        }
        dl = gdp_len(d->dpath) + 4;
        for (int k = 0; k < dl; k++) {
            opt[ol + k] = gld8(d->dpath + k);
        }
        stw_le_p(opt + 4, dl);
        ol += dl;
        if (!var_find((uint16_t[]){ 'B', 'o', 'o', 't', '0', '0', '0', '0',
                                    0 }, &guid_global_var)) {
            var_set_ascii("Boot0000", opt, ol);
            var_set_ascii("BootOrder", &boot0, 2);
        }
        break;
    }
}

/* Load a boot application from a FAT volume and run it */
static bool boot_image(CPUIA64State *env, Disk *d, const uint16_t *path,
                       const void *options, uint32_t options_size,
                       uint16_t option)
{
    uint64_t len, handle, stack, bstore, st;
    DevPath fp = { .n = 0 };
    uint8_t *buf;
    Image *im;
    char s[256];

    str16_ascii(path, s, sizeof(s));
    if (!d->has_fat || !(buf = fat_slurp(d, path, &len))) {
        DPRINTF("boot: %s not found\n", s);
        return false;
    }
    dp_file(&fp, path);
    dp_end(&fp);
    st = image_create(buf, len, 0, d->handle, dp_store(&fp), options,
                      options_size, &handle);
    g_free(buf);
    if (st != EFI_SUCCESS) {
        error_report("ia64 efi: %s: load error 0x%" PRIx64, s, st);
        return false;
    }
    im = image_find(handle);
    im->boot = true;

    /* a memory stack and a register backing store */
    mm_alloc(0, EFI_BS_DATA, 64, &stack);
    mm_alloc(0, EFI_BS_DATA, 16, &bstore);
    env->gr[12] = stack + 64 * PAGE - 16;
    env->ar[AR_BSPSTORE] = bstore;
    env->ar[AR_RNAT] = 0;
    env->ar[AR_RSC] = 0;
    env->rse_ndirty = 0;
    env->cfm = 2;
    /*
     * The IVT and the stubs must stay reachable when the loader turns
     * translation on: the last translation registers map the firmware.
     */
    env->cr[CR_IVA] = FW_IVT;
    ia64_tr_insert(env, true, 7, FW_BASE, FW_BASE | 0x661, 24 << 2);
    ia64_tr_insert(env, false, 7, FW_BASE, FW_BASE | 0x661, 24 << 2);
    snap_save(env, &im->snap);
    var_set_ascii("BootCurrent", &option, 2);
    DPRINTF("booting %s, option %04x\n", s, option);
    image_enter(env, im);
    return true;
}

/*
 * The volume and file a boot option's device path names.  The partition is
 * found by its HD() node, so that short-form paths work; a device without
 * a file system stands for its first partition that has one (the CD's El
 * Torito image).
 */
static Disk *boot_resolve(const uint8_t *dp, int len, uint16_t *path, int max)
{
    Disk *found = NULL;
    int n = 0, at = 0;

    path[0] = 0;
    while (at + 4 <= len) {
        uint8_t type = dp[at], sub = dp[at + 1];
        int nl = lduw_le_p(dp + at + 2);

        if (nl < 4 || at + nl > len || type == 0x7f) {
            break;
        }
        if (type == 4 && sub == 1 && nl >= 42) {
            for (int i = 0; i < efi.ndisks; i++) {
                Disk *d = &efi.disks[i];

                /*
                 * Partition number, signature and its type: Windows XP's
                 * setup writes start and size in bytes, not blocks.
                 */
                for (int k = 0; d->partition && k + 42 <= d->dp.n; k++) {
                    const uint8_t *h = d->dp.b + k;

                    if (h[0] == 4 && h[1] == 1 &&
                        !memcmp(h + 4, dp + at + 4, 4) &&
                        !memcmp(h + 24, dp + at + 24, 16) &&
                        h[41] == dp[at + 41]) {
                        found = d;
                    }
                }
            }
        } else if (type == 4 && sub == 4) {
            if (n && path[n - 1] != '\\') {
                path[n++] = '\\';
            }
            for (int i = 4; i + 1 < nl && n < max - 1; i += 2) {
                uint16_t c = lduw_le_p(dp + at + i);

                if (!c) {
                    break;
                }
                path[n++] = c;
            }
            path[n] = 0;
        }
        at += nl;
    }
    if (!found) {
        /* a full path to a device */
        for (int i = 0; i < efi.ndisks; i++) {
            Disk *d = &efi.disks[i];
            int dl = d->dp.n - 4;

            if (dl <= len && !memcmp(d->dp.b, dp, dl) &&
                (!found || dl > found->dp.n - 4)) {
                found = d;
            }
        }
    }
    if (found && !found->has_fat) {
        for (int i = 0; i < efi.ndisks; i++) {
            if (efi.disks[i].blk == found->blk && efi.disks[i].has_fat) {
                return &efi.disks[i];
            }
        }
    }
    return found;
}

/*
 * The boot manager: the entries of BootOrder, from where it stopped, and
 * then \EFI\BOOT\BOOTIA64.EFI on every volume.
 */
static void efi_boot(CPUIA64State *env)
{
    uint16_t name[16], order[64], defpath[64];
    Var *ov, *nv;
    int norder = 0;

    ascii_str16("\\EFI\\BOOT\\BOOTIA64.EFI", defpath);
    ascii_str16("BootOrder", name);
    ov = getenv("IA64_NO_BOOTORDER") ? NULL : var_find(name, &guid_global_var);
    /* BootNext goes first, for this boot only */
    if (efi.boot_next == 0) {
        ascii_str16("BootNext", name);
        nv = var_find(name, &guid_global_var);
        if (nv && nv->size >= 2) {
            efi.boot_once = lduw_le_p(nv->data) + 1;
            var_set(name, &guid_global_var, 0, NULL, 0);
            nvram_save();
        }
    }
    if (efi.boot_once) {
        order[norder++] = efi.boot_once - 1;
    }
    if (ov) {
        int n = MIN(ov->size / 2, ARRAY_SIZE(order) - norder);

        memcpy(order + norder, ov->data, 2 * n);
        norder += n;
    }
    while (efi.boot_next < norder) {
        uint16_t num = lduw_le_p(order + efi.boot_next++);
        char vn[16];
        Var *v;
        uint16_t path[260];
        const uint8_t *o;
        int dl, at;
        Disk *d;

        snprintf(vn, sizeof(vn), "Boot%04X", num);
        ascii_str16(vn, name);
        v = var_find(name, &guid_global_var);
        if (!v || v->size < 8 || !(ldl_le_p(v->data) & 1)) {
            continue;
        }
        o = v->data;
        dl = lduw_le_p(o + 4);
        for (at = 6; at + 1 < v->size && lduw_le_p(o + at); at += 2) {
        }
        at += 2;
        if (at + dl > v->size) {
            continue;
        }
        d = boot_resolve(o + at, dl, path, ARRAY_SIZE(path));
        DPRINTF("boot option %04x: %s\n", num, d ? "found" : "no device");
        if (d && boot_image(env, d, path[0] ? path : defpath, o + at + dl,
                            v->size - at - dl, num)) {
            return;
        }
    }
    for (int i = efi.boot_next - norder; i < efi.ndisks; i++) {
        efi.boot_next++;
        if (boot_image(env, &efi.disks[i], defpath, NULL, 0, 0)) {
            return;
        }
    }
    error_report("ia64 efi: no bootable device");
    con_printf("\r\nNo bootable device.\r\n");
    qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
    efi.dead = true;
    env->ip = FW_STUB(FW_CALL_BOOT);
    env->ri = 0;
}

/*
 * The firmware's IVT, in use until the OS loader installs its own.  The
 * Windows loader runs with data translation on and leaves its TLB misses
 * to the firmware, which maps everything one to one: RAM and the firmware
 * write-back, the rest uncached, in 16 MB pages.  Anything else is fatal.
 */
static void ivt_call(CPUIA64State *env)
{
    MachineState *ms = MACHINE(qdev_get_machine());
    uint64_t vec = env->ip - FW_IVT;
    uint64_t ifa = env->cr[CR_IFA], isr = env->cr[CR_ISR];

    if ((isr & ISR_SP) && vec != IVT_VHPT && vec != IVT_DTLB &&
        vec != IVT_ALT_DTLB) {
        /*
         * A speculative load's fault, the unaligned reference of the SP2
         * loader's ld.s say: defer it, as an OS does, by repeating the
         * load with psr.ed; the loader's chk.s recovers
         */
        env->cr[CR_IPSR] |= PSR_ED;
        ia64_rfi(env, 0);
        return;
    }
    switch (vec) {
    case IVT_VHPT:
    case IVT_ITLB:
    case IVT_DTLB:
    case IVT_ALT_ITLB:
    case IVT_ALT_DTLB:
        if ((ifa >> 61) == 0) {
            uint64_t pa = ifa & ((1ull << 50) - 1) & ~((1ull << 24) - 1);
            bool wb = pa < ms->ram_size || (pa >= FW_BASE && pa < FW_END);
            uint64_t pte = 1 | ((wb ? 0ull : 4ull) << 2) | (1 << 5) |
                           (1 << 6) | (3 << 9) | pa;
            bool insn = vec == IVT_ITLB || vec == IVT_ALT_ITLB ||
                        (vec == IVT_VHPT && (isr & (1ull << 32)));

            ia64_tc_insert(env, insn, ifa, pte, 24 << 2);
            ia64_rfi(env, 0);
            return;
        }
        break;
    }
    error_report("ia64 efi: unhandled interruption 0x%04" PRIx64 " at 0x%"
                 PRIx64 " slot %d: isr 0x%" PRIx64 " ifa 0x%" PRIx64,
                 vec, env->cr[CR_IIP],
                 (int)((env->cr[CR_IPSR] >> 41) & 3), isr, ifa);
    cpu_dump_state(env_cpu(env), stderr, 0);
    qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_PANIC);
    efi.dead = true;
    env->ip = FW_STUB(FW_CALL_BOOT);
    env->ri = 0;
}

void efi_boot_call(CPUIA64State *env, unsigned n)
{
    bool locked = bql_locked();

    if (!locked) {
        bql_lock();
    }
    efi.env = env;
    if (n == FW_CALL_BOOT && efi.dead) {
        env->ip = FW_STUB(FW_CALL_BOOT);
        env->ri = 0;
    } else if (n == FW_CALL_BOOT) {
        efi_init(env);
        efi_boot(env);
    } else if (n == FW_CALL_IVT) {
        ivt_call(env);
    } else if (n == FW_CALL_IMAGE_RETURN) {
        if (efi.nactive) {
            image_finish(env, efi.active[efi.nactive - 1], env->gr[8]);
        }
    } else if (n >= FW_CALL_BS && n < FW_CALL_BS + BS_NUM) {
        bs_call(env, n - FW_CALL_BS);
    } else if (n == P_NOTIFY_RETURN) {
        notify_return(env);
    } else if (n >= FW_CALL_PROTO && n < P_END) {
        proto_call(env, n);
    } else {
        fw_ret(env, EFI_UNSUPPORTED, 0, 0, 0);
    }
    if (!locked) {
        bql_unlock();
    }
}
