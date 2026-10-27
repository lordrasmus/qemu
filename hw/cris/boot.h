#ifndef HW_CRIS_BOOT_H
#define HW_CRIS_BOOT_H

struct cris_load_info
{
    const char *image_filename;
    const char *cmdline;
    const char *initrd_filename;
    int image_size;
    ram_addr_t ram_size;
    hwaddr cmdline_addr;

    hwaddr entry;
};

void cris_load_image(CRISCPU *cpu, struct cris_load_info *li);

#endif
