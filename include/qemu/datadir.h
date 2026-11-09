#ifndef QEMU_DATADIR_H
#define QEMU_DATADIR_H

typedef enum {
    QEMU_FILE_TYPE_BIOS,
    QEMU_FILE_TYPE_DTB,
    QEMU_FILE_TYPE_KEYMAP,
} QemuFileType;

/* A data file built into the binary, see qemu_register_embedded_files(). */
typedef struct QemuEmbeddedFile {
    const char *name;           /* as qemu_find_file() looks it up, "dtb/x" */
    const unsigned char *data;
    size_t size;
} QemuEmbeddedFile;

/**
 * qemu_find_file:
 * @type: QEMU_FILE_TYPE_BIOS (for BIOS, VGA BIOS)
 *        QEMU_FILE_TYPE_DTB (for device tree blobs)
 *        or QEMU_FILE_TYPE_KEYMAP (for keymaps).
 * @name: Relative or absolute file name
 *
 * If @name exists on disk as an absolute path, or a path relative
 * to the current directory, then returns @name unchanged.
 * Otherwise searches for @name file in the data directories, either
 * configured at build time (DATADIR) or registered with the -L command
 * line option.
 *
 * The caller must use g_free() to free the returned data when it is
 * no longer required.
 *
 * Returns: a path that can access @name, or NULL if no matching file exists.
 */
char *qemu_find_file(QemuFileType type, const char *name);
void qemu_add_default_firmwarepath(void);

/**
 * qemu_register_embedded_files:
 * @files: array terminated by an entry with a NULL name
 *
 * qemu_find_file() falls back to these when a file is in none of the data
 * directories, so a file on disk always wins.
 */
void qemu_register_embedded_files(const QemuEmbeddedFile *files);
void qemu_add_data_dir(char *path);
void qemu_list_data_dirs(void);

#endif
