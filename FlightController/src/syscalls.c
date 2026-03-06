/*
 * Minimal syscall stubs for bare-metal newlib.
 * These override the nosys stubs that emit linker warnings in newlib 4.x.
 */
#include <sys/stat.h>

int _close(int fd)                        { (void)fd; return -1; }
int _fstat(int fd, struct stat *st)       { (void)fd; st->st_mode = S_IFCHR; return 0; }
int _isatty(int fd)                       { (void)fd; return 1; }
int _lseek(int fd, int off, int whence)   { (void)fd; (void)off; (void)whence; return 0; }
int _read(int fd, char *buf, int len)     { (void)fd; (void)buf; (void)len; return 0; }
int _write(int fd, char *buf, int len)    { (void)fd; (void)buf; return len; }
