#include <cdcacm.h>
#include <errno.h>
#include <reent.h>
#include <sys/types.h>
#include <unused.h>

ssize_t _read_r(struct _reent *UNUSED(r), int UNUSED(fd), void *UNUSED(buf),
                size_t UNUSED(len))
{
    return 0;
}

ssize_t _write_r(struct _reent *UNUSED(r), int UNUSED(fd), const void *buf, size_t len)
{
    cdcacm_write(buf, len);
    return (ssize_t)len;
}

off_t _lseek_r(struct _reent *r, int UNUSED(fd), off_t UNUSED(off), int UNUSED(whence))
{
    r->_errno = ESPIPE;
    return (off_t)-1;
}

int _close_r(struct _reent *UNUSED(r), int UNUSED(fd))
{
    return 0;
}

int _fstat_r(struct _reent *r, int UNUSED(fd), struct stat *UNUSED(st))
{
    r->_errno = ENOSYS;
    return -1;
}

int _isatty_r(struct _reent *UNUSED(r), int UNUSED(fd))
{
    return 1;
}
