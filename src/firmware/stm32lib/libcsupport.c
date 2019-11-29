#include <errno.h>
#include <newlib.h>
#include <reent.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

#ifndef _WANT_IO_LONG_LONG
#error You must build newlib with --enable-newlib-io-long-long
#endif

extern char linker_heap_vma, linker_heap_vma_end;

void *_sbrk_r(struct _reent *re, intptr_t increment)
{
    static char *break_pointer = &linker_heap_vma;
    char *old_break            = break_pointer;
    char *new_break            = break_pointer + increment;
    if (new_break > &linker_heap_vma_end)
    {
        re->_errno = ENOMEM;
        return (void *)-1;
    }
    else
    {
        break_pointer = new_break;
        return old_break;
    }
}

void __aeabi_unwind_cpp_pr0(void)
{
    abort();
}
