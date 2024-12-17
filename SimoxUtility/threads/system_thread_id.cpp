#include "system_thread_id.h"

#include <unistd.h>

#include <syscall.h>

namespace simox
{
    long
    system_thread_id()
    {
        return syscall(SYS_gettid);
    }
} // namespace simox
