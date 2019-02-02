#ifndef UTIL_ASYNC_OPERATION_H
#define UTIL_ASYNC_OPERATION_H

#include <sigc++/signal.h>

#include <memory>

#include "util/noncopyable.h"

/**
 * An asynchronous operation which is currently in progress.
 *
 * @param <T> the return type of the operation.
 */
template <typename T>
class AsyncOperation : public NonCopyable
{
   public:
    /**
     * Invoked when the operation completes, whether successfully or not.
     *
     * The callback functions are passed the operation itself.
     */
    sigc::signal<void, AsyncOperation<T> &> signal_done;

    /**
     * Destroys the object.
     */
    virtual ~AsyncOperation();

    /**
     * Checks for the success of the operation and returns the return
     * value.
     *
     * If the operation failed, this function throws the relevant exception.
     *
     * @return the return value.
     * @throws an exception if operation has failed
     */
    virtual T result() const = 0;

    /**
     * Checks whether or not the operation failed.
     *
     * The default implementation calls result() and checks whether it throws an
     * exception.
     *
     * @return true if the operation failed, or false if not.
     */
    virtual bool succeeded() const
    {
        try
        {
            result();
            return true;
        }
        catch (...)
        {
            return false;
        }
    }
};

template <typename T>
inline AsyncOperation<T>::~AsyncOperation() = default;

#endif
