#ifndef UTIL_PROPERTY_H
#define UTIL_PROPERTY_H

#include <sigc++/signal.h>

#include <utility>

#include "util/noncopyable.h"

/**
 * A variable holding a value of some type, with the ability to notify
 * listeners when the value changes.
 *
 * @param <T> the type of value to hold.
 */
template <typename T>
class Property final : public NonCopyable
{
   public:
    /**
     * Constructs a new Property.
     *
     * @param value the value with which to initialize the Property.
     */
    explicit Property(const T &value) : value(value) {}

    /**
     * Move-constructs a new Property.
     *
     * @param value the value with which to initialize the Property.
     */
    explicit Property(T &&value) : value(std::move(value)) {}

    /**
     * Move-constructs a Property.
     *
     * Properties are not copyable, but they are movable.
     *
     * @param moveref the original Property to destroy while initializing
     * this Property.
     */
    Property(Property<T> &&moveref)
        : value(std::move(moveref.value)),
          signal_changing_(std::move(moveref.signal_changing_)),
          signal_changed_(std::move(moveref.signal_changed_))
    {
    }

    /**
     * Returns the signal fired when the value of the Property is about
     * to change.
     *
     * @return the signal.
     */
    sigc::signal<void> &signal_changing() const
    {
        return signal_changing_;
    }

    /**
     * Returns the signal fired when the value of the Property changes.
     *
     * @return the signal.
     */
    sigc::signal<void> &signal_changed() const
    {
        return signal_changed_;
    }

    /**
     * Move-assigns a Property.
     *
     * Properties are not copyable, but they are movable.
     *
     * @param moveref the original Property to destroy while initializing
     * this Property.
     */
    Property<T> &operator=(Property<T> &&moveref)
    {
        value            = std::move(moveref.value);
        signal_changing_ = std::move(moveref.signal_changing_);
        signal_changed_  = std::move(moveref.signal_changed_);
        return *this;
    }

    /**
     * Assigns a new value to the Property.
     *
     * @param val the new value to assign.
     */
    Property &operator=(const T &val)
    {
        if (value != val)
        {
            signal_changing().emit();
            value = val;
            signal_changed().emit();
        }
        return *this;
    }

    /**
     * Move-assigns a new value to the Property.
     *
     * @param val the new value to assign.
     */
    Property &operator=(T &&val)
    {
        if (value != val)
        {
            signal_changing().emit();
            value = std::move(val);
            signal_changed().emit();
        }
        return *this;
    }

    /**
     * Returns the value of the Property.
     *
     * @return the value.
     */
    const T &get() const
    {
        return value;
    }

    /**
     * Returns the value of the Property.
     *
     * @return the value.
     */
    operator T() const
    {
        return value;
    }

    /**
     * Returns the value of the Property.
     *
     * @return the value.
     */
    const T &operator->() const
    {
        return value;
    }

   private:
    T value;
    mutable sigc::signal<void> signal_changing_;
    mutable sigc::signal<void> signal_changed_;
};

#endif
