#ifndef UTIL_PROPERTY_H
#define UTIL_PROPERTY_H

#include <sigc++/signal.h>

#include <utility>

#include "util/noncopyable.h"

/**
 * \brief A variable holding a value of some type, with the ability to notify
 * listeners when the value changes.
 *
 * \tparam T the type of value to hold.
 */
template <typename T>
class Property final : public NonCopyable
{
   public:
    /**
     * \brief Constructs a new Property.
     *
     * \param[in] value the value with which to initialize the Property.
     */
    explicit Property(const T &value) : value(value) {}

    /**
     * \brief Move-constructs a new Property.
     *
     * \param[in] value the value with which to initialize the Property.
     */
    explicit Property(T &&value) : value(std::move(value)) {}

    /**
     * \brief Move-constructs a Property.
     *
     * Properties are not copyable, but they are movable.
     *
     * \param[in] moveref the original Property to destroy while initializing
     * this Property.
     */
    Property(Property<T> &&moveref)
        : value(std::move(moveref.value)),
          signal_changing_(std::move(moveref.signal_changing_)),
          signal_changed_(std::move(moveref.signal_changed_))
    {
    }

    /**
     * \brief Returns the signal fired when the value of the Property is about
     * to change.
     *
     * \return the signal.
     */
    sigc::signal<void> &signal_changing() const
    {
        return signal_changing_;
    }

    /**
     * \brief Returns the signal fired when the value of the Property changes.
     *
     * \return the signal.
     */
    sigc::signal<void> &signal_changed() const
    {
        return signal_changed_;
    }

    /**
     * \brief Move-assigns a Property.
     *
     * Properties are not copyable, but they are movable.
     *
     * \param[in] moveref the original Property to destroy while initializing
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
     * \brief Assigns a new value to the Property.
     *
     * \param[in] val the new value to assign.
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
     * \brief Move-assigns a new value to the Property.
     *
     * \param[in] val the new value to assign.
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
     * \brief Returns the value of the Property.
     *
     * \return the value.
     */
    const T &get() const
    {
        return value;
    }

    /**
     * \brief Returns the value of the Property.
     *
     * \return the value.
     */
    operator T() const
    {
        return value;
    }

    /**
     * \brief Returns the value of the Property.
     *
     * \return the value.
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
