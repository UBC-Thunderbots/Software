#pragma once
#include <functional>

#include "software/multithreading/observer.hpp"
#include "software/multithreading/subject.hpp"

/**
 * ObserverSubjectAdapter is a class made to "adapt" a type emitted by a Subject to
 * a different type consumed by an Observer with a conversion function passed in the
 * constructor.
 *
 * Based on the Adapter design pattern described here:
 * https://refactoring.guru/design-patterns/adapter
 *
 * @tparam ObserverType The type that this class observes, i.e. converted-from type
 * @tparam SubjectType The type that this class is a subject of, i.e. converted-to type
 */
template <typename ObserverType, typename SubjectType>
class ObserverSubjectAdapter : public Observer<ObserverType>, public Subject<SubjectType>
{
   public:
    // the type of the conversion function
    using ConversionFunctionType = SubjectType(const ObserverType&);

    /**
     * Constructs an ObserverSubjectAdapter with the given conversion function that
     * consumes a const-ref of the observed type and returns the subject type.
     * @param conversion_function_ A function that converts ObserverType to SubjectType.
     */
    explicit ObserverSubjectAdapter(
        std::function<ConversionFunctionType> conversion_function_)
        : conversion_function(std::move(conversion_function_))
    {
    }

    /**
     * Receives a value, calls the conversion function on it, and sends it to the
     * Observers of this object.
     * @param val a value
     */
    virtual void receiveValue(ObserverType val) final;

   private:
    std::function<ConversionFunctionType> conversion_function;
};

template <typename ObserverType, typename SubjectType>
void ObserverSubjectAdapter<ObserverType, SubjectType>::receiveValue(ObserverType val)
{
    Subject<SubjectType>::sendValueToObservers(conversion_function(val));
}
