#pragma once
#include <functional>
#include "software/multithreading/observer.h"
#include "software/multithreading/subject.h"

template <typename ObserverType, typename SubjectType>
class ObserverSubjectAdapter : public Observer<ObserverType>, public Subject<SubjectType> {
   public:
    using ConversionFunctionType = SubjectType(const ObserverType&);
    explicit ObserverSubjectAdapter(std::function<ConversionFunctionType> conversion_function_)
    : conversion_function(std::move(conversion_function_)) {}
    virtual void receiveValue(ObserverType val) final;
   private:
    std::function<ConversionFunctionType> conversion_function;
};

#include "software/multithreading/observer_subject_adapter.tpp"