#include "software/multithreading/observer_subject_adapter.h"

template <typename ObserverType, typename SubjectType>
void ObserverSubjectAdapter<ObserverType, SubjectType>::receiveValue(ObserverType val)
{
    Subject<SubjectType>::sendValueToObservers(conversion_function(val));
}
