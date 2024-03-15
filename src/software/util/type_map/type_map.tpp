template <class TValue>
TypeMap<TValue>::TypeMap() : map_()
{
}

template <class TValue>
int TypeMap<TValue>::type_id_counter_(0);

template <class TValue>
TypeMap<TValue>::Iterator TypeMap<TValue>::begin()
{
    return map_.begin();
}

template <class TValue>
TypeMap<TValue>::Iterator TypeMap<TValue>::end()
{
    return map_.end();
}

template <class TValue>
TypeMap<TValue>::ConstIterator TypeMap<TValue>::begin() const
{
    return map_.begin();
}

template <class TValue>
TypeMap<TValue>::ConstIterator TypeMap<TValue>::end() const
{
    return map_.end();
}

template <class TValue>
template <class TKey>
TypeMap<TValue>::Iterator TypeMap<TValue>::find()
{
    return map_.find(getTypeId<TKey>());
}

template <class TValue>
template <class TKey>
TypeMap<TValue>::ConstIterator TypeMap<TValue>::find() const
{
    return map_.find(getTypeId<TKey>());
}

template <class TValue>
template <class TKey>
bool TypeMap<TValue>::contains() const
{
    return map_.contains(getTypeId<TKey>());
}

template <class TValue>
template <class TKey>
void TypeMap<TValue>::put(TValue&& value)
{
    map_[getTypeId<TKey>()] = std::forward<TValue>(value);
}

template <class TValue>
template <class TKey>
TValue& TypeMap<TValue>::getOrDefault()
{
    return map_[getTypeId<TKey>()];
}
