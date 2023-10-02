from typing import TypeVar, List, Iterable, Union, Callable

T = TypeVar("T")


class ChangeAction:
    ADD = 1
    REMOVE = 2


class Change:
    """Represents a change in a list"""

    def __init__(self, changed_list: List, changed_slice: slice, action: ChangeAction):
        """Initialize the Change

        :param changed_list: The list that changed
        :param changed_slice: The slice of the observable_list that changed 
        :param type: The action that caused the change
        
        """
        self._changed_list = changed_list
        self._changed_slice = changed_slice
        self._action = action

    @property
    def elements(self) -> List:
        """The elements in the list affected by the change

        :returns: A list of elements affected by the change

        """
        return self._changed_list[self._changed_slice]

    @property
    def action(self) -> ChangeAction:
        """The action that caused the change
        
        :returns: The action that caused the change
        
        """
        return self._action


class ObservableList(list):
    """List that notifies observers when elements are added or removed"""

    def __init__(
        self, observer: Callable[[Change], None] = None, iterable: Iterable[T] = ()
    ):
        """Initialize the ObservableList
        
        :param observer: Observer method to register
        :param iterable: iterable from which elements are to be copied to
                         the initialized ObservableList
        
        """
        self.observers = []
        self.extend(iterable)
        self.register_observer(observer)

    def register_observer(self, observer: Callable[[Change], None]):
        """Register an observer method that will be called when the
        ObservableList provides a change notification

        :param observer: Observer method that takes a Change as its first argument
        
        """
        if observer:
            self.observers.append(observer)

    def append(self, element: T):
        """See list.append"""
        super().append(element)
        self.__notify_elements_added(self.__slice_at_index(len(self) - 1))

    def insert(self, index: int, element: T):
        """See list.insert"""
        super().insert(index, element)
        length = len(self)
        if index >= length:
            index = length - 1
        elif index < 0:
            index += length - 1
            if index < 0:
                index = 0
        self.__notify_elements_added(self.__slice_at_index(index))

    def extend(self, other: Iterable[T]):
        """See list.extend"""
        if other:
            index = len(self)
            super().extend(other)
            self.__notify_elements_added(self.__slice_at_index(index, len(other)))

    def pop(self, index: int = -1):
        """See list.pop"""
        self.__notify_elements_removed(self.__slice_at_index(index))
        return super().pop(index)

    def clear(self):
        """See list.clear"""
        self.__notify_elements_removed(slice(0, len(self)))
        return super().clear()

    def __iadd__(self, other: Iterable[T]):
        """See list.__iadd__"""
        self.extend(other)
        return self

    def __imul__(self, multiplier):
        raise NotImplementedError()

    def __delitem__(self, index_or_slice: Union[int, slice]):
        """See list.__delitem__"""
        self.__notify_elements_removed_index_or_slice(index_or_slice)
        super().__delitem__(index_or_slice)

    def __setitem__(self, index_or_slice: Union[int, slice], value: T):
        """See list.__setitem__"""
        notify_added = self.__notify_elements_removed_index_or_slice(index_or_slice)
        super().__setitem__(index_or_slice, value)
        notify_added()

    def __notify_elements_added(self, added_slice: slice):
        """Notify observers about elements added to the ObservableList

        :param added_slice: the slice of the ObservableList with the added elements

        """
        change = Change(self, added_slice, ChangeAction.ADD)
        for observer in self.observers:
            observer(change)

    def __notify_elements_removed(self, removed_slice: slice):
        """Notify observers about elements removed from the ObservableList

        :param added_slice: the slice of the ObservableList with the elements to be removed

        """
        change = Change(self, removed_slice, ChangeAction.REMOVE)
        for observer in self.observers:
            observer(change)

    def __notify_elements_removed_index_or_slice(
        self, index_or_slice: Union[int, slice]
    ) -> Callable:
        """Notify observers about elements removed at an index or slice

        :return: a function that notifies about an add at the same place

        """
        if isinstance(index_or_slice, int):
            length = len(self)
            if -length <= index_or_slice < length:
                slice_ = self.__slice_at_index(index_or_slice)
                self.__notify_elements_removed(slice_)
                return lambda: self.__notify_elements_added(slice_)
        elif isinstance(index_or_slice, slice):
            slice_ = slice(*index_or_slice.indices(len(self)))
            self.__notify_elements_removed(slice_)
            return lambda: self.__notify_elements_added(index_or_slice)

    def __slice_at_index(self, index: int, length: int = 1) -> slice:
        """Create a slice starting at an index and with a given length
        
        :param index: the index to start the slice at

        """
        length_ = len(self)
        if -length <= index < 0:
            index += length_
        return slice(index, index + length)
