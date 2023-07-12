from typing import Callable

class GraphicsList():
    """Grouped collection of GLGraphicsItems with support for added/removed item tracking"""

    def __init__(self):
        """Initialize the GraphicsList"""
        # Associates a grouping of graphics with a key
        self.graphics = {}

        # Functions for instantiating a GLGraphic for a given group key
        self.graphics_factories = {}

        self.num_fetched = {}
        self.num_added = {}

    def registerGraphicsGroup(self, key: str, graphics_factory: Callable):
        """Register a new group of GLGraphicsItems in the collection.

        :param key: The key used to identify the group
        :param graphics_factory: The function to call to instantiate a GLGraphicsItem for this group
        
        """
        self.graphics[key] = []
        self.graphics_factories[key] = graphics_factory
        self.num_fetched[key] = 0
        self.num_added[key] = 0

    def getGraphics(self, key: str, num_graphics: int):
        """Get a specified number of GLGraphicsItems from the specified group.
        The collection will automatically add more GLGraphicsItems to the group and return
        them if necessary.

        :param key: The key of the group
        :param num_graphics: The number of graphics to return from the group
        :returns: The fetched GLGraphicsItems from the group
        
        """
        graphics_list = self.graphics[key]

        num_fetched = self.num_fetched[key]
        num_added = self.num_added[key]

        num_graphics_to_add = max(num_graphics - (len(graphics_list) - num_fetched), 0)
        if num_graphics_to_add > 0:
            graphics_factory = self.graphics_factories[key]
            for _ in range(num_graphics_to_add):
                graphics_list.append(graphics_factory())
            self.num_added[key] += num_graphics_to_add

        fetched_graphics = graphics_list[num_fetched:num_fetched + num_graphics]
        self.num_fetched[key] += num_graphics

        return fetched_graphics

    def getChanges(self):
        """Return all graphics added or removed from the list since the last
        time getChanges was called.
        
        Graphics that haven't been fetched with getGraphics since the last
        time getChanges was called will be purged from the collection.

        Thus, calling getChanges without calling getGraphics beforehand 
        effectively clears all graphics from the collection.

        :returns: tuple (added_graphics, removed_graphics)
            - added_graphics - List of the added GLGraphicsItems
            - removed_graphics - List of the removed GLGraphicsItems

        """
        added_graphics = []
        removed_graphics = []

        for key, graphics_list in self.graphics.items():

            num_fetched = self.num_fetched[key]
            num_added = self.num_added[key]

            if num_added > 0:
                added_graphics += graphics_list[-num_added:]
            elif num_fetched < len(graphics_list):
                num_graphics_to_remove = len(graphics_list) - num_fetched
                for _ in range(num_graphics_to_remove):
                    removed_graphics.append(graphics_list.pop())

            self.num_fetched[key] = 0
            self.num_added[key] = 0

        return added_graphics, removed_graphics