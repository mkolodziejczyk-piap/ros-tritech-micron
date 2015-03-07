# -*- coding: utf-8 -*-

"""Tritech Micron Sonar ideal ellipse configuration."""

__author__ = "Malcolm Watt"
__version__ = "0.0.1"

#It will be important to modify this so that it uses ellipses.
class IdealConfig:

    """Contains the control state of the current task.

    Attributes:
        task: The current task being attempted.
        ellipse_set: A list of ellipse objects representing the current task.
    """

    def __init__(self, task):
        """Constructs IdealConfig object.

        Args:
            task: The current task underway.
        """
        self.task = task
        self.ellipse_set = self._ellipse_set(task)
        #_ellipse_set(arg) is the internal method returning a set of ellipses

    def _ellipse_set(self, task):
        """Returns a list of integers.
        Will eventually return a list of ellipse objects
            Args
                task: The current task underway.    
        """
        # TODO(Malcolm): Fetch a list of ideal objects from dictionary
        return [1,2,3,4,5]
    

