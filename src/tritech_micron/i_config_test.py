# -*- coding: utf-8 -*-

"Tritech Micron Sonar ideal ellipse unit tests."""

__author__ = "Malcolm Watt"
__version__ = "0.0.1"

# TODO (Malcolm): Make this accessible as a script ("main")
def test_cstr(task):
    """Unit Test for the constructor of the IdealConfig object.
    
    Args: 
        task: The current task
    """
    import ideal_configuration
    a = ideal_configuration.IdealConfig(task)
    print "The current task is: " + a.task
