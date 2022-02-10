from random import randint

"""
wrapper class made so that node indices are not compared against one another
every comparison of less than, greather than, less than equal to, or greather
than equal to, return a random integer between 1 and 0. 
"""

"""
The wrapper uses a random number from package random.
https://docs.python.org/3/library/random.html

Notes:
1.  if you have to fix on a seed. Define it in the __init__ function.
"""

class Wrapper:
  def __init__(self, val):
    self.val = val

  def __lt__(self, other):
    return randint(0,1)

  def __le__(self, other):
    return randint(0,1)

  def __gt__(self, other):
    return randint(0,1)

  def __ge__(self, other):
    return randint(0,1)

  def __eq__(self, other):
    if self.val == other.val:
      return 1
    else:
      return 0

  def __ne__(self, other):
    if self.val != other.val:
      return 1
    else:
      return 0

