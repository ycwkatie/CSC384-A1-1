#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the Sokoban warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

#import os for time functions
from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS, sokoban_goal_state #for Sokoban specific classes and problems

#SOKOBAN HEURISTICS
def heur_displaced(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''       
  count = 0
  for box in state.boxes:
    if box not in state.storage:
      count += 1
  return count

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible sokoban heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''      
    #We want an admissible heuristic, which is an optimistic heuristic. 
    #It must always underestimate the cost to get from the current state to the goal.
    #The sum Manhattan distance of the boxes to their closest storage spaces is such a heuristic.  
    #When calculating distances, assume there are no obstacles on the grid and that several boxes can fit in one storage bin.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.
    result = 0
    for box in state.boxes:
      tmp_dist = []
      for store in state.storage:
        tmp_dist.append(abs(box[0] - store[0]) + abs(box[1]-store[1]))
      result += min(tmp_dist)
    return result

def heur_alternate(state):
#IMPLEMENT
    '''a better sokoban heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''        
    #heur_min_moves has flaws.   
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.
    
    #for box in state.boxes:
      ##box in bottom left corner
      #if (box[0]==0) and (box[1]==0):
        #return 99999
      ##box in top right corner
      #elif (box[0] == state.width-1) and (box[1]==state.height-1):
        #return 99999
      ##box in bottom right corner
      #elif (box[0] == state.width-1) and (box[1] == 0):
        #return 99999
      ##box in top left corner
      #elif (box[0]==0) and (box[1] == state.height-1):
        #return 99999
        
    
    result = 0
    m = Munkres()
    matrix = []
    for box in state.boxes:
        tmp_dist = []
        for store in state.storage:
          tmp_dist.append(abs(box[0] - store[0]) + abs(box[1]-store[1]))
        matrix.append(tmp_dist)
        
    indexes = m.compute(matrix)
    for row, column in indexes:
      x = matrix[row][column]
      result += x
   
    return result 

def fval_function(sNode, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state. 

    @param SokobanState state: A sokoban state
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return (1 - weight) * sNode.gval + weight * sNode.hval 

def weighted_astar(initial_state, timebound = 10):
#IMPLEMENT
    '''Provides an implementation of weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    se = SearchEngine('custom', 'full')
    found_solution = False
    for x in range(10,0,-1):
      weight = x*0.1
      final = se.search(initState=initial_state, heur_fn=heur_alternate, timebound=timebound, fval_function = fval_function, weight = weight, goal_fn=sokoban_goal_state)
      if final:
        found_solution = True
        solution = final
      if final==False and found_solution:
        return solution
    if found_solution:
      return solution
    else:
      return False 

#Munkres class to perform the Hungarian Algorithm
class Munkres:
      """
      Calculate the Munkres solution to the classical assignment problem.
      See the module documentation for usage.
      """
  
      def __init__(self):
          """Create a new instance"""
          self.C = None
          self.row_covered = []
          self.col_covered = []
          self.n = 0
          self.Z0_r = 0
          self.Z0_c = 0
          self.marked = None
          self.path = None
  
      def make_cost_matrix(profit_matrix, inversion_function):
          """
          **DEPRECATED**
  
          Please use the module function ``make_cost_matrix()``.
          """
          import munkres
          return munkres.make_cost_matrix(profit_matrix, inversion_function)
  
      make_cost_matrix = staticmethod(make_cost_matrix)
  
      def pad_matrix(self, matrix, pad_value=0):
          """
          Pad a possibly non-square matrix to make it square.
  
          :Parameters:
              matrix : list of lists
                  matrix to pad
  
              pad_value : int
                  value to use to pad the matrix
  
          :rtype: list of lists
          :return: a new, possibly padded, matrix
          """
          max_columns = 0
          total_rows = len(matrix)
  
          for row in matrix:
              max_columns = max(max_columns, len(row))
  
          total_rows = max(max_columns, total_rows)
  
          new_matrix = []
          for row in matrix:
              row_len = len(row)
              new_row = row[:]
              if total_rows > row_len:
                  # Row too short. Pad it.
                  new_row += [0] * (total_rows - row_len)
              new_matrix += [new_row]
  
          while len(new_matrix) < total_rows:
              new_matrix += [[0] * total_rows]
  
          return new_matrix
  
      def compute(self, cost_matrix):
          """
          Compute the indexes for the lowest-cost pairings between rows and
          columns in the database. Returns a list of (row, column) tuples
          that can be used to traverse the matrix.
  
          :Parameters:
              cost_matrix : list of lists
                  The cost matrix. If this cost matrix is not square, it
                  will be padded with zeros, via a call to ``pad_matrix()``.
                  (This method does *not* modify the caller's matrix. It
                  operates on a copy of the matrix.)
  
                  **WARNING**: This code handles square and rectangular
                  matrices. It does *not* handle irregular matrices.
  
          :rtype: list
          :return: A list of ``(row, column)`` tuples that describe the lowest
                   cost path through the matrix
  
          """
          self.C = self.pad_matrix(cost_matrix)
          self.n = len(self.C)
          self.original_length = len(cost_matrix)
          self.original_width = len(cost_matrix[0])
          self.row_covered = [False for i in range(self.n)]
          self.col_covered = [False for i in range(self.n)]
          self.Z0_r = 0
          self.Z0_c = 0
          self.path = self.__make_matrix(self.n * 2, 0)
          self.marked = self.__make_matrix(self.n, 0)
  
          done = False
          step = 1
  
          steps = { 1 : self.__step1,
                    2 : self.__step2,
                    3 : self.__step3,
                    4 : self.__step4,
                    5 : self.__step5,
                    6 : self.__step6 }
  
          while not done:
              try:
                  func = steps[step]
                  step = func()
              except KeyError:
                  done = True
  
          # Look for the starred columns
          results = []
          for i in range(self.original_length):
              for j in range(self.original_width):
                  if self.marked[i][j] == 1:
                      results += [(i, j)]
  
          return results
  
      def __copy_matrix(self, matrix):
          """Return an exact copy of the supplied matrix"""
          return copy.deepcopy(matrix)
  
      def __make_matrix(self, n, val):
          """Create an *n*x*n* matrix, populating it with the specific value."""
          matrix = []
          for i in range(n):
              matrix += [[val for j in range(n)]]
          return matrix
  
      def __step1(self):
          """
          For each row of the matrix, find the smallest element and
          subtract it from every element in its row. Go to Step 2.
          """
          C = self.C
          n = self.n
          for i in range(n):
              minval = min(self.C[i])
              # Find the minimum value for this row and subtract that minimum
              # from every element in the row.
              for j in range(n):
                  self.C[i][j] -= minval
  
          return 2
  
      def __step2(self):
          """
          Find a zero (Z) in the resulting matrix. If there is no starred
          zero in its row or column, star Z. Repeat for each element in the
          matrix. Go to Step 3.
          """
          n = self.n
          for i in range(n):
              for j in range(n):
                  if (self.C[i][j] == 0) and \
                     (not self.col_covered[j]) and \
                     (not self.row_covered[i]):
                      self.marked[i][j] = 1
                      self.col_covered[j] = True
                      self.row_covered[i] = True
  
          self.__clear_covers()
          return 3
  
      def __step3(self):
          """
          Cover each column containing a starred zero. If K columns are
          covered, the starred zeros describe a complete set of unique
          assignments. In this case, Go to DONE, otherwise, Go to Step 4.
          """
          n = self.n
          count = 0
          for i in range(n):
              for j in range(n):
                  if self.marked[i][j] == 1:
                      self.col_covered[j] = True
                      count += 1
  
          if count >= n:
              step = 7 # done
          else:
              step = 4
  
          return step
  
      def __step4(self):
          """
          Find a noncovered zero and prime it. If there is no starred zero
          in the row containing this primed zero, Go to Step 5. Otherwise,
          cover this row and uncover the column containing the starred
          zero. Continue in this manner until there are no uncovered zeros
          left. Save the smallest uncovered value and Go to Step 6.
          """
          step = 0
          done = False
          row = -1
          col = -1
          star_col = -1
          while not done:
              (row, col) = self.__find_a_zero()
              if row < 0:
                  done = True
                  step = 6
              else:
                  self.marked[row][col] = 2
                  star_col = self.__find_star_in_row(row)
                  if star_col >= 0:
                      col = star_col
                      self.row_covered[row] = True
                      self.col_covered[col] = False
                  else:
                      done = True
                      self.Z0_r = row
                      self.Z0_c = col
                      step = 5
  
          return step
  
      def __step5(self):
          """
          Construct a series of alternating primed and starred zeros as
          follows. Let Z0 represent the uncovered primed zero found in Step 4.
          Let Z1 denote the starred zero in the column of Z0 (if any).
          Let Z2 denote the primed zero in the row of Z1 (there will always
          be one). Continue until the series terminates at a primed zero
          that has no starred zero in its column. Unstar each starred zero
          of the series, star each primed zero of the series, erase all
          primes and uncover every line in the matrix. Return to Step 3
          """
          count = 0
          path = self.path
          path[count][0] = self.Z0_r
          path[count][1] = self.Z0_c
          done = False
          while not done:
              row = self.__find_star_in_col(path[count][1])
              if row >= 0:
                  count += 1
                  path[count][0] = row
                  path[count][1] = path[count-1][1]
              else:
                  done = True
  
              if not done:
                  col = self.__find_prime_in_row(path[count][0])
                  count += 1
                  path[count][0] = path[count-1][0]
                  path[count][1] = col
  
          self.__convert_path(path, count)
          self.__clear_covers()
          self.__erase_primes()
          return 3
  
      def __step6(self):
          """
          Add the value found in Step 4 to every element of each covered
          row, and subtract it from every element of each uncovered column.
          Return to Step 4 without altering any stars, primes, or covered
          lines.
          """
          minval = self.__find_smallest()
          for i in range(self.n):
              for j in range(self.n):
                  if self.row_covered[i]:
                      self.C[i][j] += minval
                  if not self.col_covered[j]:
                      self.C[i][j] -= minval
          return 4
  
      def __find_smallest(self):
          """Find the smallest uncovered value in the matrix."""
          minval = 99999
          for i in range(self.n):
              for j in range(self.n):
                  if (not self.row_covered[i]) and (not self.col_covered[j]):
                      if minval > self.C[i][j]:
                          minval = self.C[i][j]
          return minval
  
      def __find_a_zero(self):
          """Find the first uncovered element with value 0"""
          row = -1
          col = -1
          i = 0
          n = self.n
          done = False
  
          while not done:
              j = 0
              while True:
                  if (self.C[i][j] == 0) and \
                     (not self.row_covered[i]) and \
                     (not self.col_covered[j]):
                      row = i
                      col = j
                      done = True
                  j += 1
                  if j >= n:
                      break
              i += 1
              if i >= n:
                  done = True
  
          return (row, col)
  
      def __find_star_in_row(self, row):
          """
          Find the first starred element in the specified row. Returns
          the column index, or -1 if no starred element was found.
          """
          col = -1
          for j in range(self.n):
              if self.marked[row][j] == 1:
                  col = j
                  break
  
          return col
  
      def __find_star_in_col(self, col):
          """
          Find the first starred element in the specified row. Returns
          the row index, or -1 if no starred element was found.
          """
          row = -1
          for i in range(self.n):
              if self.marked[i][col] == 1:
                  row = i
                  break
  
          return row
  
      def __find_prime_in_row(self, row):
          """
          Find the first prime element in the specified row. Returns
          the column index, or -1 if no starred element was found.
          """
          col = -1
          for j in range(self.n):
              if self.marked[row][j] == 2:
                  col = j
                  break
  
          return col
  
      def __convert_path(self, path, count):
          for i in range(count+1):
              if self.marked[path[i][0]][path[i][1]] == 1:
                  self.marked[path[i][0]][path[i][1]] = 0
              else:
                  self.marked[path[i][0]][path[i][1]] = 1
  
      def __clear_covers(self):
          """Clear all covered matrix cells"""
          for i in range(self.n):
              self.row_covered[i] = False
              self.col_covered[i] = False
  
      def __erase_primes(self):
          """Erase all prime markings"""
          for i in range(self.n):
              for j in range(self.n):
                  if self.marked[i][j] == 2:
                      self.marked[i][j] = 0
  
  # ---------------------------------------------------------------------------
  # Functions
  # ---------------------------------------------------------------------------
  
def make_cost_matrix(profit_matrix, inversion_function):
      """
      Create a cost matrix from a profit matrix by calling
      'inversion_function' to invert each value. The inversion
      function must take one numeric argument (of any type) and return
      another numeric argument which is presumed to be the cost inverse
      of the original profit.
  
      This is a static method. Call it like this:
  
      .. python::
  
          cost_matrix = Munkres.make_cost_matrix(matrix, inversion_func)
  
      For example:
  
      .. python::
  
          cost_matrix = Munkres.make_cost_matrix(matrix, lambda x : sys.maxint - x)
  
      :Parameters:
          profit_matrix : list of lists
              The matrix to convert from a profit to a cost matrix
  
          inversion_function : function
              The function to use to invert each entry in the profit matrix
  
      :rtype: list of lists
      :return: The converted matrix
      """
      cost_matrix = []
      for row in profit_matrix:
          cost_matrix.append([inversion_function(value) for value in row])
      return cost_matrix
  

if __name__ == "__main__":
  #TEST CODE
  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 2; #2 second time limit for each problem
  print("*************************************")  
  print("Running A-star")     

  for i in range(0,40): #note that there are 40 problems in the set that has been provided.  We just run through 10 here for illustration.

    print("*************************************")  
    print("PROBLEM {}".format(i))
    
    s0 = PROBLEMS[i] #Problems will get harder as i gets bigger

    se = SearchEngine('astar', 'full')
    final = se.search(s0, sokoban_goal_state, heur_displaced, timebound)

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)    
    counter += 1

  if counter > 0:  
    percent = (solved/counter)*100

  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 

  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 8; #8 second time limit 
  print("Running Anytime Weighted A-star")   

  for i in range(0,40):
    print("*************************************")  
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i] #Problems get harder as i gets bigger
    final = weighted_astar(s0, timebound)

    if final:
      final.print_path()   
      solved += 1 
    else:
      unsolved.append(i)
    counter += 1      

  if counter > 0:  
    percent = (solved/counter)*100   
      
  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 


