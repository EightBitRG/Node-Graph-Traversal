'''
Michael Mundell
CS2420
'''
import math
import numpy as np
from graphviz import Source

class DijkstraNode:
    '''Class used to hold nodes using Dijkstra's shortest path algorithm'''
    def __init__(self):
        self.label = ""
        self.prev = None
        self.weight = math.inf


class Graph:
    def __init__(self):
        self.labels = [] # used to store all vertexes added to graph
        self.matrix = np.zeros((0,0)) #stores the graph and all the edges connecting vertexes
    def _add_column(self):
        '''Expands the  number of columns on self.matrix'''
        new_col = np.zeros((self.matrix.shape[0],1))
        self.matrix = np.hstack((self.matrix, new_col))
    def _add_row(self):
        '''Expands the  number of rows on self.matrix'''
        new_row =np.zeros((1, self.matrix.shape[1]))
        self.matrix = np.vstack((self.matrix, new_row))
    def add_vertex(self, label):
        '''add a vertex with the specified label. Return the graph. label must be a
        string or raise ValueError'''
        if not isinstance(label, str):
            raise ValueError
        if label not in self.labels:
            self.labels.append(label)
            self._add_column()
            self._add_row()
        return self
    def _isthere(self,vertex):
        '''checks to see if a vertex is in the label is in the graph or not
        otherwise a ValueError is raised'''
        if not vertex in self.labels:
            raise ValueError
    def add_edge(self, src, dest, w):
        '''add an edge from vertex src to vertex dest with weight w. Return
        the graph. validate src, dest, and w: raise ValueError if not valid.'''
        if not ((src in self.labels and dest in self.labels) and (isinstance(w, int) or isinstance(w,float))):
            #raises if src or dest is not stored in graph or w is not a valid float or integer
            raise ValueError
        src_index = self.labels.index(src)
        dest_index = self.labels.index(dest)
        self.matrix[src_index][dest_index] = w
        return self
    def get_weight(self,src,dest):
        '''Return the weight on edge src-dest (math.inf if no path exists,
            raise ValueError if src or dest not added to graph).'''
        self._isthere(src)
        self._isthere(dest) #checks to see if src and dest vertexes exist in graph
        src_index = self.labels.index(src)
        dest_index = self.labels.index(dest)
        if self.matrix[src_index][dest_index] == 0:
            return math.inf
        return self.matrix[src_index][dest_index]
    def _get_neighbors(self,starting_vertex):
        '''Gets all vertexes nearby a chosen vertex'''
        neighbors = []
        vertex_index = self.labels.index(starting_vertex)
        for i in range (len(self.labels)):
            if self.matrix[vertex_index][i] != 0:
                neighbors.append(self.labels[i])
        return neighbors
    def _dbfs(self,starting_vertex,depth):
        '''Perform either a Depth or Breadth first search and return the result'''
        if depth == True:
            #doing a depth first search
            popnum = -1
        else:
            #doing a breadth first search
            popnum = 0
        self._isthere(starting_vertex)
        result = []
        visited = []
        collection = [starting_vertex]
        visited.append(starting_vertex)
        while collection:
            focus = collection.pop(popnum)
            result.append(focus)
            for neighbor in self._get_neighbors(focus):
                if neighbor not in visited:
                    collection.append(neighbor)
                    visited.append(neighbor)
        return result
    def dfs(self,starting_vertex):
        '''Return a generator for traversing the graph in depth-first order
        starting from the specified vertex. Raise a ValueError if the vertex does not exist.'''
        result = self._dbfs(starting_vertex,True)
        for item in result:
            yield item
    def bfs(self,starting_vertex):
        '''Return a generator for traversing the graph in breadth-first order
        starting from the specified vertex. Raise a ValueError if the vertex does not exist'''
        result = self._dbfs(starting_vertex,False)
        for item in result:
            yield item
    def dsp(self,src,dest):
        '''Return a tuple (path length , the list of vertices on the path from dest
        back to src). If no path exists, return the tuple (math.inf, empty list.)'''
        if src == dest:
            return (0, [src])
        self._isthere(src)
        self._isthere(dest)#checks to see if src and dest vertexes exist in graph
        priority_Queue = []
        visited = [] #set all nodes as unvisited
        src_index = self.labels.index(src)
        for label in self.labels:
            node = DijkstraNode()
            node.label = label
            if node.label == src:
                node.weight = 0
            priority_Queue.append(node)
        focus_node = priority_Queue[src_index]
        success= False
        while not success:
            for neighbor in self._get_neighbors(focus_node.label):
                current_weight = self.get_weight(focus_node.label,neighbor)
                neighbor_index = self.labels.index(neighbor)
                if current_weight+focus_node.weight < priority_Queue[neighbor_index].weight:
                    priority_Queue[neighbor_index].weight = current_weight+focus_node.weight
                    priority_Queue[neighbor_index].prev = focus_node
            #When we are done considering all of the unvisited neighbours of the
            #current node, mark the current node as visited
            visited.append(focus_node)
            lowest = [None,math.inf]
            for node in priority_Queue:
                if node not in visited:
                    if node.weight < lowest[1]:
                        lowest = [node,node.weight]
                        if lowest[0].label == dest:
                            focus_node = priority_Queue[self.labels.index(dest)]
                            distance = focus_node.weight
                            result = []
                            while focus_node.label != src:
                                result.append(focus_node.label)
                                focus_node = focus_node.prev
                            result.append(src)
                            return (distance, [ele for ele in reversed(result)])
            if lowest[0] is None:
                #if there is not path to the vertex, return math.inf and an empty array
                return (math.inf, [])
            focus_node = lowest[0]
    def dsp_all(self,src):
        '''Return a dictionary of the shortest weighted path between src and all
        other vertices using Dijkstra's Shortest Path algorithm. In the dictionary, the key is the the
        destination vertex label, the value is a list of vertices on the path from src to dest inclusive. '''
        self._isthere(src)
        dictionary = {}
        for item in self.labels:
            dictionary[item] = self.dsp(src,item)[1]
        return dictionary
    def display(self):
        mySource = Source(self.__str__())
        mySource.view()
    def __str__(self):
        '''Produce a string representation of the graph in GraphViz dot notation
        that can be used with print().'''
        result = "digraph G {\n"
        for i in range(len(self.labels)):
            src = self.labels[i]
            for j in range(len(self.labels)):
                dest = self.labels[j]
                weight = self.matrix[i][j]
                if weight != 0:
                    weight = str(format(weight,".1f"))
                    result += '   '+src+' -> '+dest+' [label="'+weight+'",weight="'+weight+'"];\n'
        result += '}\n'
        return result
def main():
    G = Graph()
    G.add_vertex("A")
    G.add_vertex("A")
    G.add_vertex("B")
    G.add_vertex("C")
    G.add_vertex("D")
    G.add_vertex("E")
    G.add_vertex("F")
    G.add_edge("A","B",2)
    G.add_edge("A","F",9)
    G.add_edge("B","F",6)
    G.add_edge("F","B",6)
    G.add_edge("B","C",8)
    G.add_edge("B","D",15)
    G.add_edge("C","D",1)
    G.add_edge("E","C",7)
    G.add_edge("E","D",3)
    G.add_edge("F","E",3)
    print(G)
    print("Starting BFS with Vertex A")
    print(''.join([x for x in G.bfs("A")]))
    print("Starting DFS with Vertex A")
    print(''.join([x for x in G.dfs("A")]))
    print("Starting dsp starting at A going to F")
    print(G.dsp("A","F"))
    print("Starting dsp on all at A going to all vertexes")
    dictionary = G.dsp_all("A")
    for item in dictionary:
        print(item+": "+str(dictionary[item]))
    #G.display()
if __name__ == '__main__':
    main()
