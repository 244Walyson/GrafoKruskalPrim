using System;
using System.Collections.Generic;
using System.Linq;

class Program
{
    static Random random = new Random();

    static void Main(string[] args)
    {
        int numVertices = 6; 
        int numArestas = 10;

        Graph graph = GenerateRandomGraph(numVertices, numArestas);

        Console.WriteLine("Grafo aleatório:");
        graph.PrintGraph();

        List<Edge> spanningTree = ArvoreGeradora(graph);
        Console.WriteLine("\nÁrvore Geradora:");
        PrintEdges(spanningTree);

        List<Edge> arvoreGeradoraMinimaPrim = ArvoreGeradoraMinimaPrim(graph);
        Console.WriteLine("\nÁrvore Geradora Mínima com Prim:");
        PrintEdges(arvoreGeradoraMinimaPrim);

        List<Edge> arvoreGeradoraMinimaKruskal = ArvoreGeradoraMinimaKruskal(graph);
        Console.WriteLine("\nÁrvore Geradora Mínima com Kruskal:");
        PrintEdges(arvoreGeradoraMinimaKruskal);

        Console.ReadLine();
    }

    class Edge
    {
        public int Source { get; set; }
        public int Destiny { get; set; }
        public int Weight { get; set; }
    }

    class Graph
    {
        public int Vertices { get; }
        public List<Edge> Edges { get; }

        public Graph(int vertices)
        {
            Vertices = vertices;
            Edges = new List<Edge>();
        }

        public void AddEdge(int Source, int Destiny, int Weight)
        {
            Edges.Add(new Edge { Source = Source, Destiny = Destiny, Weight = Weight });
        }

        public void PrintGraph()
        {
            foreach (var edge in Edges)
            {
                Console.WriteLine($"({edge.Source} --> {edge.Destiny}) : {edge.Weight}");
            }
        }
    }

    static Graph GenerateRandomGraph(int numVertices, int numArestas)
    {
        Graph graph = new Graph(numVertices);

        for (int i = 0; i < numArestas; i++)
        {
            int Source = random.Next(numVertices);
            int Destiny = random.Next(numVertices);
            int Weight = random.Next(1, 100);
            graph.AddEdge(Source, Destiny, Weight);
        }

        return graph;
    }

    static List<Edge> ArvoreGeradora(Graph graph)
    {
        List<Edge> mst = new List<Edge>();

        // Marque todos os vértices como não visitados
        bool[] visited = new bool[graph.Vertices];
        for (int i = 0; i < graph.Vertices; i++)
            visited[i] = false;

        // Comece a busca em profundidade a partir do vértice 0 (ou outro vértice inicial de sua escolha)
        DFS(graph, 0, visited, mst);

        return mst;
    }

    static void DFS(Graph graph, int v, bool[] visited, List<Edge> mst)
    {
        visited[v] = true;

        foreach (Edge edge in graph.Edges)
        {
            if (!visited[edge.Destiny] && (edge.Source == v || edge.Destiny == v))
            {
                mst.Add(edge); // Adicione a aresta à árvore geradora
                int nextVertex = (edge.Source == v) ? edge.Destiny : edge.Source;
                DFS(graph, nextVertex, visited, mst);
            }
        }
    }


    static List<Edge> ArvoreGeradoraMinimaPrim(Graph graph)
    {
        List<Edge> mst = new List<Edge>();

        int startVertex = 0;

        HashSet<int> includedVertices = new HashSet<int> { startVertex };

        while (includedVertices.Count < graph.Vertices)
        {
            Edge minEdge = null;
            foreach (var edge in graph.Edges)
            {
                if (includedVertices.Contains(edge.Source) && !includedVertices.Contains(edge.Destiny)
                    || includedVertices.Contains(edge.Destiny) && !includedVertices.Contains(edge.Source))
                {
                    if (minEdge == null || edge.Weight < minEdge.Weight)
                    {
                        minEdge = edge;
                    }
                }
            }

            if (minEdge != null)
            {
                mst.Add(minEdge);
                includedVertices.Add(minEdge.Source);
                includedVertices.Add(minEdge.Destiny);
            }
        }

        return mst;
    }

    static List<Edge> ArvoreGeradoraMinimaKruskal(Graph graph)
    {
        List<Edge> mst = new List<Edge>();

        List<Edge> sortedEdges = graph.Edges.OrderBy(edge => edge.Weight).ToList();

        Dictionary<int, int> disjointSet = new Dictionary<int, int>();
        foreach (var edge in graph.Edges)
        {
            disjointSet[edge.Source] = edge.Source;
            disjointSet[edge.Destiny] = edge.Destiny;
        }

        foreach (var edge in sortedEdges)
        {
            int SourceRoot = FindRoot(disjointSet, edge.Source);
            int destRoot = FindRoot(disjointSet, edge.Destiny);

            if (SourceRoot != destRoot)
            {
                mst.Add(edge);
                disjointSet[SourceRoot] = destRoot;
            }
        }

        return mst;
    }

    static int FindRoot(Dictionary<int, int> disjointSet, int vertex)
    {
        if (disjointSet[vertex] == vertex)
        {
            return vertex;
        }
        return FindRoot(disjointSet, disjointSet[vertex]);
    }

    static void PrintEdges(List<Edge> edges)
    {
        foreach (var edge in edges)
        {
            Console.WriteLine($"({edge.Source} --> {edge.Destiny}) : {edge.Weight}");
        }
    }
}
