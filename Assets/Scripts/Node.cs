using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

/// <summary>
/// The Node.
/// </summary>
public class Node 
{   
    private Vector3 position;
    private string name;
    /// <summary>
    /// The connections (neighbors).
    /// </summary>
    [SerializeField]
    protected List<Node> m_Connections = new List<Node>();
    public const float MIN_NODE_DIST = 0.01f;
    /// <summary>
    /// Gets the connections (neighbors).
    /// </summary>
    /// <value>The connections.</value>
    public virtual List<Node> connections
    {
        get
        {
            return m_Connections;
        }
    }

    public Node this[int index]
    {
        get
        {
            return m_Connections[index];
        }
    }
    public float distance(Node neighbour)
    {
        return Vector3.Distance(this.position,neighbour.position);
    }
    public void AddConnection(Node n)
    {
        m_Connections.Add(n);
    }
    public void RemoveConnection(Node n)
    {
        m_Connections.Remove(n);
    }

    public Node(string name,Vector3 pos)
    {
        this.name = name;
        this.position = pos;
    }

    public string Name
    {
        get
        {
            return name;
        }
    }
    public Vector3 Position
    {
        get
        {
            return position;
        }
    }

    public bool isNeighbour(Node n)
    {
        foreach(var conn in m_Connections)
        {
            if(n.distance(conn) <= MIN_NODE_DIST)
            {
                return true;
            }
        }
        return false;
    }
}
