#pragma once
#include "headers.h"


struct apcitem
{
    Point p;
    Segment s;
};

struct annPointCloud
{
    vector<apcitem>  pts;
    void clear() { pts.clear(); }
};

struct PointCloudAdaptor
{

    const annPointCloud& obj; //!< A const ref to the data set origin

    /// The constructor that sets the data set source
    PointCloudAdaptor(const annPointCloud& obj_) : obj(obj_) { }

    /// CRTP helper method
    inline const annPointCloud& derived() const { return obj; }

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return derived().pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (dim == 0) return derived().pts[idx].p.x();
        else if (dim == 1) return derived().pts[idx].p.y();
        else return derived().pts[idx].p.z();
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }
};

typedef KDTreeSingleIndexAdaptor<
    L2_Simple_Adaptor<double, PointCloudAdaptor >,
    PointCloudAdaptor,
    3 /* dim */
> my_kd_tree_t;


struct Segment_Hash
{
    size_t operator()(const Segment& rhs) const
    {
        return (hash<double>()(rhs.source().x())+1) 
    		^ (hash<double>()(rhs.source().y()) + 2)
			^ (hash<double>()(rhs.source().z()) + 3)
    		^ (hash<double>()(rhs.target().x()) + 4)
    		^ (hash<double>()(rhs.target().y()) + 5)
            ^ (hash<double>()(rhs.target().z()) + 6);
    }
};
struct Segment_equal
{
    bool operator()(const Segment& lhs, const Segment& rhs) const
	{
        return lhs.source().x() == rhs.source().x() 
    		&& lhs.source().y() == rhs.source().y()
            && lhs.source().z() == rhs.source().z()
    		&& lhs.target().x() == rhs.target().x()
    		&& lhs.target().y() == rhs.target().y()
            && lhs.source().z() == rhs.source().z();
    }
};

struct Point_Hash
{
    size_t operator() (const Point& rhs) const
    {
        return hash<double>()(rhs.x())
            ^ hash<double>()(rhs.y())
            ^ hash<double>()(rhs.z());
    }
};
struct Point_equal
{
    bool operator()(const Point& lhs, const Point& rhs) const
    {
        return lhs.x() == rhs.x()
            && lhs.y() == rhs.y()
            && lhs.z() == rhs.z();
    }
};

inline Segment TwinEdge(Segment& edge)
{
    Segment re(edge.target(), edge.source());
    return re;
}

class assignment
{
public:
    vector<Point> assined_points;
	vector<Point> close_points;
    double to_edge_cost = 0;
    double to_vertex_cost = 0;
    double normal_cost = 0;
    double tangential_cost = 0;

    double total_cost = 0;

    void clear_num()
    {
        to_edge_cost = 0;
        to_vertex_cost = 0;
        normal_cost = 0;
        tangential_cost = 0;
        total_cost = 0;
        Cost_max = 0;
    }

    void clearAll()
    {
        assined_points.clear();
        to_edge_cost = 0;
        to_vertex_cost = 0;
        normal_cost = 0;
        tangential_cost = 0;
        total_cost = 0;
        Cost_max = 0;
    }

    Point maxpoint;
    double Cost_max=0;
};

class edge_data
{
public:
    //Segment seg_edge;

    assignment assign;

    double valid_value;

    edge_data(){};
    ~edge_data(){};
};

class Vertex_data
{
public:
    //Point point;
    list<Segment> adjacent_edges;

    assignment assign;

    Vertex_data(){};
    ~Vertex_data(){};
};

class mymesh
{
public:
	unordered_map<Segment, edge_data,Segment_Hash, Segment_equal> edges;
	unordered_map<Point, Vertex_data,Point_Hash,Point_equal> Vertexs;

    annPointCloud cloud;

    mymesh();
    ~mymesh();

    void BuildSampleKDtree();
    void BuildSampleKDtree(unordered_set<Segment, Segment_Hash, Segment_equal> local_edges);
    int MakeCollaps(const Point& s, const Point& t);
	void InsertPoint(Segment e, Point p);
	void MovePoint(Point s, Point t);
	void buildAdj();
    void ClearAssin();
    void clear();
};

