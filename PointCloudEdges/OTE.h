#pragma once
#include "mymesh.h"

typedef struct maxCost
{
	Segment e;
	Point p;
	bool is_e;
}maxCost;

class OTE
{
public:
	vector<Point> points_input;
	Delaunay delaunay_input;
	Delaunay delaunay_temp;
	mymesh ms1;
	mymesh ms2;

	vector<Point> assin_points;					//待分配的顶点
	maxCost maxc;

	unordered_set<Segment, Segment_Hash,Segment_equal> to_be_Collaps;	//下次迭代中将要被计算合并的边
	unordered_map<Segment, double, Segment_Hash, Segment_equal> pri_queue;
	double less_cost;
	Segment less_seg;
	unordered_set<Segment, Segment_Hash, Segment_equal> two_ring_edge;
	unordered_set<Point, Point_Hash, Point_equal> two_ring_point;
	unordered_set<Segment, Segment_Hash, Segment_equal> sample_edge;

	double pri_cost;							//上一次的代价
	int iter_times;								//迭代次数

	int debugvalue = 0;

	bool isCollaps = false;

	OTE();
	~OTE();

	void InitAddPoint(vector<Point> input);		//初始化加点
	void InitCollapAfterAdd();
	void UpdatePriQueue();
	void CollectAssPoint(Segment e);
	void addPoint();							//加点，一次迭代
	void ReDelauna();
	void CopyDeToMs();
	void PickAndCollap();
	double CaculateAssinCost(int local = 0);					//计算当前分配方案总代价
	void GetMaxCost();							//获取最大的那个点
	double CaculateEachEdgeCost();				//计算每条边的代价
	void CaculateTangentialCost(Segment e, assignment& c);	//计算切向代价
	double PointProjectToSource(Segment e, Point p);	//计算点p在e上的投影到e原点的距离
	void CaculateVertexCost(Segment e, assignment& c);	//计算切向代价
	void AssinToVertex(Segment e, assignment& c);		//重新分配给顶点
	void GetValid1();


	void RelocateOnce();
	Point Relocatev(Point v);		//relocate顶点v
	double Getlamuda(Segment e, Point p);		//获取点P在e上投影的重心坐标lamuda
	void applyRelocate(Point& s, Point& t);		//将s移到t

	vector<Point> GetOneRingVertex(Point v, int flag = 0);	//获取ms2中顶点v周围一圈的顶点
	unordered_set<Point, Point_Hash, Point_equal> GetOneRingVertex(vector <Point> v, int flag = 0);	//获取ms2中顶点v周围一圈的顶点
	unordered_set<Segment, Segment_Hash, Segment_equal> GetOneRingEdge(vector<Point> s, int flag = 0);		//获取一圈的边
};