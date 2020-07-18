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

	vector<Point> assin_points;					//������Ķ���
	maxCost maxc;

	unordered_set<Segment, Segment_Hash,Segment_equal> to_be_Collaps;	//�´ε����н�Ҫ������ϲ��ı�
	unordered_map<Segment, double, Segment_Hash, Segment_equal> pri_queue;
	double less_cost;
	Segment less_seg;
	unordered_set<Segment, Segment_Hash, Segment_equal> two_ring_edge;
	unordered_set<Point, Point_Hash, Point_equal> two_ring_point;
	unordered_set<Segment, Segment_Hash, Segment_equal> sample_edge;

	double pri_cost;							//��һ�εĴ���
	int iter_times;								//��������

	int debugvalue = 0;

	bool isCollaps = false;

	OTE();
	~OTE();

	void InitAddPoint(vector<Point> input);		//��ʼ���ӵ�
	void InitCollapAfterAdd();
	void UpdatePriQueue();
	void CollectAssPoint(Segment e);
	void addPoint();							//�ӵ㣬һ�ε���
	void ReDelauna();
	void CopyDeToMs();
	void PickAndCollap();
	double CaculateAssinCost(int local = 0);					//���㵱ǰ���䷽���ܴ���
	void GetMaxCost();							//��ȡ�����Ǹ���
	double CaculateEachEdgeCost();				//����ÿ���ߵĴ���
	void CaculateTangentialCost(Segment e, assignment& c);	//�����������
	double PointProjectToSource(Segment e, Point p);	//�����p��e�ϵ�ͶӰ��eԭ��ľ���
	void CaculateVertexCost(Segment e, assignment& c);	//�����������
	void AssinToVertex(Segment e, assignment& c);		//���·��������
	void GetValid1();


	void RelocateOnce();
	Point Relocatev(Point v);		//relocate����v
	double Getlamuda(Segment e, Point p);		//��ȡ��P��e��ͶӰ����������lamuda
	void applyRelocate(Point& s, Point& t);		//��s�Ƶ�t

	vector<Point> GetOneRingVertex(Point v, int flag = 0);	//��ȡms2�ж���v��ΧһȦ�Ķ���
	unordered_set<Point, Point_Hash, Point_equal> GetOneRingVertex(vector <Point> v, int flag = 0);	//��ȡms2�ж���v��ΧһȦ�Ķ���
	unordered_set<Segment, Segment_Hash, Segment_equal> GetOneRingEdge(vector<Point> s, int flag = 0);		//��ȡһȦ�ı�
};