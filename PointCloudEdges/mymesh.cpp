#include "mymesh.h"

mymesh::mymesh()
{
}

mymesh::~mymesh()
{
}

void mymesh::ClearAssin()
{
	for (auto &vit:Vertexs)
	{
		vit.second.assign.clearAll();
	}
	for (auto &eit : edges)
	{
		eit.second.assign.clearAll();
	}
}

void mymesh::clear()
{
	edges.clear();
	Vertexs.clear();
	cloud.clear();
}

void mymesh::BuildSampleKDtree()
{
	cloud.clear();
	int n = 9;
	for (auto eit:edges)
	{
		Segment e = eit.first;
		int a = 0;
		int b = n;
		for (int i = 0; i < n - 1; ++i)
		{
			a++;
			b--;
			double xx = (b * e.source().x() + a * e.target().x()) / (double)n;
			double yy = (b * e.source().y() + a * e.target().y()) / (double)n;
			double zz = (b * e.source().z() + a * e.target().z()) / (double)n;
			Point pp(xx, yy, zz);
			apcitem titem;
			titem.p = pp;
			titem.s = e;
			cloud.pts.push_back(titem);
		}
	}

}

void mymesh::BuildSampleKDtree(unordered_set<Segment, Segment_Hash, Segment_equal> local_edges)
{
	cloud.clear();
	int n = 9;
	for (auto eit = local_edges.begin(); eit != local_edges.end(); ++eit)
	{
		Segment e = *eit;
		int a = 0;
		int b = n;
		for (int i = 0; i < n - 1; ++i)
		{
			a++;
			b--;
			double xx = (b * e.source().x() + a * e.target().x()) / (double)n;
			double yy = (b * e.source().y() + a * e.target().y()) / (double)n;
			double zz = (b * e.source().z() + a * e.target().z()) / (double)n;
			Point pp(xx, yy, zz);
			apcitem titem;
			titem.p = pp;
			titem.s = e;
			cloud.pts.push_back(titem);
		}
	}

}

int mymesh::MakeCollaps(const Point& s, const Point& t)
{
	vector<Point> stars;
	vector<Point> starp;
	//Vertexs.erase(s);
	

	list<Segment> sedge = Vertexs.at(s).adjacent_edges;
	for (auto evit = sedge.begin(); evit != sedge.end(); ++evit)
	{
		stars.push_back(evit->target());
	}

	list<Segment> pedge = Vertexs.at(t).adjacent_edges;
	for (auto evit = pedge.begin(); evit != pedge.end(); ++evit)
	{
		starp.push_back(evit->target());
	}

	for (auto sit = stars.begin(); sit != stars.end(); ++sit)
	{
		Segment ss1(*sit, s);
		Segment ss2(s, *sit);


		edges.erase(ss1);
		edges.erase(ss2);
		Vertexs.at(*sit).adjacent_edges.erase(
			find(Vertexs.at(*sit).adjacent_edges.begin(), Vertexs.at(*sit).adjacent_edges.end(), ss1)
			);

		if (*sit == t)
		{
			continue;
		}

		Segment si1(*sit, t);
		Segment si2(t, *sit);
		if (find(Vertexs.at(t).adjacent_edges.begin(), Vertexs.at(t).adjacent_edges.end(), si2) == Vertexs.at(t).adjacent_edges.end())
		{
			if (edges.find(si1) == edges.end())
			{
				edge_data ed;
				edges.emplace(si2,ed);
			}
			Vertexs.at(t).adjacent_edges.push_back(si2);
		}

		if (find(Vertexs.at(*sit).adjacent_edges.begin(), Vertexs.at(*sit).adjacent_edges.end(), si1) == Vertexs.at(*sit).adjacent_edges.end())
		{
			//edges.insert(si1);
			Vertexs.at(*sit).adjacent_edges.push_back(si1);
		}




	}
	//pe_map.clear();
	Segment ss1(s, t);
	Segment ss2(t, s);

	edges.erase(ss1);
	edges.erase(ss2);
	Vertexs.at(s).adjacent_edges.clear();
	Vertexs.erase(s);

	//pe_map.at(t).erase(find(pe_map.at(t).begin(),pe_map.at(t).end(),ss2));


	if (stars.empty() && starp.empty())
	{
		Vertexs.erase(t);
		Vertexs.at(t).adjacent_edges.clear();
		Vertexs.erase(t);
		//pe_map.clear();
		return 1;
	}
	else
	{
		//pe_map.clear();
		return 0;
	}

}