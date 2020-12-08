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
	double res = R_MEAN;
	
	for (auto eit:edges)
	{
		int n = sqrt(eit.first.squared_length())/res;
		if(n<=5)
		{
			n = 5;
		}
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
	double res = R_MEAN;

	//int n = 5;
	for (auto eit = local_edges.begin(); eit != local_edges.end(); ++eit)
	{
		int n = sqrt(eit->squared_length()) / res;
		if (n <= 5)
		{
			n = 5;
		}
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


	if (stars.size()==1 && starp.size() == 1)
	{
		//Vertexs.erase(t);
		Vertexs.at(t).adjacent_edges.clear();
		Vertexs.erase(t);
		//cout << "111" << endl;
		//pe_map.clear();
		return 1;
	}
	else
	{
		//pe_map.clear();
		return 0;
	}

}

void mymesh::InsertPoint(Segment e, Point p)
{
	Point s = e.source();
	Point t = e.target();


	Segment tw(t, s);
	//Segment si2(t, p);

	Segment si1(s, p);
	Segment si2(t, p);

	Segment st1(p, s);
	Segment st2(p, t);

	Vertexs.at(s).adjacent_edges.erase(
		find(Vertexs.at(s).adjacent_edges.begin(), Vertexs.at(s).adjacent_edges.end(), e)
	);
	Vertexs.at(t).adjacent_edges.erase(
		find(Vertexs.at(t).adjacent_edges.begin(), Vertexs.at(t).adjacent_edges.end(), tw)
	);

	Vertexs.at(s).adjacent_edges.push_back(si1);
	Vertexs.at(t).adjacent_edges.push_back(si2);
	
	Vertex_data vd;
	vd.adjacent_edges.push_back(st1);
	vd.adjacent_edges.push_back(st2);
	Vertexs.emplace(p, vd);

	edges.erase(e);
	edge_data ed1;
	edge_data ed2;
	edges.emplace(si1, ed1);
	edges.emplace(st2, ed2);
	
	//Vertexs.at(s).adjacent_edges
}

void mymesh::MovePoint(Point s, Point t)
{
	vector<Segment> eraseseg;
	vector<Segment> addseg;

	list<Segment> adj = Vertexs.at(s).adjacent_edges;

	for (auto adjeit : adj)
	{
		Segment twin(adjeit.target(), adjeit.source());
		if (edges.find(adjeit) != edges.end())
		{
			Segment newseg(t, adjeit.target());
			addseg.push_back(newseg);
			eraseseg.push_back(adjeit);

		}
		if (edges.find(twin) != edges.end())
		{
			Segment newseg(twin.source(), t);
			addseg.push_back(newseg);
			eraseseg.push_back(twin);
		}
	}
	list<Segment> map_add;

	for (auto eit : adj)
	{
		Point p = eit.target();
		list<Segment>& se1 = Vertexs.at(p).adjacent_edges;
		for (auto teit : se1)
		{
			if (teit.target() == s)
			{
				se1.remove(teit);
				Segment newsegt(teit.source(), t);
				se1.push_back(newsegt);
				break;
			}
		}
		Segment newseg(t, eit.target());
		map_add.push_back(newseg);
	}

	Vertexs.erase(s);
	Vertex_data vd;
	vd.adjacent_edges = map_add;
	Vertexs.emplace(t, vd);


	for (auto eit = eraseseg.begin(); eit != eraseseg.end(); eit++)
	{
		edges.erase(*eit);
	}
	for (auto ait = addseg.begin(); ait != addseg.end(); ait++)
	{
		edge_data ed;
		edges.emplace(*ait, ed);
	}
}

void mymesh::buildAdj()
{
	for (auto &eit:edges)
	{
		Point s = eit.first.source();
		Point t = eit.first.target();
		Segment twin(t, s);
		Vertexs.at(s).adjacent_edges.push_back(eit.first);
		Vertexs.at(t).adjacent_edges.push_back(twin);
	}
}
