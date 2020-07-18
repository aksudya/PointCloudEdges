#include "OTE.h"

OTE::OTE()
{
}

OTE::~OTE()
{
}

void OTE::InitAddPoint(vector<Point> input)
{
	points_input = input;
	default_random_engine engine(6);
	vector<bool> index(input.size(), false);
	uniform_int_distribution<int> ud(0, input.size() - 1);
	for (int i = 0; i < 3; ++i)
	{
		int id = ud(engine);
		while (index[id])
		{
			id = ud(engine);
		}
		//index[id] = true;
		index[id] = true;
	}

	for (int i = 0; i < input.size(); ++i)
	{
		if (index[i])
		{
			delaunay_input.insert(Point(points_input[i].x(), points_input[i].y(), points_input[i].z()));
		}
		else
		{
			assin_points.push_back(points_input[i]);
		}
	}

	for (auto vit = delaunay_input.finite_vertices_begin(); vit != delaunay_input.finite_vertices_end(); ++vit)
	{
		vector<d_vertex_handle> outit;
		delaunay_input.finite_adjacent_vertices(vit, std::back_inserter(outit));
		list<Segment> adjacent_edge;
		for (auto cvit = outit.begin(); cvit != outit.end(); ++cvit)
		{
			d_vertex_handle v = *cvit;
			Point p1 = v->point();
			Point p2 = vit->point();
			Segment s(p2, p1);
			adjacent_edge.push_back(s);

		}
		Vertex_data vd;
		vd.adjacent_edges = adjacent_edge;
		ms1.Vertexs.emplace(vit->point(), vd);

	}

	for (auto eit = delaunay_input.finite_edges_begin(); eit != delaunay_input.finite_edges_end(); ++eit)
	{

		Point p1 = eit->first->vertex(eit->second)->point();
		Point p2 = eit->first->vertex(eit->third)->point();
		Segment s(p1, p2);
		edge_data ed;
		ms1.edges.emplace(s, ed);
	}

	//pri_cost = 0;
	iter_times = 0;
	ms2 = ms1;
}


void OTE::InitCollapAfterAdd()
{
	less_cost = DBL_MAX;
	pri_cost = 0;
	iter_times = 0;
	debugvalue = 0;
	isCollaps = true;
	//ms2 = ms1;
	to_be_Collaps.clear();
	for (auto eit = ms2.edges.begin(); eit != ms2.edges.end(); ++eit)
	{
		Segment sss = eit->first;
		to_be_Collaps.insert(sss);
		to_be_Collaps.insert(TwinEdge(sss));
	}
}

void OTE::addPoint()
{
	iter_times++;

	//ms1 = ms2;

	ms1.ClearAssin();
	ms2.ClearAssin();
	
	CaculateAssinCost();


	//ReDelauna();

	if (maxc.is_e)
	{
		Point p = ms2.edges.at(maxc.e).assign.maxpoint;
		delaunay_input.insert(p);
		//delaunay_temp.insert(p);
		//cout << "1" << p.mass << " ";
	}
	else
	{
		Point p = ms2.Vertexs.at(maxc.p).assign.maxpoint;
		delaunay_input.insert(p);
		//delaunay_temp.insert(p);
		//cout << p.mass << " ";
	}


	CopyDeToMs();

	ms1 = ms2;

	assin_points.clear();
	for (auto ipit = points_input.begin(); ipit != points_input.end(); ++ipit)
	{
		Point ip = *ipit;
		if (ms2.Vertexs.find(ip) == ms2.Vertexs.end())
		{
			assin_points.push_back(*ipit);
		}
	}


	ms2.ClearAssin();
	//CaculateAssinCost();
	cout << ms2.Vertexs.size() << endl;
}

void OTE::ReDelauna()
{
	delaunay_temp.clear();
	for (auto vit : ms2.Vertexs)
	{
		delaunay_temp.insert(vit.first);
	}
}

void OTE::CopyDeToMs()
{
	ms2.clear();
	for (auto vit = delaunay_input.finite_vertices_begin(); vit != delaunay_input.finite_vertices_end(); ++vit)
	{
		vector<d_vertex_handle> outit;
		delaunay_input.finite_adjacent_vertices(vit, std::back_inserter(outit));
		list<Segment> adjacent_edge;
		for (auto cvit = outit.begin(); cvit != outit.end(); ++cvit)
		{
			d_vertex_handle v = *cvit;
			Point p1 = v->point();
			Point p2 = vit->point();
			Segment s(p2, p1);
			adjacent_edge.push_back(s);

		}
		Vertex_data vd;
		vd.adjacent_edges = adjacent_edge;
		ms2.Vertexs.emplace(vit->point(), vd);

	}

	for (auto eit = delaunay_input.finite_edges_begin(); eit != delaunay_input.finite_edges_end(); ++eit)
	{

		Point p1 = eit->first->vertex(eit->second)->point();
		Point p2 = eit->first->vertex(eit->third)->point();
		Segment s(p1, p2);
		edge_data ed;
		ms2.edges.emplace(s, ed);
	}
}

double OTE::CaculateAssinCost(int local)
{
	if (local == 0 || sample_edge.empty())
	{
		ms2.BuildSampleKDtree();
	}
	else
	{
		ms2.BuildSampleKDtree(sample_edge);
	}
	const PointCloudAdaptor  pc2kd1(ms2.cloud);
	my_kd_tree_t   index1(3 /*dim*/, pc2kd1, KDTreeSingleIndexAdaptorParams(10/* max leaf */));
	index1.buildIndex();

	const size_t num_results = 1;
	size_t ret_index;
	double out_dist_sqr;
	nanoflann::KNNResultSet<double> resultSet(num_results);

	for (auto apit = assin_points.begin(); apit != assin_points.end(); apit++)
	{

		Point pnow = *apit;

		double query_pt[3] = { pnow.x(), pnow.y(), pnow.z() };

		resultSet.init(&ret_index, &out_dist_sqr);
		index1.findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));

		Segment nearest_edge = ms2.cloud.pts[ret_index].s;

		if (ms2.edges.find(nearest_edge) != ms2.edges.end())
		{
			ms2.edges.at(nearest_edge).assign.assined_points.push_back(pnow);
		}
		else
		{
			ms2.edges.at(TwinEdge(nearest_edge)).assign.assined_points.push_back(pnow);
		}
		//ms2.edges.at(nearest_edge).assign.assined_points.push_back(pnow);

	}


	double cost = CaculateEachEdgeCost();
	if (!isCollaps)
	{
		GetMaxCost();
	}

	return cost;
}

void OTE::GetMaxCost()
{
	double maxcost = 0;
	for (const auto& pmit : ms2.Vertexs)
	{

		Point p = pmit.first;
		assignment cost = pmit.second.assign;

		double ttcost = 0;
		double maxdist = 0;
		for (auto pit = cost.assined_points.begin(); pit != cost.assined_points.end(); pit++)
		{
			double dists = squared_distance(*pit, p);
			if (dists > maxdist)
			{
				maxdist = dists;
				cost.maxpoint = *pit;
			}
			ttcost += dists;
		}
		cost.total_cost = ttcost;
		if (cost.total_cost > maxcost)
		{
			maxcost = cost.total_cost;
			maxc.p = p;
			maxc.is_e = false;
		}
		ms2.Vertexs.at(p).assign = cost;
	}

	for (const auto& emit : ms2.edges)
	{
		Segment e = emit.first;
		assignment cost = emit.second.assign;
		if (cost.total_cost > maxcost)
		{
			maxcost = cost.total_cost;
			maxc.e = e;
			maxc.is_e = true;
		}
		ms2.edges.at(e).assign = cost;
	}
}

double OTE::CaculateEachEdgeCost()
{
	double totalcost = 0;
	vector<Segment> Delete_edge;
	for (const auto& eit : ms2.edges)
	{
		Segment e = eit.first;
		assignment ase = eit.second.assign;
		ase.clear_num();
		if (ase.assined_points.empty())
		{
			continue;
		}

		CaculateTangentialCost(e, ase);
		ase.to_edge_cost = ase.normal_cost + ase.tangential_cost;
		CaculateVertexCost(e, ase);
		//cout << cost.to_edge_cost << " " << cost.to_vertex_cost<<endl;
		if (ase.to_edge_cost < ase.to_vertex_cost)		//分配给边
		{
			ase.total_cost = ase.to_edge_cost;
			totalcost += ase.total_cost;
			ms2.edges.at(e).assign = ase;
			//eit.second.assign = ase;
			//cout << "aaa" << endl;
		}
		else									//重新分配给顶点
		{
			ase.total_cost = ase.to_vertex_cost;
			totalcost += ase.total_cost;
			AssinToVertex(e, ase);
			Delete_edge.push_back(e);         //先把要删的边存起来
			//cout << "b" << endl;
		}
	}
	for (auto deit = Delete_edge.begin(); deit != Delete_edge.end(); deit++)		//删除已经分配给顶点的那些边
	{
		ms2.edges.at(*deit).assign.assined_points.clear();
	}
	Delete_edge.clear();
	return sqrt(totalcost);


}

void OTE::CaculateTangentialCost(Segment e, assignment& c)
{
	double tangential_cost = 0;
	double centerCord;		//每段中点的坐标
	const Point ps = e.source();
	const Point pt = e.target();
	//double Me = 0;
	vector<pair<double, Point>> sort_tos;
	//vector<pair<double, Point>> sort_tos;
	//unordered_map<double, Point> sort_tos_m;
	debugvalue++;
	//map<double, Point> sort_tos;
	//cout << c.assined_points.size()<<endl;
	for (auto pit = c.assined_points.begin(); pit != c.assined_points.end(); pit++)
	{
		double dist = PointProjectToSource(e, *pit);
		sort_tos.emplace_back(dist, *pit);
		//sort_tos_m.emplace(dist, *pit);

		//sort_tos.insert(pair<double, Point>(dist, *pit));

	}
	sort(sort_tos.begin(), sort_tos.end());

	double l = sqrt(e.squared_length()) / c.assined_points.size();
	double sql = l * l;
	centerCord = l / 2;		//每段中点的坐标

	centerCord = 0;
	double endcord = 0;

	double normal_cost = 0;
	tangential_cost = 0;
	double max_cost = 0;
	for (auto pit = sort_tos.begin(); pit != sort_tos.end(); pit++)
	{
		double one_point_cost = 0;

		//normal_cost
		Point point = pit->second;
		double li = l;
		//double li = l ;
		double n_cost = squared_distance(point, e);
		normal_cost += n_cost;

		//tangential_cost
		double dist = pit->first;
		centerCord = endcord + (li / 2);

		double sqci = (dist - centerCord) * (dist - centerCord);
		double t_cost = ((li * li) / 12 + sqci);
		//double t_cost = point.mass * ((l * l) / 12 + sqci);
		tangential_cost += t_cost;
		endcord += li;
		//centerCord += l ;

		one_point_cost = n_cost + t_cost;
		if (one_point_cost > max_cost) {
			max_cost = one_point_cost;
			c.maxpoint = point;
		}
	}
	c.normal_cost = normal_cost;
	c.tangential_cost = tangential_cost;
}

double OTE::PointProjectToSource(Segment e, Point p)
{
	const Point ps = e.source();
	Line le = e.supporting_line();
	Point project = le.projection(p);


	double dist = sqrt(squared_distance(project, ps));
	return dist;
}

void OTE::CaculateVertexCost(Segment e, assignment& c)
{
	const Point ps = e.source();
	const Point pt = e.target();
	double CostS = 0;
	double Costt = 0;
	for (auto pit = c.assined_points.begin(); pit != c.assined_points.end(); pit++)
	{
		double dists = squared_distance(*pit, ps);
		double distt = squared_distance(*pit, pt);

		if (dists < distt)
		{
			CostS += dists;
		}
		else
		{
			Costt += distt;
		}
	}
	c.to_vertex_cost = CostS + Costt;
}

void OTE::AssinToVertex(Segment e, assignment& c)
{
	const Point ps = e.source();
	const Point pt = e.target();
	for (auto pit = c.assined_points.begin(); pit != c.assined_points.end(); pit++)
	{
		double dists = squared_distance(*pit, ps);
		double distt = squared_distance(*pit, pt);


		if (dists < distt)	//分配给s
		{
			ms2.Vertexs.at(ps).assign.assined_points.push_back(*pit);
		}
		else
		{
			ms2.Vertexs.at(pt).assign.assined_points.push_back(*pit);
		}
	}
}

void OTE::GetValid1()
{
	ms1 = ms2;
	ms2.clear();
	assin_points.clear();
	for (auto epmit : ms1.edges)
	{

		if (!epmit.second.assign.assined_points.empty() || sqrt(epmit.first.squared_length()) < 0.5)
		{
			Vertex_data vd1;
			Vertex_data vd2;
			//vd.adjacent_edges = adjacent_edge;
			//ms1.Vertexs.emplace(vit->point(), vd);

			edge_data ed;

			double len = epmit.first.squared_length();
			if (epmit.second.assign.assined_points.size() != 0)
			{
				ed.valid_value = len * epmit.second.assign.assined_points.size() / epmit.second.assign.total_cost;
			}
			else
				ed.valid_value = 0;


			ms2.edges.emplace(epmit.first, ed);

			ms2.Vertexs.emplace(epmit.first.source(), vd1);
			ms2.Vertexs.emplace(epmit.first.target(), vd2);

			Segment tw(epmit.first.target(), epmit.first.source());

			ms2.Vertexs.at(epmit.first.source()).adjacent_edges.push_back(epmit.first);
			ms2.Vertexs.at(epmit.first.target()).adjacent_edges.push_back(tw);

		}
	}
	for (auto ipit = points_input.begin(); ipit != points_input.end(); ++ipit)
	{
		Point ip = *ipit;
		if (ms2.Vertexs.find(ip) == ms2.Vertexs.end())
		{
			assin_points.push_back(ip);
		}
	}
	ms1 = ms2;
}

void OTE::RelocateOnce()
{
	vector<Point> newps;
	vector<Point> ppits;
	int nump = 0;
	for (auto pit = ms2.Vertexs.begin(); pit != ms2.Vertexs.end(); ++pit)
	{
		Point newp = Relocatev(pit->first);
		Point ppit = pit->first;
		nump++;
		newps.push_back(newp);
		ppits.push_back(ppit);
	}

	for (int i = 0; i < nump; ++i)
	{
		applyRelocate(ppits[i], newps[i]);
	}
	ms1 = ms2;
}

Point OTE::Relocatev(Point v)
{
	auto vcit = ms2.Vertexs.find(v);

	double sumVpx = 0;
	double sumVpy = 0;
	double sumVpz = 0;

	double sumVp = 0;
	if (vcit != ms2.Vertexs.end())
	{
		assignment v_cost = vcit->second.assign;
		for (auto vcpit = v_cost.assined_points.begin(); vcpit != v_cost.assined_points.end(); vcpit++)
		{
			sumVpx += vcpit->x();
			sumVpy += vcpit->y();
			sumVpz += vcpit->z();
			sumVp++;
		}
	}

	double sumRingEx = 0;
	double sumRingEy = 0;
	double sumRingEz = 0;
	double sumRingElow = 0;
	//auto ceiter = tgl2.incident_edges(v);

	vector<Segment> onestar;

	for (auto eit : ms2.Vertexs.at(v).adjacent_edges)
	{
		onestar.push_back(eit);
	}
	for (int i = 0; i < onestar.size(); ++i)
	{
		Segment curedge = onestar[i];
		Segment twcur(curedge.target(), curedge.source());

		auto cemp_it = ms2.edges.find(curedge);
		double bx = curedge.target().x();
		double by = curedge.target().y();
		double bz = curedge.target().z();
		if (cemp_it != ms2.edges.end())
		{
			assignment v_cost = cemp_it->second.assign;
			for (auto cepit = v_cost.assined_points.begin(); cepit != v_cost.assined_points.end(); cepit++)		//当前边的上的每个点
			{
				double lamuda = Getlamuda(curedge, *cepit);
				double pix = cepit->x();
				double piy = cepit->y();
				double piz = cepit->z();
				sumRingEx += (1 - lamuda) * (pix - lamuda * bx);
				sumRingEy += (1 - lamuda) * (piy - lamuda * by);
				sumRingEz += (1 - lamuda) * (piz - lamuda * bz);
				sumRingElow += (1 - lamuda) * (1 - lamuda);
			}
		}

		auto cemp_it1 = ms2.edges.find(twcur);
		double bx1 = twcur.source().x();		//反向的所以是source
		double by1 = twcur.source().y();
		double bz1 = twcur.source().z();
		if (cemp_it1 != ms2.edges.end())
		{
			assignment v_cost = cemp_it1->second.assign;
			for (auto cepit = v_cost.assined_points.begin(); cepit != v_cost.assined_points.end(); cepit++)		//当前边的上的每个点
			{
				double lamuda = Getlamuda(curedge, *cepit);
				double pix = cepit->x();
				double piy = cepit->y();
				double piz = cepit->z();
				sumRingEx += (1 - lamuda) * (pix - lamuda * bx1);
				sumRingEy += (1 - lamuda) * (piy - lamuda * by1);
				sumRingEz += (1 - lamuda) * (piz - lamuda * bz1);
				sumRingElow += (1 - lamuda) * (1 - lamuda);
			}
		}

	}

	double resx = (sumVpx + sumRingEx) / (sumVp + sumRingElow);
	double resy = (sumVpy + sumRingEy) / (sumVp + sumRingElow);
	double resz = (sumVpz + sumRingEz) / (sumVp + sumRingElow);
	if (sumVp + sumRingElow == 0)		//如果分母为0就代表没变，直接复制
	{
		resx = v.x();
		resy = v.y();
		resz = v.z();
	}

	Point returnp(resx, resy, resz);

	return returnp;
}

double OTE::Getlamuda(Segment e, Point p)
{
	const Point ps = e.source();
	const Point pt = e.target();
	//Segment seg(ps, pt);
	Line le = e.supporting_line();
	Point project = le.projection(p);

	double x1 = ps.x();
	double x2 = pt.x();
	double xp = project.x();

	if (abs(x2 - x1) < 0.001)		//x坐标相差太小的话精度可能较低，换y坐标
	{
		x1 = ps.y();
		x2 = pt.y();
		xp = project.y();
		if (abs(x2 - x1) < 0.001)
		{
			x1 = ps.z();
			x2 = pt.z();
			xp = project.z();
		}
	}

	double res = (xp - x1) / (x2 - x1);


	return res;
}

void OTE::applyRelocate(Point & s, Point & t)
{

	vector<Segment> eraseseg;
	vector<Segment> addseg;

	list<Segment> adj = ms2.Vertexs.at(s).adjacent_edges;

	for (auto adjeit : adj)
	{
		Segment twin(adjeit.target(), adjeit.source());
		if (ms2.edges.find(adjeit) != ms2.edges.end())
		{
			Segment newseg(t, adjeit.target());
			addseg.push_back(newseg);
			eraseseg.push_back(adjeit);

		}
		if (ms2.edges.find(twin) != ms2.edges.end())
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
		list<Segment>& se1 = ms2.Vertexs.at(p).adjacent_edges;
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

	ms2.Vertexs.erase(s);
	Vertex_data vd;
	vd.adjacent_edges = map_add;
	ms2.Vertexs.emplace(t, vd);


	for (auto eit = eraseseg.begin(); eit != eraseseg.end(); eit++)
	{
		ms2.edges.erase(*eit);
	}
	for (auto ait = addseg.begin(); ait != addseg.end(); ait++)
	{
		edge_data ed;
		ms2.edges.emplace(*ait, ed);
	}
}

void OTE::PickAndCollap()
{
	iter_times++;


	//vertex_points_map = vertex_points_map_temp;
	//edge_points_map = edge_points_map_temp;			//为了显示
	//vertex_points_map_temp.clear();
	//edge_points_map_temp.clear();
	//pri_cost = CaculateAssinCost();
	//if(iter_times==1)
	//{
	ms2.ClearAssin();
	pri_cost = CaculateAssinCost();
	ms1 = ms2;
	ms2.ClearAssin();
	//vertex_points_map = vertex_points_map_temp;
	//edge_points_map = edge_points_map_temp;
	////
	//vertex_points_map_temp.clear();
	//edge_points_map_temp.clear();


	UpdatePriQueue();



	//Segment fst_edge = half_edge_queue.top().edge;
	cout << less_cost << endl;
	//pri_cost = half_edge_queue.top().totalcost;

	vector<Point> OneRingPoint;
	//OneRingPoint.clear();
	OneRingPoint = GetOneRingVertex(less_seg.source());


	int re = ms2.MakeCollaps(less_seg.source(), less_seg.target());


	assin_points.clear();
	for (auto ipit = points_input.begin(); ipit != points_input.end(); ++ipit)
	{
		Point ip = *ipit;
		if (ms2.Vertexs.find(ip) == ms2.Vertexs.end())
		{
			assin_points.push_back(ip);
		}
	}
	cout << " ass1:" << assin_points.size() << " ";
	ms1 = ms2;

	to_be_Collaps.clear();

	if (iter_times % 100 == 0 || ms2.Vertexs.size() <= 200)
	{
		for (auto eit = ms2.edges.begin(); eit != ms2.edges.end(); ++eit)
		{
			Segment sss = eit->first;
			to_be_Collaps.insert(sss);
			to_be_Collaps.insert(TwinEdge(sss));
		}
	}
	else
	{
		to_be_Collaps = GetOneRingEdge(OneRingPoint);
	}


	vector<Segment> to_be_erase;
	for (auto& ppit : pri_queue)
	{
		Segment ss = ppit.first;
		/*Segment s = eit.first;
		Segment tw = TwinEdge(s);*/
		if ((ms2.edges.find(ss) == ms2.edges.end()
			&& ms2.edges.find(TwinEdge(ss)) == ms2.edges.end()))
		{
			to_be_erase.push_back(ss);
			//pri_queue.erase(ss);
		}

	}
	for (auto ssit : to_be_erase)
	{
		pri_queue.erase(ssit);
	}
	for (auto tit : to_be_Collaps)
	{
		pri_queue.erase(tit);
	}


	ms2.ClearAssin();
	//edge_points_map_temp.clear();
	CaculateAssinCost();
	cout << iter_times << " ";
}

void OTE::UpdatePriQueue()
{
	int kk = 0;
	//vector<Point> Save_AssinPoints=assin_points;
	for (auto eit = to_be_Collaps.begin(); eit != to_be_Collaps.end(); ++eit)
	{
		kk++;
		ms2 = ms1;
		//assin_points = Save_AssinPoints;
		Segment edge1 = *eit;

		//vertex_points_map_temp = vertex_points_map;
		//edge_points_map_temp = edge_points_map;
		CollectAssPoint(edge1);

		for (auto adje : two_ring_edge)
		{
			if (ms2.edges.find(adje) != ms2.edges.end())
			{
				ms2.edges.at(adje).assign.clearAll();
			}
		}
		for (auto adjp : two_ring_point)
		{
			ms2.Vertexs.at(adjp).assign.clearAll();
		}


		vector<Point> OneRingPoint_t = GetOneRingVertex(edge1.source());
		int re = ms2.MakeCollaps(edge1.source(), edge1.target());

		sample_edge = GetOneRingEdge(OneRingPoint_t, 1);

		//if (re == 1)
		//{
		//	assin_points.push_back(edge1.target());
		//}
		//assin_points.push_back(edge1.source());

		double cost = CaculateAssinCost(1);

		/*pri_queue_item pi;
		pi.edge = edge1;
		pi.cost = cost - pri_cost;*/
		//pi.totalcost = cost;
		pri_queue.emplace(edge1, cost - pri_cost);

	}
	ms2 = ms1;

	less_cost = DBL_MAX;
	for (auto prit : pri_queue)
	{
		if (prit.second < less_cost)
		{
			less_cost = prit.second;
			less_seg = prit.first;
		}
	}
	//debugvalue=0;

	int assPoints = 0;
	for (auto eeit : ms2.edges)
	{
		assPoints += eeit.second.assign.assined_points.size();
	}
	for (auto veit : ms2.Vertexs)
	{
		assPoints += veit.second.assign.assined_points.size();
	}

	cout << "edge: " << ms2.edges.size() << " queuesize:" << pri_queue.size() << " " << ms2.Vertexs.size() << " "
		<< to_be_Collaps.size() << " " << sample_edge.size() << " " << "de:" << debugvalue << " " << "toass:" << assin_points.size()
		<< " assP:" << assPoints << " ";
	debugvalue = 0;
}

void OTE::CollectAssPoint(Segment e)
{
	assin_points.clear();
	vector<Point>  oneRing = GetOneRingVertex(e.source(), 1);
	unordered_set<Segment, Segment_Hash, Segment_equal> edges = GetOneRingEdge(oneRing, 1);

	unordered_set<Point, Point_Hash, Point_equal> plist = GetOneRingVertex(oneRing, 1);
	//list<Point> plist;
	two_ring_edge = edges;


	for (auto eit = edges.begin(); eit != edges.end(); eit++)
	{
		Segment ss = *eit;
		//plist.push_back(eit->source());
		//plist.push_back(eit->target());
		if (ms2.edges.find(ss) != ms2.edges.end())
		{
			vector<Point> eit_ass = ms2.edges.at(ss).assign.assined_points;
			for (auto pit : eit_ass)
			{
				assin_points.push_back(pit);
			}
		}
	}
	//plist.sort();
	//plist.unique();
	two_ring_point = plist;
	for (auto pit = plist.begin(); pit != plist.end(); pit++)
	{
		Point ss = *pit;
		if (ms2.Vertexs.find(ss) != ms2.Vertexs.end())
		{
			vector<Point> eit_ass = ms2.Vertexs.at(ss).assign.assined_points;
			for (auto ppit : eit_ass)
			{
				assin_points.push_back(ppit);
			}
		}
	}
}

vector<Point> OTE::GetOneRingVertex(Point v, int flag)
{
	vector<Point> res;
	list<Segment> adj_edge = ms2.Vertexs.at(v).adjacent_edges;
	for (auto eit = adj_edge.begin(); eit != adj_edge.end(); eit++)
	{
		res.push_back(eit->target());
	}
	if (flag == 1)
	{
		res.push_back(v);
	}

	return res;
}

unordered_set<Point, Point_Hash, Point_equal> OTE::GetOneRingVertex(vector <Point> v, int flag)
{
	unordered_set<Point, Point_Hash, Point_equal> res;
	for (auto vit = v.begin(); vit != v.end(); vit++)
	{
		list<Segment> adj_edge = ms2.Vertexs.at(*vit).adjacent_edges;
		for (auto eit = adj_edge.begin(); eit != adj_edge.end(); eit++)
		{
			res.insert(eit->target());
			if (flag == 1)
			{
				res.insert(*vit);
			}
		}
	}

	return res;
}

unordered_set<Segment, Segment_Hash, Segment_equal> OTE::GetOneRingEdge(vector<Point> s, int flag)
{
	unordered_set<Segment, Segment_Hash, Segment_equal> res;
	unordered_set<Point, Point_Hash, Point_equal> onePoint;
	for (auto pit = s.begin(); pit != s.end(); ++pit)
	{
		list<Segment> adj_edge = ms2.Vertexs.at(*pit).adjacent_edges;
		onePoint.insert(*pit);
		for (auto eit = adj_edge.begin(); eit != adj_edge.end(); eit++)
		{
			onePoint.insert(eit->target());
		}
	}

	for (auto pit = onePoint.begin(); pit != onePoint.end(); ++pit)
	{
		list<Segment> adj_edge = ms2.Vertexs.at(*pit).adjacent_edges;
		for (auto eit = adj_edge.begin(); eit != adj_edge.end(); eit++)
		{
			Segment edge1 = *eit;
			res.insert(edge1);
			if (flag == 1)
			{
				res.insert(TwinEdge(edge1));
			}
		}
	}
	return res;
}