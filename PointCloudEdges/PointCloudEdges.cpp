#pragma once
//#include "headers.h"
#include "OTE.h"
#include <time.h>
std::vector<Point> points_1;
std::vector<Point> points_re;
std::vector<Point> points_ori;

std::vector<std::vector<std::vector<Point>>> oriPoints;

std::vector<OTE> otes;
std::vector<OTE> oteblks;
//OTE ote;
string filename = "./meccube/";
double run_time = 0;

void ReadDataAndInit()
{
    ifstream infile;
    ifstream infile_ori;
	//ifstream infile_te;

	//std::set<Point> points_te;
	//infile_te.open("boundary_stg_sd11 - Cloud.txt", ios::in);
	//while (!infile_te.eof())
	//{
	//	double x, y, z, ma;
	//	infile_te >> x >> y >> z;
	//	Point p(x, y, z, 1);
	//	points_te.insert(p);
	//}

	//oriPoints.resize(1000);

    infile.open(filename+"outsplit.xyz", ios::in);
	int maxct = 0;
	int blksize = 0;
    while (!infile.eof())
    {
        double x, y, z;
		int ct, blk;
        infile >> x >> y >> z>>ct>>blk;
		//infile >> x >> y>>z ;
		////z = 0;
		//ct = 0;
		//blk = 0;
        Point p(x, y, z);
    	if(ct+1>maxct)
		{
			oriPoints.resize(ct+1);
			maxct = ct+1;
		}
		if(oriPoints[ct].size()<blk+1)
		{
			oriPoints[ct].resize(1 + blk);
		}
		//oriPoints[ct].resize(1000);
		oriPoints[ct][blk].push_back(p);
		
        points_1.push_back(p);

		
    }


	for (auto &ctit:oriPoints)
	{
		blksize += ctit.size();
	}

    infile_ori.open(filename+"outsplit.xyz", ios::in);
    while (!infile_ori.eof())
    {
        double x, y, z, ma;
		int ct, blk;
		//infile >> x >> y >> z >> ct >> blk;
        infile_ori >> x >> y >> z >> ct >> blk;
		//infile_ori >> x >> y>>z ;
        Point p(x, y, z, 1);
        points_ori.push_back(p);
    }

    std::vector<glm::vec3> points;
    for (size_t i = 0; i < points_1.size(); i++)
    {
        points.push_back(
            glm::vec3{ points_1[i].x(), points_1[i].y(), points_1[i].z() });
    }
    polyscope::registerPointCloud("points on edges", points);
    //addDataToPointCloud("really great points" + std::to_string(j), points);

    std::vector<glm::vec3> points_oriv;
    for (size_t i = 0; i < points_ori.size(); i++)
    {
        points_oriv.push_back(
            glm::vec3{ points_ori[i].x(), points_ori[i].y(), points_ori[i].z() });
    }
    polyscope::registerPointCloud("origin point cloud", points_oriv);
    polyscope::getPointCloud("origin point cloud")->setPointRadius(0.0005);
    polyscope::getPointCloud("points on edges")->setPointRadius(0.001);

	int ctid = 0;
	for (auto& ctit : oriPoints)
	{
		//blksize += ctit.size();
		for (auto &bit:ctit)
		{
			if (bit.size() >= 5)
			{
				OTE tempo;
				tempo.ctidx = ctid;
				tempo.InitAddPoint(bit);
				otes.push_back(tempo);
				
			}
			
		}
		ctid++;
	}

	//ote.InitAddPoint(points_1);
    
}

void updateOriPoints(OTE &ote,int i)
{
	std::vector<glm::vec3> points;
	for (size_t i = 0; i < ote.points_input.size(); i++)
	{
		points.push_back(
			glm::vec3{ ote.points_input[i].x(), ote.points_input[i].y(), ote.points_input[i].z() });
	}
	polyscope::registerPointCloud("points on edges"+std::to_string(i), points);
	polyscope::getPointCloud("points on edges" + std::to_string(i))->setPointRadius(0.001);
}



void ShowCurveNetwork(OTE& ote, int tt)
{
    std::vector<std::array<size_t, 2>> edges;
    std::vector<glm::vec3> vertexPositions;
    unordered_map<Point, int, Point_Hash, Point_equal> vertex_order;
    int i = 0;
    for (auto vit:ote.ms1.Vertexs)
    {
        
        Point p = vit.first;
        vertexPositions.push_back(glm::vec3{ p.x(), p.y(), p.z() });
        vertex_order.insert(pair<Point, int>(p, i));
    	i++;
    }
    for (auto eit:ote.ms1.edges)
    {
        Segment s = eit.first;

		if(vertex_order.find(s.source())==vertex_order.end()||
		   vertex_order.find(s.target()) == vertex_order.end())
		{
			continue;
		}
        size_t i0 = vertex_order.at(s.source());
        size_t i1 = vertex_order.at(s.target());
		
        edges.push_back({ i0,i1});
    }
    polyscope::registerCurveNetwork("lines1" + std::to_string(tt), vertexPositions, edges);
    polyscope::getCurveNetwork("lines1" + std::to_string(tt))->setRadius(0.001);
}

void DeleteCurveNetwork()
{
	for (int i = 0; i < otes.size(); ++i)
	{
		polyscope::removeStructure("lines1" + std::to_string(i+1));
	}
	
	//polyscope::getCurveNetwork("lines1" + std::to_string(tt))->setRadius(0.001);
}

void ShowCurveNetwork(OTE& ote, int tt,float value)
{
    std::vector<std::array<size_t, 2>> edges;
    std::vector<glm::vec3> vertexPositions;
    unordered_map<Point, int, Point_Hash, Point_equal> vertex_order;
    int i = 0;
    for (auto vit : ote.ms1.Vertexs)
    {

        Point p = vit.first;
        vertexPositions.push_back(glm::vec3{ p.x(), p.y(), p.z() });
        vertex_order.insert(pair<Point, int>(p, i));
        i++;
    }
    for (auto eit : ote.ms1.edges)
    {
        Segment s = eit.first;
        if(eit.second.valid_value<value && sqrt(s.squared_length())>=5*R_MEAN)
        {
	        continue;
        }
        size_t i0 = vertex_order.at(s.source());
        size_t i1 = vertex_order.at(s.target());
        edges.push_back({ i0,i1 });
    }
    polyscope::registerCurveNetwork("lines1" + std::to_string(tt), vertexPositions, edges);
    polyscope::getCurveNetwork("lines1" + std::to_string(tt))->setRadius(0.001);

}

void AddDataToEdge(OTE& ote, int i)
{
    std::vector<double> edgevalue;
    for (auto eit : ote.ms1.edges)
    {
        edgevalue.push_back(eit.second.valid_value);
    }
    polyscope::getCurveNetwork("lines1" + std::to_string(i))->addEdgeScalarQuantity("edge value",edgevalue);
}

void Writedata()
{
	ofstream ofile;
	ofile.open(filename + "out1.xyz");
	double res = 0.01;
	for (auto& ote : otes)
	{
		for (auto eit : ote.ms1.edges)
		{
			Segment s = eit.first;
			Point i0 = s.source();
			Point i1 = s.target();
			double length = sqrt(s.squared_length());
			int k = length / res;
			double dx = (i1.x() - i0.x()) / (double)k, dy = (i1.y() - i0.y()) / (double)k, dz = (i1.z() - i0.z()) / (double)k;
			double x = i0.x();
			double y = i0.y();
			double z = i0.z();
			for (int j = 0; j < k; ++j)
			{
				x += dx;
				y += dy;
				z += dz;

				ofile << x << " " << y << " " << z << endl;
			}
			//edges.push_back({ i0,i1 });
		}
	}
	
	ofile.close();
	cout << "write finish" << endl;
}

void writedata_lines()
{
	ofstream ofile;
	ofile.open(filename + "outLines.xyz");
	//double res = 0.01;
	for (auto& ote : otes)
	{
		for (auto eit : ote.ms1.edges)
		{
			Segment s = eit.first;
			Point i0 = s.source();
			Point i1 = s.target();
			
			ofile << i0.x() << " " << i0.y() << " " << i0.z() << " ";
			ofile << i1.x() << " " << i1.y() << " " << i1.z() << endl;
			//edges.push_back({ i0,i1 });
		}
	}

	ofile.close();
	cout << "write finish" << endl;
}

void callback() 
{

    static float value = 0.00;
    static float steps = 20;
    static float steps1 = 10;

    ImGui::PushItemWidth(100);

    //ImGui::InputInt("num points", &numPoints);
    //ImGui::InputFloat("param value", &param);

	int vertexsize = 0;
	for (auto oit:otes)
	{
		vertexsize += oit.ms2.Vertexs.size();
	}

    ImGui::Value("vertex", vertexsize);

  /*  if (ImGui::Button("1 Step")) 
    {
	    for (int i = 0; i < 1; ++i)
	    {
		    ote.addPoint();
	    }       
        ShowCurveNetwork();
    }
    ImGui::SameLine();
    if (ImGui::Button("10 Steps"))
    {
        for (int i = 0; i < 10; ++i)
        {
            ote.addPoint();
        }
        ShowCurveNetwork();
    }
    ImGui::SameLine();
    if (ImGui::Button("100 Steps"))
    {
        for (int i = 0; i < 100; ++i)
        {
            ote.addPoint();
        }
        ShowCurveNetwork();
    }*/

    ImGui::InputFloat("until % Vertex:", &steps, 0.1, 5);
    ImGui::SameLine();
	if (ImGui::Button("Go"))
    {
		clock_t start, end;
		//start = clock(); 

		//end = clock();   
		double percent = steps / 100.0;
		
		start = clock();
		int idx = 0;

#pragma omp parallel for
		for (int i = 0; i < otes.size(); ++i)
		{
			//auto oit = otes[i];
			//idx++;
		/*	int goal = otes[i].oripoints.size() * percent;
			if (goal <= 5)
			{
				goal = 5;
			}
			if (otes[i].oripoints.size() <= 5)
			{
				goal = otes[i].oripoints.size()-1;
			}*/

			/*goal = 200;
			while (otes[i].ms2.Vertexs.size() <= goal)
			{
				otes[i].addPoint();
			}*/

			while (otes[i].endtimes < 3 && otes[i].assin_points.size() >= 2)
			{
				otes[i].addPoint();
			}

			//otes[i].addPoint();
			//ShowCurveNetwork(oit, i+1);
//#pragma omp critical
//			{
//				idx++;
//				cout << idx << endl;
//			}
		}
		end = clock();   //结束时间

		double addtime = double(end - start) / CLOCKS_PER_SEC;
		run_time += addtime;
		cout << "add time:" << addtime << "\ttotaltime:" << run_time << endl;

		
		for (int i = 0; i < otes.size(); ++i)
		{
			auto oit = otes[i];
			ShowCurveNetwork(oit,i+1);
			//cout << idx << endl;
		}


		/*for (auto &oit:otes)
		{
			idx++;
			int goal = oit.oripoints.size() * percent;
			if(goal<=3)
			{
				goal = 3;
			}
			if(oit.oripoints.size()<=4)
			{
				goal = 3;
			}
			while (oit.ms2.Vertexs.size() <= goal)
			{
				oit.addPoint();
			}
			ShowCurveNetwork(oit,idx);
			cout << idx << endl;
		}*/

	    /*while (ote.ms2.Vertexs.size()<steps)
	    {
            ote.addPoint();
	    }*/
        
    }

    if (ImGui::Button("Get valid"))
    {
		clock_t start, end;
		start = clock(); 

		  
		int idx = 0;
		for (auto& oit : otes)
		{
			if (oit.ms2.Vertexs.size() <= 1)
			{
				continue;
			}
			idx++; 
			oit.CaculateAssinCost();
			oit.GetValid1();
			//ShowCurveNetwork(oit, idx);
			//AddDataToEdge(oit, idx);
		}

        end = clock();
		double validtime = double(end - start) / CLOCKS_PER_SEC;
		run_time += validtime;
		cout << "valid time:" << validtime << "\ttotaltime:" << run_time << endl;

		idx = 0;
		for (auto& oit : otes)
		{
			if (oit.ms2.Vertexs.size() <= 1)
			{
				continue;
			}
			idx++;
			ShowCurveNetwork(oit, idx);
			AddDataToEdge(oit, idx);
		}
    	//updateOriPoints();
        //ShowCurveNetwork();
    	
    }
  //  ImGui::SameLine();
  //  if (ImGui::Button("Relocate1"))
  //  {
  //      ote.CaculateAssinCost();
  //      ote.RelocateOnce();
		//ote.ReDelauna();
		//ote.CopyDeToMs();
  //      ote.CaculateAssinCost();
  //      ShowCurveNetwork();
  //      AddDataToEdge();
  //  }
	ImGui::SameLine();
	if (ImGui::Button("Relocate"))
	{
		clock_t start, end;
		start = clock();
		int idx = 0;
		for (auto& oit : otes)
		{
			idx++;
			if(oit.ms2.Vertexs.size()<=1)
			{
				continue;
			}
			oit.CaculateAssinCost();
			oit.RelocateOnce();
			oit.CaculateAssinCost();
			//ShowCurveNetwork(oit, idx);
			//AddDataToEdge(oit, idx);
		}
		end = clock();
		double Relocatetime = double(end - start) / CLOCKS_PER_SEC;
		run_time += Relocatetime;
		cout << "Relocate time:" << Relocatetime << "\ttotaltime:" << run_time << endl;

		idx = 0;
		for (auto& oit : otes)
		{
			if (oit.ms2.Vertexs.size() <= 1)
			{
				continue;
			}
			idx++;
			ShowCurveNetwork(oit, idx);
		}
		//ote.CaculateAssinCost();
		//ote.RelocateOnce();
		////ote.ReDelauna();
		////ote.CopyDeToMs();
		//ote.CaculateAssinCost();
		//ShowCurveNetwork();
		//AddDataToEdge();
	}

    if (ImGui::InputFloat("threshold", &value, 2, 20))
    {
		int idx = 0;
		for (auto& oit : otes)
		{
			idx++;
			//oit.CaculateAssinCost();
			//oit.RelocateOnce();
			//oit.CaculateAssinCost();
			ShowCurveNetwork(oit, idx,value);
			//AddDataToEdge(oit, idx);
		}
        
    }

    if (ImGui::Button("Prepare collaps"))
    {
		clock_t start, end;
		start = clock();
    	
		for (auto& oit : otes)
		{
			oit.InitCollapAfterAdd();
			/*oit.CaculateAssinCost();
			oit.RelocateOnce();
			oit.CaculateAssinCost();
			ShowCurveNetwork(oit, idx);
			AddDataToEdge(oit, idx);*/
		}
		end = clock();
		double pctime = double(end - start) / CLOCKS_PER_SEC;
		run_time += pctime;
		cout << "Prepare collaps time:" << pctime << "\ttotaltime:" << run_time << endl;
       // ote.InitCollapAfterAdd();
        cout << "fin pre!" << endl;
    }
   // ImGui::SameLine();
  //  if (ImGui::Button("1 Step 1"))
  //  {
  //      /*for (int i = 0; i < 1; ++i)
  //      {*/
  //          ote.PickAndCollap();
  //      //}
		//updateOriPoints();
  //      ShowCurveNetwork();
  //  }
  //  ImGui::SameLine();
  //  if (ImGui::Button("10 Step 1"))
  //  {
  //      for (int i = 0; i < 10; ++i)
  //      {
  //          ote.PickAndCollap();
  //      }
		//updateOriPoints();
  //      ShowCurveNetwork();
  //  }
  //  ImGui::SameLine();
  //  if (ImGui::Button("100 Step 1"))
  //  {
  //      for (int i = 0; i < 100; ++i)
  //      {
  //          ote.PickAndCollap();
  //      }
		//updateOriPoints();
  //      ShowCurveNetwork();
  //  }
    ImGui::InputFloat("until Vertex1:", &steps1, 1, 5);
    ImGui::SameLine();
    if (ImGui::Button("Go1"))
    {
		double percent = steps1 / 100.0;
		clock_t start, end;
		start = clock();
		int idx = 0;

#pragma omp parallel for
		for (int i = 0; i < otes.size(); ++i)
		{
			//auto oit = otes[i];
			//idx++;
			/*int goal = otes[i].oripoints.size() * percent;
			if (goal < 5)
			{
				goal = 5;
			}*/
			//while (otes[i].ms2.Vertexs.size() > 8)
			//{
			//	otes[i].PickAndCollap();
			//}

			while (otes[i].endtimes < 3&& otes[i].ms2.Vertexs.size()>=10)
			{
				otes[i].PickAndCollap();
			}

			//updateOriPoints(oit, idx);

			//ShowCurveNetwork(oit, i+1);

//#pragma omp critical
//			{
//				idx++;
//				cout << idx << endl;
//			}
		}

		end = clock();
		double collapstime = double(end - start) / CLOCKS_PER_SEC;
		run_time += collapstime;
		cout << "collaps time:" << collapstime << "\ttotaltime:" << run_time << endl;
    	
		for (int i = 0; i < otes.size(); ++i)
		{
			auto oit = otes[i];
			ShowCurveNetwork(oit, i + 1);
			//cout << idx << endl;
		}
        /*while (ote.ms2.Vertexs.size() > steps1)
        {
            ote.PickAndCollap();
        }*/
		
        //ShowCurveNetwork();
    }
	//ImGui::SameLine();
	if (ImGui::Button("Get validres"))
	{
		int idx = 0;
		for (auto& oit : otes)
		{
			if (oit.ms2.Vertexs.size() <= 1)
			{
				continue;
			}
			idx++;
			oit.CaculateAssinCost();
			oit.GetValidres(value);
			//updateOriPoints(oit, idx);
			ShowCurveNetwork(oit, idx);
		}
	}

	if (ImGui::Button("Prepare Global"))
	{
		DeleteCurveNetwork();
		std::vector<std::vector<Point>> oriP;
		std::vector<std::vector<Point>> seedP;
		std::vector<std::vector<Segment>> seededges;
		std::map<Point, int> PNum;
		oriP.resize(oriPoints.size());
		seedP.resize(oriPoints.size());
		seededges.resize(oriPoints.size());
		//oteblks.resize(oriPoints.size());
		int bid = 0;
		for (auto& oit : otes)
		{
			int ctid = oit.ctidx;
			bid++;
			for (auto& pit:oit.oripoints)
			{
				oriP[ctid].push_back(pit);
			}
			for (auto& vit:oit.ms2.Vertexs)
			{
				seedP[ctid].push_back(vit.first);
				PNum.emplace(vit.first, bid);
			}
			for (auto& eit : oit.ms2.edges)
			{
				seededges[ctid].push_back(eit.first);
			}
		}
		otes.clear();
		for (int i = 0; i < oriPoints.size(); ++i)
		{
			OTE otetemp;
			otetemp.InitGlobalCollape(oriP[i], seedP[i], seededges[i], PNum);
			otes.push_back(otetemp);
		}

		int idx = 0;
		for (auto& oit : otes)
		{
			idx++;
			ShowCurveNetwork(oit, idx);
		}
	}

	ImGui::SameLine();
	if (ImGui::Button("save data"))
	{
		Writedata();
		writedata_lines();
		//AddDataToEdge();
	}
	ImGui::SameLine();
	if (ImGui::Button("merge"))
	{
		int idx = 0;
		for (auto& oit : otes)
		{
			idx++;
			oit.MergeLines();
			//oit.GetValidres(value);
			//updateOriPoints(oit, idx);
			ShowCurveNetwork(oit, idx);
		}
		//ote.MergeLines();
		//ShowCurveNetwork();
		//AddDataToEdge();
	}
    ImGui::PopItemWidth();
}

int main() 
{

    polyscope::init();
    ReadDataAndInit();

	int idx = 0;
	for (auto& oit : otes)
	{
		idx++;
		ShowCurveNetwork(oit, idx);
	}

    //ShowCurveNetwork();
    polyscope::options::maxFPS = -1;
    polyscope::view::upDir = polyscope::view::UpDir::ZUp;  
    polyscope::view::bgColor = array<float, 4>{0.0, 0.0, 0.0, 1.0};
    polyscope::state::userCallback = callback;
	polyscope::options::groundPlaneEnabled = false;
    
    polyscope::show();

    return 0;
}
