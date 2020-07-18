#pragma once
//#include "headers.h"
#include "OTE.h"

std::vector<Point> points_1;
std::vector<Point> points_re;
std::vector<Point> points_ori;

OTE ote;

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

    infile.open("result_edge_fan.xyz", ios::in);
    while (!infile.eof())
    {
        double x, y, z, ma;
        infile >> x >> y >> z;
        Point p(x, y, z, 1);
        points_1.push_back(p);
    }

    infile_ori.open("fan1.xyz", ios::in);
    while (!infile_ori.eof())
    {
        double x, y, z, ma;
        infile_ori >> x >> y >> z;
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
    polyscope::getPointCloud("origin point cloud")->setPointRadius(0.001);
    polyscope::getPointCloud("points on edges")->setPointRadius(0.003);
    ote.InitAddPoint(points_1);
    
}


void addDataToPointCloud(string pointCloudName, const std::vector<glm::vec3>& points) {


    // Add some scalar quantities
    std::vector<double> xC(points.size());
    std::vector<std::array<double, 3>> randColor(points.size());
    for (size_t i = 0; i < points.size(); i++) {
        xC[i] = points[i].x;
        randColor[i] = { {polyscope::randomUnit(), polyscope::randomUnit(), polyscope::randomUnit()} };
    }
    polyscope::getPointCloud(pointCloudName)->addScalarQuantity("xC", xC);
    polyscope::getPointCloud(pointCloudName)->addColorQuantity("random color", randColor);
    polyscope::getPointCloud(pointCloudName)->addColorQuantity("random color2", randColor);


    // Add some vector quantities
    std::vector<glm::vec3> randVec(points.size());
    std::vector<glm::vec3> centerNormalVec(points.size());
    std::vector<glm::vec3> toZeroVec(points.size());
    for (size_t i = 0; i < points.size(); i++) {
        randVec[i] = (float)(10. * polyscope::randomUnit()) *
            glm::vec3{ polyscope::randomUnit(), polyscope::randomUnit(), polyscope::randomUnit() };
        centerNormalVec[i] = glm::normalize(points[i]);
        toZeroVec[i] = -points[i];
    }
    polyscope::getPointCloud(pointCloudName)->addVectorQuantity("random vector", randVec);
    polyscope::getPointCloud(pointCloudName)->addVectorQuantity("unit 'normal' vector", centerNormalVec);
    polyscope::getPointCloud(pointCloudName)->addVectorQuantity("to zero", toZeroVec, polyscope::VectorType::AMBIENT);
}


void ShowCurveNetwork()
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
        size_t i0 = vertex_order.at(s.source());
        size_t i1 = vertex_order.at(s.target());
        edges.push_back({ i0,i1});
    }
    polyscope::registerCurveNetwork("lines1", vertexPositions, edges);
    polyscope::getCurveNetwork("lines1")->setRadius(0.003);
}

void ShowCurveNetwork(float value)
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
        if(eit.second.valid_value<value)
        {
	        continue;
        }
        size_t i0 = vertex_order.at(s.source());
        size_t i1 = vertex_order.at(s.target());
        edges.push_back({ i0,i1 });
    }
    polyscope::registerCurveNetwork("lines2", vertexPositions, edges);
    polyscope::getCurveNetwork("lines2")->setRadius(0.003);

}

void AddDataToEdge()
{
    std::vector<double> edgevalue;
    for (auto eit : ote.ms1.edges)
    {
        edgevalue.push_back(eit.second.valid_value);
    }
    polyscope::getCurveNetwork("lines1")->addEdgeScalarQuantity("edge value",edgevalue);
}

void callback() 
{

    static float value = 0.00;
    static int steps = 0;
    static int steps1 = 0;

    ImGui::PushItemWidth(100);

    //ImGui::InputInt("num points", &numPoints);
    //ImGui::InputFloat("param value", &param);

    ImGui::Value("vertex", (int)ote.ms2.Vertexs.size());

    if (ImGui::Button("1 Step")) 
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
    }

    ImGui::InputInt("until Vertex:", &steps, 1, 100);
    ImGui::SameLine();
	if (ImGui::Button("Go"))
    {
	    while (ote.ms2.Vertexs.size()<steps)
	    {
            ote.addPoint();
	    }
        ShowCurveNetwork();
    }

    if (ImGui::Button("Get valid"))
    {
        ote.CaculateAssinCost();
        ote.GetValid1();      
        ShowCurveNetwork();
    	AddDataToEdge();
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
		ote.CaculateAssinCost();
		ote.RelocateOnce();
		//ote.ReDelauna();
		//ote.CopyDeToMs();
		ote.CaculateAssinCost();
		ShowCurveNetwork();
		AddDataToEdge();
	}

    if (ImGui::InputFloat("threthold", &value, 2, 20))
    {
        ShowCurveNetwork(value);
    }

    if (ImGui::Button("Prepare collaps"))
    {
        ote.InitCollapAfterAdd();
        cout << "fin pre!" << endl;
    }
    ImGui::SameLine();
    if (ImGui::Button("1 Step 1"))
    {
        /*for (int i = 0; i < 1; ++i)
        {*/
            ote.PickAndCollap();
        //}
        ShowCurveNetwork();
    }
    ImGui::SameLine();
    if (ImGui::Button("10 Step 1"))
    {
        for (int i = 0; i < 10; ++i)
        {
            ote.PickAndCollap();
        }
        ShowCurveNetwork();
    }
    ImGui::SameLine();
    if (ImGui::Button("100 Step 1"))
    {
        for (int i = 0; i < 100; ++i)
        {
            ote.PickAndCollap();
        }
        ShowCurveNetwork();
    }
    ImGui::InputInt("until Vertex1:", &steps1, 1, 100);
    ImGui::SameLine();
    if (ImGui::Button("Go1"))
    {
        while (ote.ms2.Vertexs.size() > steps1)
        {
            ote.PickAndCollap();
        }
        ShowCurveNetwork();
    }
    ImGui::PopItemWidth();
}

int main() 
{

    polyscope::init();
    ReadDataAndInit();
    ShowCurveNetwork();
    polyscope::options::maxFPS = -1;
    polyscope::view::upDir = polyscope::view::UpDir::ZUp;  
    polyscope::view::bgColor = array<float, 4>{0.0, 0.0, 0.0, 1.0};
    polyscope::state::userCallback = callback;
    
    polyscope::show();

    return 0;
}
