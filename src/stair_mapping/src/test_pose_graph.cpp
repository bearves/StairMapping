#include <iostream>
#include "PoseGraph.h"

const int N = 100;

int main(int argc, char** argv)
{
    using namespace stair_mapping;
    PoseGraph pg;

    Vertex3d v[N];
    Edge3d e[N-1];
    Vertex3d v_answer[N];

    for (int i = 0; i < N; i++)
    {
        pg.addVertex(v[i]);
    }
    for (int i = 0; i < N-1; i++)
    {
        Pose3d t_edge;
        t_edge.p = Eigen::Vector3d::Random();
        t_edge.q = Eigen::Quaterniond::UnitRandom();
        pg.addEdge(i, i+1, t_edge);

        v_answer[i+1].t_vertex.q = v_answer[i].t_vertex.q * t_edge.q;
        v_answer[i+1].t_vertex.p = v_answer[i].t_vertex.q * t_edge.p + v_answer[i].t_vertex.p;
    }

    try
    {
        bool ret = pg.solve();
    }
    catch (std::runtime_error& ex)
    {
        std::cout << ex.what() << std::endl;
    }

    const std::vector<Vertex3d>* const result = pg.getVertices();

    //for(int i = 0; i < result->size(); i++)
    //{
    //    auto vtx = result->at(i);
    //    std::cout << "Transition:" << vtx.t_vertex.p.transpose()  << std::endl;
    //    std::cout << "Rotation:" << vtx.t_vertex.q.coeffs().transpose() << std::endl;
    //    std::cout << "Transition:" << v_answer[i].t_vertex.p.transpose()  << std::endl;
    //    std::cout << "Rotation:" << v_answer[i].t_vertex.q.coeffs().transpose() << std::endl;
    //    std::cout << "-----------------------------------------" << std::endl;
    //}
    
    return 0;
}