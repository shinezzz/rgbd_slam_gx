#include <iostream>
#include <fstream>
#include <sstream>

#include "slamBase.h"
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

FRAME readFrame( int index, ParameterReader& pd );
double normofTransform( cv::Mat rvec, cv::Mat tvec );
int main( int argc, char** argv ){
    ParameterReader pd;
    int startIndex = atoi( pd.getData( "start_index" ).c_str() );
    int endIndex = atoi( pd.getData( "end_index" ).c_str())
    
    cout<<"Initializing ..."<<endl;
    int currIndex = startIndex;
    FRAME lastFrame = readFrame( currIndex,pd );
    string detector = pd.getData( "detector" );
    string descriptor = pd.getData( "descriptor" );
    CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
    computeKeyPointsAndDesp( lastFrame, detector,descriptor);
    PointCloud::Ptr cloud = image2PointCloud( lastFrame.rgb,lastFrame.depth,camera);
    pcl::visualization::CloudViewer viewer("viewer");

    bool visualize = pd.getData("visualize_pointcloud")==string("yes");
    int min_inliers = atoi( pd.getData("min_inliers").c_str());
    double max_norm = atof( pd.getData("max_norm").c_str());

    typedef g2o::BlockSolver_6_3 SlamBlockSolver;
    typedef g2o::LinearSolverEigen< SlamBlockSolver::PoseMatrixType > SlamLinearSolver;

    SlamLinearSolver* linerSolver = new SlamBlockSolver();
    linearSolver->setBlockOrdering( false );
    SlamBlockSolver* blockSolver = new SlamBlockSolver( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( blockSolver );
    //真正使用的g2o全局优化器 
    g2o::SparseOptimizer globalOptimizer;
    globalOptimizer.setAlgorithm( solver );
    // 是否输出调试信息
    globalOptimizer.setVerbose( false );
    // 向优化器增加第一个顶点
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->serId( currIndex );
    v->setEstimate( Eigen::Isometry3d::Identity());
    globalOptimizer.addVertex( v );
    
    int lastIndex = currIndex;

    for ( currIndex=startIndex+1;currIndex<endIndex;currIndex++){
        FRAME currFrame = readFrame( currIndex,pd );
        computeKeyPointsAndDesp( currFrame,detector,descriptor );
        RESULT_OF_PNP result = estimateMotion( lastFrame,currFrame,camera );
        if ( result.inliers < min_inliers )
            continue;
        double norm = normofTransform(result.rvec,result.tvec);
        cout<<"norm= "<<norm<<endl;
        if ( norm >= max_norm )
            continue;
        Eigen::Isometry3d T = cvMat2Eigen( result.rvec, result.tvec );
        cout<<"T="<<T.matrix()<<endl;
        
        // 可视化，可以关掉
        if ( visualize == true ){
            cloud = joinPointCloud( cloud,currFrame,T,camera);
            viewer.showCloud( cloud );
        }

        // g2o增加顶点与上一帧联系的边
        // 设定顶点，只需要设定id
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId( currIndex );
        v->setEstimate( Eigen::Isometry#d::Identity());
        globalOptimizer.addVertex(v);

        // 边部分
        g2o::EdgeSE3* edge = new g2o::EdgeSE3();
        edge->vertices() [0] = globalOptimizer.vertex( lastIndex );
        edge->vertices() [1] = globalOptimizer.vertex( currIndex );

        Eigen::Matrix<double,6,6> information =Eigen::Matrix< double,6,6 >::Identity();

        information(0,0) = information(1,1) = information(2,2) = 100;
        information(3,3) = information(4,4) = information(5,5) = 100;

        edge->setInformation( information );
        edge->setMeasurement(T);
        globalOptimizer.addEdge(edge);
        
        lastFrame = currFrame;
        lastIndex = currIndex;

    }
    // 优化所有边
    cout<<"optimizing pose graph,vertices:"<<globalOptimizer.vertices().size()<<endl;
    globalOptimizer.save("./data/result_before.g2o");
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize(100);
    globalOptimizer.save("./data/result_after.g2o");
    cout<<"Optimization done."<<endl;

    globalOptimizer.clear();
    returen 0;
}

