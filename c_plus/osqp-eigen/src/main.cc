
/*
OSQP solves the convex quadratic optimization problem:

min_x 0.5 * x'Hx + f'x
s.t.  l <= Ax <= u
     | 1  -1  1 |       | 2 |
 H = |-1   2 -2 | , f = |-3 |
     | 1  -2  4 |       | 1 |

  0 <= x <= 1 ,  sum (x) = 1/2

 */

// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"
// eigen
#include <Eigen/Dense>
#include <iostream>
int testQP();
int testQP_2();
int eric_simple_test();
void hi();
void hello();
using namespace Eigen;

void stupid_jin(){
  Matrix2d a;
  a << 1, 2,
      3, 4;

  MatrixXd b(2,2);
  b << 5, 6,
      7, 8;
  MatrixXd tem_b(b.rows(), b.cols());
  tem_b.setZero();
  std::cout << "\ntem_b =\n" << tem_b << std::endl;
  std::cout << "-a =\n" << tem_b - a << std::endl;
  std::cout << "a + b =\n" << a + b << std::endl;
  std::cout << "a - b =\n" << a - b << std::endl;
  std::cout << "Doing a += b;" << std::endl;
  a += b;
  std::cout << "Now a =\n" << a << std::endl;

  std::cout << std::endl;
  Vector3d v(1,2,3);
  Vector3d w(1,0,0);
  std::cout << "-v + w - v =\n" << -v + w - v << std::endl;
}

int main()
{

  testQP();
//  testQP_2();
//  hi();
//  hello();
//  eric_simple_test();

  return 0;
}

int testQP()
{
  // allocate QP problem matrices and vectors
  Eigen::SparseMatrix<double> hessian; //P or H
  Eigen::VectorXd gradient;  //f or q
  Eigen::SparseMatrix<double> linearMatrix;  //A
  Eigen::VectorXd lowerBound; //l
  Eigen::VectorXd upperBound; //u

  // 具有线性约束和边界的二次最小化
  hessian.resize(3,3);
  hessian.insert(0,0) = 1;
  hessian.insert(1,0) = -1;
  hessian.insert(2,0) = 1;
  hessian.insert(0,1) = -1;
  hessian.insert(1,1) = 2;
  hessian.insert(2,1) = -2;
  hessian.insert(0,2) = 1;
  hessian.insert(1,2) = -2;
  hessian.insert(2,2) = 4;
  std::cout << "hessian:" << std::endl << hessian << std::endl;

  gradient.resize(3);
  gradient << 2, -3, 1;

  std::cout << "gradient:" << std::endl << gradient << std::endl;

  linearMatrix.resize(4,3);
  linearMatrix.insert(0,0) = 1;
  linearMatrix.insert(1,0) = 0;
  linearMatrix.insert(2,0) = 0;
  linearMatrix.insert(3,0) = 1;

  linearMatrix.insert(0,1) = 0;
  linearMatrix.insert(1,1) = 1;
  linearMatrix.insert(2,1) = 0;
  linearMatrix.insert(3,1) = 1;

  linearMatrix.insert(0,2) = 0;
  linearMatrix.insert(1,2) = 0;
  linearMatrix.insert(2,2) = 1;
  linearMatrix.insert(3,2) = 1;
  std::cout << "linearMatrix:" << std::endl << linearMatrix << std::endl;

  lowerBound.resize(4);
  lowerBound << 0, 0, 0, 0.5;
  std::cout << "lowerBound:" << std::endl << lowerBound << std::endl;

  upperBound.resize(4);
  upperBound << 1, 1, 1, 0.5;
//  upperBound << 0, 0, 0, 0.5;
  std::cout << "upperBound:" << std::endl << upperBound << std::endl;

//  int NumberOfVariables = 3; //A矩阵的列数
  int NumberOfVariables = hessian.rows(); //A矩阵的列数
  int NumberOfConstraints = linearMatrix.rows(); //A矩阵的行数



  // instantiate the solver
  OsqpEigen::Solver solver;

  // settings
  //solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(true);

  // set the initial data of the QP solver
  //矩阵A为m*n矩阵
  solver.data()->setNumberOfVariables(NumberOfVariables); //设置A矩阵的列数，即n
  solver.data()->setNumberOfConstraints(NumberOfConstraints); //设置A矩阵的行数，即m
  if(!solver.data()->setHessianMatrix(hessian)) return 1;//设置P矩阵
  if(!solver.data()->setGradient(gradient)) return 1; //设置q or f矩阵。当没有时设置为全0向量
  if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;//设置线性约束的A矩阵
  if(!solver.data()->setLowerBound(lowerBound)) return 1;//设置下边界
  if(!solver.data()->setUpperBound(upperBound)) return 1;//设置上边界

  // instantiate the solver
  if(!solver.initSolver()) return 1;

  Eigen::VectorXd QPSolution;

  // solve the QP problem
  if(!solver.solve()) return 1;

  // get the controller input
  QPSolution = solver.getSolution();

  std::cout << "QPSolution:" << std::endl << QPSolution << std::endl;
}


void hi(){
  Eigen::MatrixXf a, b,c;
  a.resize(2,2);
  b.resize(2,2);
  c.resize(4,2);
  a << 1, 2,
        3, 4;
  b << 5, 6,
      7, 8;
  c.block<2,2>(0, 0) = a;
  c.block<2,2>(2, 0) = b;
  std::cout << a << "\n\n";
  std::cout << b << "\n\n";
  std::cout << c << "\n\n";

}

void hello(){
  Eigen::SparseMatrix<double> m1(3,2), m2(2,2);
  m1.insert(0,0)=1;
  m1.insert(0,1)=2;
  m1.insert(1,0)=4;
  m1.insert(1,1)=5;
  m1.insert(2,0)=7;
  m1.insert(2,1)=8;
  std::cout << m1 << "\nm1 \n";
  Eigen::SparseMatrix<double> m1_tp  = m1.transpose();

//  Eigen::SparseMatrix<double> m2(3,2);
  m2.insert(0,0)=11;
  m2.insert(0,1)=12;
  m2.insert(1,0)=14;
  m2.insert(1,1)=15;

  std::cout << m2 << "\nm2 \n";
  Eigen::SparseMatrix<double> m2_tp  = m2.transpose();

  Eigen::SparseMatrix<double> m_top_re(2,5);

  m_top_re.leftCols(3) = m1_tp;
  m_top_re.rightCols(2) = m2_tp;

  std::cout<<m_top_re<<"\n\n"<<std::endl;
  Eigen::SparseMatrix<double> m_leftcol_re_  = m_top_re.transpose();
  std::cout<<m_leftcol_re_<<"\n--\n"<<std::endl;
  Eigen::VectorXd b1;
  Eigen::VectorXd b2;
  Eigen::VectorXd b3;
  Eigen::VectorXd b4;
  b1.resize(3);
  b1 << 1,2,3;
  std::cout<<b1<<std::endl;
  b2.resize(3);
  b2 << 4,5,6;
  b3.resize(6);
  b3 << b1, b2;
  std::cout<<b3<<std::endl;
  b4.resize(1);
  for (int i(0); i < b3.size(); ++i){
    b3(i) = 9;
    b4<<9;
  }
  std::cout<<b3<<std::endl;
  std::cout<<"\nsize: \n"<<b4.size()<<std::endl;
  std::cout<<b4<<std::endl;

}


int eric_simple_test(){
    Eigen::SparseMatrix<double> hessian; //P or H
    Eigen::VectorXd gradient;  //f or q
    Eigen::SparseMatrix<double> linearMatrix;  //A
    Eigen::VectorXd lowerBound; //l
    Eigen::VectorXd upperBound; //u

    hessian.resize(2,2);
    hessian.insert(0,0) = 1;
    hessian.insert(1,0) = -1;
    hessian.insert(0,1) = -1;
    hessian.insert(1,1) = 1;

    gradient.resize(2);
    gradient << 1, -3;

    linearMatrix.resize(2,2);
    linearMatrix.insert(0,0) = 1;
    linearMatrix.insert(1,0) = 0;
    linearMatrix.insert(0,1) = 0;
    linearMatrix.insert(1,1) = 1;

    lowerBound.resize(2);
    lowerBound << -1, -1;

    upperBound.resize(2);
    upperBound << 1, 1;

    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    //solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    //矩阵A为m*n矩阵
    solver.data()->setNumberOfVariables(2); //设置A矩阵的列数，即n
    solver.data()->setNumberOfConstraints(2); //设置A矩阵的行数，即m
    if(!solver.data()->setHessianMatrix(hessian)) return 1;//设置P矩阵
    if(!solver.data()->setGradient(gradient)) return 1; //设置q or f矩阵。当没有时设置为全0向量
    if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;//设置线性约束的A矩阵
    if(!solver.data()->setLowerBound(lowerBound)) return 1;//设置下边界
    if(!solver.data()->setUpperBound(upperBound)) return 1;//设置上边界

    // instantiate the solver
    if(!solver.initSolver()) return 1;

    Eigen::VectorXd QPSolution;

    // solve the QP problem
    if(!solver.solve()) return 1;

    // get the controller input
    QPSolution = solver.getSolution();

    std::cout << "QPSolution:" << std::endl << QPSolution << std::endl;
}

int testQP_2()
{
  /**
   * f(x) = 0.5 x1^2 + x2^2 -x1 x2 - 2x1 -6x2
   * s.t.
   *        x1 + x2  <= 2
   *       -x1 + 2x2 <= 2
   *       2x1 + x2  <= 3
   *  H = |  1  -1  |      f = | -2 |       x = |  x1  |
   *      | -1   2  |          | -6 |           |  x2  |
   *
   * A = |   1  1  |       b = |  2  |
   *     |  -1  2  |           |  2  |
   *     |   2  1  |           |  3  |
   * min(  0.5 X^T H X + f^T X  )
   *  A X <= b
   * */
  // allocate QP problem matrices and vectors
  Eigen::SparseMatrix<double> hessian; //P or H
  Eigen::VectorXd gradient;  //f or q
  Eigen::SparseMatrix<double> linearMatrix;  //A
  Eigen::VectorXd lowerBound; //l
  Eigen::VectorXd upperBound; //u

  // 具有线性约束和边界的二次最小化
  hessian.resize(2,2);
  hessian.insert(0,0) = 1;
  hessian.insert(0,1) = -1;
  hessian.insert(1,0) = -1;
  hessian.insert(1,1) = 2;

  std::cout << "hessian:" << std::endl << hessian << std::endl;

  gradient.resize(2);
  gradient << -2, -6;

  std::cout << "gradient:" << std::endl << gradient << std::endl;

  linearMatrix.resize(3,2);
  linearMatrix.insert(0,0) = 1;
  linearMatrix.insert(1,0) = -1;
  linearMatrix.insert(2,0) = 2;

  linearMatrix.insert(0,1) = 1;
  linearMatrix.insert(1,1) = 2;
  linearMatrix.insert(2,1) = 1;

  std::cout << "linearMatrix:" << std::endl << linearMatrix << std::endl;

  lowerBound.resize(3);
  lowerBound << -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY;
//  std::cout << "lowerBound:" << std::endl << lowerBound << std::endl;

  upperBound.resize(3);
  upperBound << 2, 2, 3;
//  upperBound << 0, 0, 0, 0.5;
  std::cout << "upperBound:" << std::endl << upperBound << std::endl;

//  int NumberOfVariables = 3; //A矩阵的列数
  int NumberOfVariables = linearMatrix.cols(); //A矩阵的列数
  int NumberOfConstraints = linearMatrix.rows(); //A矩阵的行数
//  int NumberOfVariables = 2;  // A矩阵的列数
//  int NumberOfConstraints = 3;  // A矩阵的行数



  // instantiate the solver
  OsqpEigen::Solver solver;

  // settings
  //solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(true);

  // set the initial data of the QP solver
  //矩阵A为m*n矩阵
  solver.data()->setNumberOfVariables(NumberOfVariables); //设置A矩阵的列数，即n
  solver.data()->setNumberOfConstraints(NumberOfConstraints); //设置A矩阵的行数，即m
  if(!solver.data()->setHessianMatrix(hessian)) return 1;//设置P矩阵
  if(!solver.data()->setGradient(gradient)) return 1; //设置q or f矩阵。当没有时设置为全0向量
  if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;//设置线性约束的A矩阵
  if(!solver.data()->setLowerBound(lowerBound)) return 1;//设置下边界
  if(!solver.data()->setUpperBound(upperBound)) return 1;//设置上边界

  // instantiate the solver
  if(!solver.initSolver()) return 1;

  Eigen::VectorXd QPSolution;

  // solve the QP problem
  if(!solver.solve()) return 1;

  // get the controller input
  QPSolution = solver.getSolution();

  std::cout << "QPSolution:" << std::endl << QPSolution << std::endl;
}