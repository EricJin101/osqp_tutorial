/**
 * MatrixXf      Matrix of floats
 * MatrixXd      Matrix of doubles
 * Matrix3d is a fixed-size 3x3 matrix type of doubles, and \c MatrixXf is a dynamic-size matrix of floats
 * Vector4cf is a fixed-size vector of 4 complex floats
 * */

#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <iostream>
void MatrixXf_pack();
void SparseMatrix_pack();
using namespace Eigen;

void Matrix2d_operate() {
  Matrix2d a;
  a << 1, 2, 3, 4;

  MatrixXd b(2, 2);
  b << 5, 6, 7, 8;
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
  Vector3d v(1, 2, 3);
  Vector3d w(1, 0, 0);
  std::cout << "-v + w - v =\n" << -v + w - v << std::endl;
}

void MatrixXd_pack(){
  Eigen::MatrixXd a(1, 3), b, c;
  a << 1, 2, 3;
  std::cout << "a: \n" << a << "\n\n";

  b.resize(1, 3);
  b << 4, 5, 6;
  std::cout << "b: \n" << b << "\n\n";

  c.resize(2, 3);  // before these operation, give it a size
  c << a, b;
  std::cout << "c: \n" << c << "\n\n";
}

void MatrixXf_pack() {
  Eigen::MatrixXf a, b, c;
  a.resize(2, 2);
  b.resize(2, 2);
  c.resize(5, 5);
  a << 1, 2, 3, 4;
  b << 5, 6, 7, 8;
  c.block<2, 2>(0, 0) = a;
  c.block<2, 2>(2, 1) = b;
  std::cout << "a: \n" << a << "\n\n";
  std::cout << "b: \n" << b << "\n\n";
  std::cout << "c: \n" << c << "\n\n";
}

void SparseMatrix_pack() {
  // sparse matrix can not use
  // c << a, b;
  // c.block<>() = a;
  Eigen::SparseMatrix<double> m1(3, 2), m2(2, 2);
  m1.insert(0, 0) = 1;
  m1.insert(0, 1) = 2;
  m1.insert(1, 0) = 4;
  m1.insert(1, 1) = 5;
  m1.insert(2, 0) = 7;
  m1.insert(2, 1) = 8;
  std::cout << "\nm1 \n" << m1;
  Eigen::SparseMatrix<double> m1_tp = m1.transpose();

  //  Eigen::SparseMatrix<double> m2(3,2);
  m2.insert(0, 0) = 11;
  m2.insert(0, 1) = 12;
  m2.insert(1, 0) = 14;
  m2.insert(1, 1) = 15;

  std::cout << "\nm2 \n" << m2;
  Eigen::SparseMatrix<double> m2_tp = m2.transpose();

  Eigen::SparseMatrix<double> m_top_re(2, 5);

  m_top_re.leftCols(3) = m1_tp;
  m_top_re.rightCols(2) = m2_tp;

  std::cout << m_top_re << "\n\n" << std::endl;
  Eigen::SparseMatrix<double> m_leftcol_re_ = m_top_re.transpose();
  std::cout << m_leftcol_re_ << "\n--\n" << std::endl;
  Eigen::VectorXd b1;
  Eigen::VectorXd b2;
  Eigen::VectorXd b3;
  Eigen::VectorXd b4;
  b1.resize(3);
  b1 << 1, 2, 3;
  std::cout << b1 << std::endl;
  b2.resize(3);
  b2 << 4, 5, 6;
  b3.resize(6);
  b3 << b1, b2;
  std::cout << b3 << std::endl;
  b4.resize(1);
  for (int i(0); i < b3.size(); ++i) {
    b3(i) = 9;
    b4 << 9;
  }
  std::cout << b3 << std::endl;
  std::cout << "\nsize: \n" << b4.size() << std::endl;
  std::cout << b4 << std::endl;
}

int main() {
  Matrix2d_operate();
  MatrixXf_pack();
  SparseMatrix_pack();
  MatrixXd_pack();
  return 0;
}

