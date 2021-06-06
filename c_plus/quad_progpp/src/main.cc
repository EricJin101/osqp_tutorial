/*
 File main.cc
 
 This file contains just an example on how to set-up the matrices for using with
 the solve_quadprog() function.
 
 The test problem is the following:
 
 Given:
 G =  4 -2   g0^T = [6 0]
     -2  4       
 
 Solve:
 min f(x) = 1/2 x G x + g0 x
 s.t.
   x_1 + x_2 = 3
   x_1 >= 0
   x_2 >= 0
   x_1 + x_2 >= 2
 
 The solution is x^T = [1 2] and f(x) = 12
*/

#include <iostream>
#include <sstream>
#include <string>
#include "QuadProg++.hh"

//int main (int argc, char *const argv[]) {
//  quadprogpp::Matrix<double> G, CE, CI;
//  quadprogpp::Vector<double> g0, ce0, ci0, x;
//  int n, m, p;
//  double sum = 0.0;
//  char ch;
//
//  n = 2; // hessian row/cols
//  G.resize(n, n);
//  { // hessian
//    std::istringstream is("4, -2,"
//                          "-2, 4 ");
//    for (int i = 0; i < n; i++)
//      for (int j = 0; j < n; j++)
//        is >> G[i][j] >> ch;
//  }
//  g0.resize(n);
//  { // gradient
//    std::istringstream is("6.0, 0.0 ");
//    for (int i = 0; i < n; i++)
//      is >> g0[i] >> ch;
//  }
//
//  m = 1; // equality linearMatrix | 1, 1|
//  CE.resize(n, m);
//  {
//    std::istringstream is("1.0, "
//                          "1.0 ");
//
//    for (int i = 0; i < n; i++)
//      for (int j = 0; j < m; j++)
//        is >> CE[i][j] >> ch;
//  }
//
//  ce0.resize(m);
//  { // equality bound
//    std::istringstream is("-3.0 ");
//
//    for (int j = 0; j < m; j++)
//      is >> ce0[j] >> ch;
//  }
//
//  p = 3;
//  CI.resize(n, p);
//  { // inequality linearMatrix
//    std::istringstream is("1.0, 0.0, 1.0, "
//                          "0.0, 1.0, 1.0 ");
//
//    for (int i = 0; i < n; i++)
//      for (int j = 0; j < p; j++)
//        is >> CI[i][j] >> ch;
//  }
//
//  ci0.resize(p);
//  { // lowerBound
//    std::istringstream is("0.0, 0.0, -2.0 ");
//
//    for (int j = 0; j < p; j++)
//      is >> ci0[j] >> ch;
//  }
//  x.resize(n);
//
//  std::cout << "f: " << solve_quadprog(G, g0, CE, ce0, CI, ci0, x) << std::endl;
//  for (int i = 0; i < n; i++){
//    std::cout << "x" << i << ":  " << x[i] << '\n';
//  }
//
//  /* FOR DOUBLE CHECKING COST since in the solve_quadprog routine the matrix G is modified */
//
//  {
//    std::istringstream is("4, -2,"
//                          "-2, 4 ");
//
//    for (int i = 0; i < n; i++)
//      for (int j = 0; j < n; j++)
//        is >> G[i][j] >> ch;
//  }
//
//  std::cout << "Double checking cost: ";
//  for (int i = 0; i < n; i++)
//    for (int j = 0; j < n; j++)
//      sum += x[i] * G[i][j] * x[j];
//  sum *= 0.5;
//
//  for (int i = 0; i < n; i++)
//    sum += g0[i] * x[i];
//  std::cout << sum << std::endl;
//}

template <typename T>
void print(T a){
  for (int i=0; i < a.nrows(); ++i){
    std::cout << " row(" << i << "): ";
    for (int j=0; j < a.ncols(); ++j){
      std::cout << a[i][j] << " ";
    }
    std::cout << std::endl;
  }
}

int main (int argc, char *const argv[]) {
  quadprogpp::Matrix<double> G, CE, CI;
  quadprogpp::Vector<double> g0, ce0, ci0, x;
  int n, m, p;
  double sum = 0.0;
  char ch;

  n = 2;
  G.resize(n, n); // hessian row/cols
  G[0][0] = 4;
  G[0][1] = -2;
  G[1][0] = -2;
  G[1][1] = 4;
  print(G);

  g0.resize(n); // gradient
  g0[0] = 6;
  g0[1] = 0;
  std::cout << g0[0] << " ";
  std::cout << g0[1] << " ";

  m = 1; // equality linearMatrix | 1, 1|
  CE.resize(n, m);
  CE[0][0] = 1;
  CE[1][0] = 1;

  ce0.resize(m); // equality bound
  ce0[0] = -3.0;

  p = 3;
  CI.resize(n, p); // inequality linearMatrix
  CI[0][0] = 1.0;
  CI[0][1] = 0.0;
  CI[0][2] = 1.0;

  CI[1][0] = 0.0;
  CI[1][1] = 1.0;
  CI[1][2] = 1.0;

  ci0.resize(p); // - lowerBound
  ci0[0] = 0.0;
  ci0[1] = 0.0;
  ci0[2] = -2.0;

  x.resize(n);

  std::cout << "f: " << solve_quadprog(G, g0, CE, ce0, CI, ci0, x) << std::endl;
  for (int i = 0; i < n; i++){
    std::cout << "x" << i << ":  " << x[i] << '\n';
  }

  /* FOR DOUBLE CHECKING COST since in the solve_quadprog routine the matrix G is modified */

  {
    std::istringstream is("4, -2,"
                          "-2, 4 ");

    for (int i = 0; i < n; i++)
      for (int j = 0; j < n; j++)
        is >> G[i][j] >> ch;
  }

  std::cout << "Double checking cost: ";
  for (int i = 0; i < n; i++)
    for (int j = 0; j < n; j++)
      sum += x[i] * G[i][j] * x[j];
  sum *= 0.5;

  for (int i = 0; i < n; i++)
    sum += g0[i] * x[i];
  std::cout << sum << std::endl;
}
