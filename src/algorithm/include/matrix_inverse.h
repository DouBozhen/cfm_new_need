#ifndef MATRIX_INVERSE44_H
#define MATRIX_INVERSE44_H

#define Math_N 4

class MatrixInverse44 
{
public: 
    bool invMatrix44ByGaussianElimination(double src[4][4], double des[4][4]);
    bool invMatrix44ByLUP(double src[4*4], double des[4*4]);

private:
    double getMatrixA(double arcs[4][4], int mat_dimension);
	void getAStart(double arcs[4][4], double ans[4][4], int n = 4);

	double* mul(double mat_a[4*4], double mat_b[4*4]); //矩阵乘法
	void lupDescompose(double src[4*4], double mat_l[4*4], double mat_u[4*4], int mat_p[4]); 
	double* lupSolve(double mat_l[4*4], double mat_u[4*4], int mat_p[4], double mat_b[4]); //LUP求解方程
	int getNext(int i, int m, int n); /* 后继 */
	int getPre(int i, int m, int n); /* 前驱 */
	void moveData(double *mtx, int i, int m, int n); /* 处理以下标i为起点的环 */
	void transPose(double *mtx, int m, int n); /* 转置，即循环处理所有环 */
};

#endif
