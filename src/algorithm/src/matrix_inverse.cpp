#include <cstring>
#include <iostream>
#include <math.h>
#include "matrix_inverse.h"

bool MatrixInverse44::invMatrix44ByGaussianElimination(double src[4][4], double des[4][4])
{
    double src_group[4][4] = {};
    memcpy(src_group, src, sizeof(src_group));

    double flag = getMatrixA(src_group, 4);
    double t[4][4];
    if (0 == flag)
    {
        std::cout << "原矩阵行列式为0，无法求逆。请重新运行" << std::endl;
        return false;//如果算出矩阵的行列式为0，则不往下进行
    }
    else
    {
        getAStart(src_group, t, 4);
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                des[i][j] = t[i][j] / flag;
            }

        }
    }

    return true;
}

bool MatrixInverse44::invMatrix44ByLUP(double src[4*4], double des[4*4])
{
    //int N=4;
    //创建矩阵A的副本，注意不能直接用A计算，因为LUP分解算法已将其改变
    double *A_mirror = new double[Math_N*Math_N]();
    double *inv_A = new double[Math_N*Math_N]();//最终的逆矩阵（还需要转置）
    double *inv_A_each = new double[Math_N]();//矩阵逆的各列
    //double *B    =new double[N*N]();
    double *b = new double[Math_N]();//b阵为B阵的列矩阵分量

    for (int i = 0; i < Math_N; i++)
    {
        double *L = new double[Math_N*Math_N]();
        double *U = new double[Math_N*Math_N]();
        int *P = new int[Math_N]();

        //构造单位阵的每一列
        for (int j = 0; j < Math_N; j++)
        {
            b[j] = j == i ? 1 : 0;
        }


        //每次都需要重新将A复制一份
        for (int i = 0; i < Math_N*Math_N; i++)
        {
            A_mirror[i] = src[i];
        }

        lupDescompose(A_mirror, L, U, P);

        inv_A_each = lupSolve(L, U, P, b);
        memcpy(inv_A + i * Math_N, inv_A_each, Math_N * sizeof(double));//将各列拼接起来
    }
    transPose(inv_A, Math_N, Math_N);//由于现在根据每列b算出的x按行存储，因此需转置

    return inv_A;
}


double MatrixInverse44::getMatrixA(double arcs[4][4], int mat_dimension)
{
    if (mat_dimension == 1)
    {
        return arcs[0][0];
    }

    double ans = 0;
    double temp[4][4] = { 0.0 };
    int i, j, k;
    for (i = 0; i < mat_dimension; i++)
    {
        for (j = 0; j < mat_dimension - 1; j++)
        {
            for (k = 0; k < mat_dimension - 1; k++)
            {
                temp[j][k] = arcs[j + 1][(k >= i) ? k + 1 : k];
            }
        }
        double t = getMatrixA(temp, mat_dimension - 1);
        if (i % 2 == 0)
        {
            ans += arcs[0][i] * t;
        }
        else
        {
            ans -= arcs[0][i] * t;
        }
    }
    return ans;
}

void MatrixInverse44::getAStart(double arcs[4][4], double ans[4][4], int n)
{   
    int i, j, k, t;
    double temp[4][4];
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            for (k = 0; k < n - 1; k++)
            {
                for (t = 0; t < n - 1; t++)
                {
                    temp[k][t] = arcs[k >= i ? k + 1 : k][t >= j ? t + 1 : t];
                }
            }

            ans[j][i] = getMatrixA(temp, 3);  //此处顺便进行了转置
            if ((i + j) % 2 == 1)
            {
                ans[j][i] = -ans[j][i];
            }
        }
    }
}

double* MatrixInverse44::mul(double mat_a[4*4], double mat_b[4*4])
{
    double *result = new double[16]{};
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            for (int k = 0; k < 4; k++)
            {
                result[i*4 + j] += mat_a[i*4 + k] * mat_b[k*4 + j];
            }
        }
    }

    //若绝对值小于10^-10,则置为0（这是我自己定的）
    for (int i = 0; i < 4*4; i++)
    {
        if (abs(result[i]) < pow(10, -10))
        {
            result[i] = 0;
        }
    }
    return result;
}

void MatrixInverse44::lupDescompose(double src[4*4], double mat_l[4*4], double mat_u[4*4], int mat_p[4])
{
    int row = 0;
    for (int i = 0; i < Math_N; i++)
    {
        mat_p[i] = i;
    }
    for (int i = 0; i < Math_N - 1; i++)
    {
        double p_value = 0.0;
        for (int j = i; j < Math_N; j++)
        {
            if (abs(src[j*Math_N + i]) > p_value)
            {
                p_value = abs(src[j*Math_N + i]);
                row = j;
            }
        }
        if (0 == p_value)
        {
            std::cout << "矩阵奇异，无法计算逆" << std::endl;
            return;
        }

        //交换P[i]和P[row]
        int tmp = mat_p[i];
        mat_p[i] = mat_p[row];
        mat_p[row] = tmp;

        double tmp2 = 0.0;
        for (int j = 0; j < Math_N; j++)
        {
            //交换A[i][j]和 A[row][j]
            tmp2 = src[i*Math_N + j];
            src[i*Math_N + j] = src[row*Math_N + j];
            src[row*Math_N + j] = tmp2;
        }

        //以下同LU分解
        double u_value = src[i*Math_N + i], l_value = 0.0;
        for (int j = i + 1; j < Math_N; j++)
        {
            l_value = src[j*Math_N + i] / u_value;
            src[j*Math_N + i] = l_value;
            for (int k = i + 1; k < Math_N; k++)
            {
                src[j*Math_N + k] = src[j*Math_N + k] - src[i*Math_N + k] * l_value;
            }
        }
    }

    //构造L和U
    for (int i = 0; i < Math_N; i++)
    {
        for (int j = 0; j <= i; j++)
        {
            if (i != j)
            {
                mat_l[i*Math_N + j] = src[i*Math_N + j];
            }
            else
            {
                mat_l[i*Math_N + j] = 1;
            }
        }
        for (int k = i; k < Math_N; k++)
        {
            mat_u[i*Math_N + k] = src[i*Math_N + k];
        }
    }
}

double* MatrixInverse44::lupSolve(double mat_l[4*4], double mat_u[4*4], int mat_p[4], double mat_b[4])
{
  	double *mat_x = new double[Math_N]();
    double *mat_y = new double[Math_N]();

    //正向替换
    for (int i = 0; i < Math_N; i++)
    {
        mat_y[i] = mat_b[mat_p[i]];
        for (int j = 0; j < i; j++)
        {
            mat_y[i] = mat_y[i] - mat_l[i*Math_N + j] * mat_y[j];
        }
    }
    //反向替换
    for (int i = Math_N - 1; i >= 0; i--)
    {
        mat_x[i] = mat_y[i];
        for (int j = Math_N - 1; j > i; j--)
        {
            mat_x[i] = mat_x[i] - mat_u[i*Math_N + j] * mat_x[j];
        }
        mat_x[i] /= mat_u[i*Math_N + i];
    }
    return mat_x;  
}

int MatrixInverse44::getNext(int i, int m, int n)
{
    return (i%n)*m + i / n;
}

int MatrixInverse44::getPre(int i, int m, int n)
{
    return (i%m)*n + i / m;
}

void MatrixInverse44::moveData(double *mtx, int i, int m, int n)
{
    double temp = mtx[i]; // 暂存
    int cur = i;    // 当前下标
    int pre = getPre(cur, m, n);
    while (pre != i)
    {
        mtx[cur] = mtx[pre];
        cur = pre;
        pre = getPre(cur, m, n);
    }
    mtx[cur] = temp;
}

void MatrixInverse44::transPose(double *mtx, int m, int n)
{
    for (int i = 0; i < m*n; ++i)
    {
        int next = getNext(i, m, n);
        while (next > i) // 若存在后继小于i说明重复,就不进行下去了（只有不重复时进入while循环）
            next = getNext(next, m, n);
        if (next == i)  // 处理当前环
            moveData(mtx, i, m, n);
    }
}

