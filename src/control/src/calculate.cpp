#include <control/calculate.hpp>
#include <control/Bspline.hpp>

/*************************************************************
* @name Matrix
* @brief 追赶法求解线性方程组
* @constantTerm 线性方程组等号右边的列矩阵
* @solution 解
* @num 行数或列数
* @m 系数矩阵对角数组
* @n 对角上方数组
* @k 对角下方数组
* ***********************************************************/

void Matrix(float* constantTerm, int num,  float* m,  float* n,  float* k,  float* solution)
{
	 
	//a为分解后的下三角矩阵的对角数组
	float* a = NULL;
	a = (float *)malloc(sizeof(float)* num);
	//b为分解后的单位上三角矩阵的对角上方数组
	float* b = NULL;
	b = (float *)malloc(sizeof(float)* (num - 1));
	//c为分解后的单位上三角矩阵的对角上方数组
	float* c = NULL;
	c = (float *)malloc(sizeof(float)* num);
	//x为求解过程中的间接解
	float* x = NULL;
	x = (float *)malloc(sizeof(float)* num);
	int i;

	a[0] = m[0];
	b[0] = n[0] / a[0];

	//给分解后下三角矩阵的对角下方数组c赋值
	for (i = 1; i < num; i++)
	{
		c[i] = k[i];
	}


	//给分解后的单位上三角矩阵的对角上方数组a和分解后的单位上三角矩阵的对角上方数组b赋值
	for (i = 1; i < num - 1; i++)
	{
		a[i] = m[i] - k[i] * b[i - 1];
		b[i] = n[i] / a[i];

	}

	a[num - 1] = m[num - 1] - k[num - 1] * b[num - 2];
	//中间解x的初始值
	x[0] = constantTerm[0] / a[0];

	//给中间解赋值
	for (i = 1; i < num; i++)
	{
		x[i] = (constantTerm[i] - k[i] * x[i - 1]) / a[i];
	}

	//解出最终解
	solution[num - 1] = x[num - 1];

	for (i = num - 1; i > 0; i--)
	{
		solution[i - 1] = x[i - 1] - solution[i] * b[i - 1];
	}
	free(a);
	free(b);
	free(c);
	free(x);
}