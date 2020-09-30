void dir_alloc_mch(const double v[3], const double umin[4], const double umax[4],
                   double u[4])
{
	double B[3][4];
	memset(B, 0, 12U * sizeof(double));
	B[0][0]=-0.5f;
	B[0][2]= 0.5f;
	B[1][1]=-0.5f;
	B[1][3]= 0.5f;
	for(int i=0;i<4;++i)
	{
		B[2][i]=0.25f;
	}
	double b[13];
	memset(b, 0, 13U * sizeof(double));
	for(int i=3;i<7;++i)
	{
		b[i]=umax[i];
	}
	b[7]=50;
	for(int i=8;i<12;++i)
	{
		b[i]=-umin[i];
	}
	double Ad[13][10];
	memset(Ad, 0, 130U * sizeof(double));
	for(int i=0;i<3;++i)
	{
		for(int j=0;j<5;++j)
		{
			if(j<4)
				Ad[i][j]=B[i][j];
			else
				Ad[i][j]=-v[i]
		}
		for(int j=5;j<10;++j)
		{
			if(j<10)
				Ad[i][j]=-B[i][j];
			else
				Ad[i][j]=v[i]
		}
	}
	for(int i=0;i<5;++i)
	{
		Ad[i+3][i]=1.0f;
		Ad[i+3][i+5]=-1.0f;
		Ad[i+8][i]=-1.0f;
		Ad[i+8][i+5]=-1.0f;
	}
	double P[13][13];
	memset(P, 0, 169U * sizeof(double));
	for(int i=0;i<13;++i)
	{
		for(int j=0;j<3;++j)
		{
			P[i][j]=Ad[i][j];
		}
	}
	for(int i=0;i<10;++i)
	{
		P[i+3][i+3]=1.0f;
	}
	double P_inv[13][13];
	memset(P_inv, 0, 169U * sizeof(double));
	matrix_inverse_LU(13, P, P_inv);
	double Ad_eye[13][10];
	memset(Ad_eye, 0, 130U * sizeof(double));
	for(int i=0;i<13;i++)
	{
		for(int j=0;j<10;j++)
		{
			double  temp = 0.0f;
			for(int k = 0 ; k < 13 ; k++)
			{
					temp += P_inv[i][k] * Ad[k][j];
			}
			Ad_eye[i][j] = temp;
		}
	}
	double A1[13][20];
	memset(A1, 0, 260U * sizeof(double));
	for(int i=0;i<13;++i)
	{
		for(int j=0;j<10;++j)
		{
			A1[i][j]=Ad[i][j];
		}
	}
	for(int i=0;i<10;++i)
	{
		A1[i+3][i+10]=1.0f;
	}
	double A[13][20];
	memset(A, 0, 260U * sizeof(double));
	for(int i=0;i<13;i++)
	{
		for(int j=0;j<20;j++)
		{
			double  temp = 0.0f;
			for(int k = 0 ; k < 13 ; k++)
			{
					temp += P_inv[i][k] * A1[k][j];
			}
			A[i][j] = temp;
		}
	}
	double c[20];
	memset(c, 0, 20U * sizeof(double));
	c[4]=-1.0f;
	c[9]= 1.0f;
	int basis[13];
	for(int i=0;i<13;++i)
	{
		if(i<3)
			basis[i]=i+1;
		else
			basis[i]=i+8;
	}
	double z=0;
	
}