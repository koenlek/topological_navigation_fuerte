// test cholesky timing in Eigen

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>


#if 1
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/Cholesky>
using namespace Eigen;
#endif

// some LAPACK Cholesky routines
#ifdef __cplusplus
extern "C" {
#endif

#define F77_FUNC(func)    func ## _
/* Cholesky decomposition, linear system solution and matrix inversion */
extern int F77_FUNC(dpotf2)(char *uplo, int *n, double *a, int *lda, int *info); /* unblocked Cholesky */
extern int F77_FUNC(dpotrf)(char *uplo, int *n, double *a, int *lda, int *info); /* block version of dpotf2 */
extern int F77_FUNC(dpotrs)(char *uplo, int *n, int *nrhs, double *a, int *lda, double *b, int *ldb, int *info);
extern int F77_FUNC(dpotri)(char *uplo, int *n, double *a, int *lda, int *info);

void MAIN__(void) {}

#ifdef __cplusplus
}
#endif

// elapsed time in microseconds
#include <sys/time.h>
static long long utime()
{
  struct timeval tv;
  gettimeofday(&tv,0);
  long long ts = tv.tv_sec;
  ts *= 1000000;
  ts += tv.tv_usec;
  return ts;
}



int main(int argc, char *argv[])
{
  if (argc!=3)
    {
      fprintf(stderr, "Usage is %s <A file> <B file>\n", argv[0]);
      exit(1);
    }

  FILE *fd;
  fd = fopen(argv[1],"r");
  if (fd == NULL)
    {
      printf("File not found\n");
      return -1;
    }
  
  // read array size
  int wx,wy;
  fscanf(fd,"%d %d",&wx,&wy);
  printf("Array size is %d x %d\n",wx,wy);

  // read in A
  int i,j;
  double *A = (double *)malloc(wx*wy*sizeof(double));
  for (i=0; i<wy; i++)
    for (j=0; j<wx; j++)
      fscanf(fd,"%le",&A[j+i*wx]);
  fclose(fd);

  // count non-zeros
  int nnz = 0;
  for (i=0; i<wy; i++)
    for (j=0; j<wx; j++)
      if (A[j+i*wx] != 0.0)
        nnz++;
  printf("Number of nnzs: %d  which is %0.1f percent fill\n",
         nnz, 100.0*(double)nnz/(double)(wx*wy));


  // read in B
  fd = fopen(argv[2],"r");
  if (fd == NULL)
    {
      printf("File not found\n");
      return -1;
    }
  fscanf(fd,"%d",&wx);
  printf("Vector size is %d\n",wx);
  double *B = (double *)malloc(wx*sizeof(double));
  for (i=0; i<wx; i++)
    fscanf(fd,"%le",&B[i]);
  fclose(fd);

#if 1
  // create Eigen vectors
  MatrixXd AA;
  AA.resize(wx,wx);
  VectorXd BB;
  BB.resize(wx);
  for (i=0; i<wx; i++)
    for (j=0; j<wx; j++)
      AA(i,j) = A[j+i*wx];
  
  for (i=0; i<wx; i++)
    BB(i) = B[i];
#endif

  int t0,t1;
  int info;
  int nrhs = 1;
  t0 = utime();
  F77_FUNC(dpotrf)("U", (int *)&wx, A, (int *)&wx, (int *)&info);
  printf("Info is %d after decomposition\n",info);
  F77_FUNC(dpotrs)("U", (int *)&wx, (int *)&nrhs, A, (int *)&wx, B, (int *)&wx, &info);
  printf("Info is %d after solving\n",info);
  t1 = utime();
  printf("Lapack doptrf/doptrs time: %f\n",0.001*(t1-t0));

#if 1
  // now do it with Eigen
  t0 = utime();
  AA.llt().solveInPlace(BB);
  t1 = utime();  
  printf("Eigen llt() time: %f\n",0.001*(t1-t0));  
#endif

  return 0;
}
