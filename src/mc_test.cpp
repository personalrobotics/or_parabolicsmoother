#include "ParabolicRamp.h"
#include "DynamicPath.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
using namespace ParabolicRamp;
using namespace std;

void MonteCarlo1DTest(int num)
{
  ParabolicRamp1D ramp;
  vector<ParabolicRamp1D> ramps;
  int numSolvedMinTime=0,numSolvedMinAccel=0;
  int numSolvedMinTimeBounded=0,numSolvedMinAccelBounded=0;
  for(int iters=0;iters<num;iters++) {
    if((((iters+1)*100)/num) != (iters*100)/num) {
      printf("%d%% ",(iters+1)*100/num);
      fflush(stdout);
    }
    Real xmin = -100.0;
    Real xmax = 100.0;
    Real vmax = Rand()*90+10;
    Real amax = Rand()*90+10;
    ramp.x0 = ((Rand()-0.5)*2.0)*100.0;
    ramp.x1 = ((Rand()-0.5)*2.0)*100.0;
    ramp.dx0 = (Rand()-0.5)*vmax*2;
    ramp.dx1 = (Rand()-0.5)*vmax*2;
    bool res=ramp.SolveMinTime(amax,vmax);
    if(res) {
      numSolvedMinTime++;
      assert(ramp.IsValid());
      Real a,b;
      ramp.DerivBounds(a,b);
      assert(Abs(a) <= vmax);
      assert(Abs(b) <= vmax);
      assert(Abs(ramp.a1) <= amax);
      assert(Abs(ramp.a2) <= amax);
    }
    if(res) {
      Real tmax;
      if(Rand() < 0.01) tmax=ramp.ttotal; //test boundary condition
      else tmax = Max(Rand()*100.0,ramp.ttotal*(Rand()*10.0+1.0));
      res=ramp.SolveMinAccel(tmax,vmax);
      if(res) {
	numSolvedMinAccel++;
	assert(ramp.IsValid());
	Real a,b;
	ramp.DerivBounds(a,b);
	assert(Abs(a) <= vmax);
	assert(Abs(b) <= vmax);
      }
      else {
	printf("Error solving min-accel for a feasible trajectory\n");
	ramp.SolveMinTime(amax,vmax);
	printf("Original: %g (%gs), %g (%gs), %g (%gs)\n",ramp.a1,ramp.tswitch1,ramp.v,ramp.tswitch2,ramp.a2,ramp.ttotal);
	abort();
      }
    }

    //adjust boundary conditions so stopping is possible
    //0 = dx0 + t*amax
    //xmin <= x0 + dx0*t + 1/2t^2amax <= xmax
    //t = -dx0/amax
    //2(xmax-x0)amax >= dx0^2 
    //2(x0-xmin)amax >= dx0^2 
    //dx0 <= sqrt(2 amax(xmax - x0))
    //dx0 >= sqrt(2 amax(x0-xmin))
    Real vamin0,vamax0,vamin1,vamax1;
    vamax0 = Min(Sqrt(2.0*amax*(xmax-ramp.x0)),vmax);
    vamin0 = Max(-Sqrt(2.0*amax*(ramp.x0-xmin)),-vmax);
    vamax1 = Min(Sqrt(2.0*amax*(ramp.x1-xmin)),vmax);
    vamin1 = Max(-Sqrt(2.0*amax*(xmax-ramp.x1)),-vmax);
    ramp.dx0 = vamin0 + Rand()*(vamax0-vamin0);
    ramp.dx1 = vamin1 + Rand()*(vamax1-vamin1);
    res=SolveMinTimeBounded(ramp.x0,ramp.dx0,ramp.x1,ramp.dx1,
			    amax,vmax,xmin,xmax,
			    ramp);
    if(res) {
      numSolvedMinTimeBounded++;
      assert(ramp.IsValid());
      Real a,b;
      ramp.DerivBounds(a,b);
      assert(Abs(a) <= vmax);
      assert(Abs(b) <= vmax);
      assert(Abs(ramp.a1) <= amax);
      assert(Abs(ramp.a2) <= amax);
      ramp.Bounds(a,b);
      assert(a >= xmin-1e-5);
      assert(b <= xmax+1e-5);
    }
    if(res) {
      Real tmax;
      if(Rand() < 0.01) tmax=ramp.ttotal; //test boundary condition
      else tmax = Max(Rand()*100.0,ramp.ttotal*(Rand()*10.0+1.0));
      res=SolveMinAccelBounded(ramp.x0,ramp.dx0,ramp.x1,ramp.dx1,
			       tmax,vmax,xmin,xmax,
			       ramps);
      if(res) {
	numSolvedMinAccelBounded++;
	Real t=0;
	for(size_t i=0;i<ramps.size();i++) {
	  ramp = ramps[i];
	  assert(ramp.IsValid());
	  Real a,b;
	  ramp.DerivBounds(a,b);
	  assert(Abs(a) <= vmax);
	  assert(Abs(b) <= vmax);
	  t += ramps[i].ttotal;
	  ramp.Bounds(a,b);
	  assert(a >= xmin-1e-5);
	  assert(b <= xmax+1e-5);
	}
	assert(FuzzyEquals(t,tmax,1e-5));
      }
      else {
	printf("Error solving bounded min-accel for a feasible trajectory\n");
	SolveMinTimeBounded(ramp.x0,ramp.dx0,ramp.x1,ramp.dx1,
			    amax,vmax,xmin,xmax,
			    ramp);
	printf("Original: %g (%gs), %g (%gs), %g (%gs)\n",ramp.a1,ramp.tswitch1,ramp.v,ramp.tswitch2,ramp.a2,ramp.ttotal);
	abort();
      }
    }
  }
  printf("\n");
  printf("Success rate: %g min time, %g min accel\n",Real(numSolvedMinTime)/Real(num),Real(numSolvedMinAccel)/Real(num));
  printf("   %g min time bounded, %g min accel bounded\n",Real(numSolvedMinTimeBounded)/Real(num),Real(numSolvedMinAccelBounded)/Real(num));
}

void MonteCarloNDTest(int d,int num)
{
  ParabolicRampND ramp;
  vector<vector<ParabolicRamp1D> > splitramps;
  vector<ParabolicRampND> ramps;
  Vector a,b;
  Vector xmin(d,-100.0);
  Vector xmax(d,100.0);
  Vector vmax(d),amax(d);
  int numSolvedMinTime=0,numSolvedMinAccel=0;
  int numSolvedMinTimeBounded=0,numSolvedMinAccelBounded=0;
  for(int iters=0;iters<num;iters++) {
    if((((iters+1)*100)/num) != (iters*100)/num) {
      printf("%d%% ",((iters+1)*100)/num);
      fflush(stdout);
    }
    ramp.x0.resize(d);
    ramp.x1.resize(d);
    ramp.dx0.resize(d);
    ramp.dx1.resize(d);
    for(int i=0;i<d;i++) {
      vmax[i] = Rand()*90+10;
      amax[i] = Rand()*90+10;
      ramp.x0[i] = ((Rand()-0.5)*2.0)*100.0;
      ramp.x1[i] = ((Rand()-0.5)*2.0)*100.0;
      ramp.dx0[i] = (Rand()-0.5)*vmax[i]*2;
      ramp.dx1[i] = (Rand()-0.5)*vmax[i]*2;
    }
    bool res=ramp.SolveMinTime(amax,vmax);
    if(res) {
      numSolvedMinTime++;
      assert(ramp.IsValid());
      ramp.DerivBounds(a,b);
      for(int i=0;i<d;i++) {
	assert(Abs(a[i]) <= vmax[i]);
	assert(Abs(b[i]) <= vmax[i]);
      }
    }
    if(res) {
      Real tmax = ramp.endTime*(Rand()*10.0+1.0);
      res=ramp.SolveMinAccel(vmax,tmax);
      if(res) {
	numSolvedMinAccel++;
	assert(ramp.IsValid());
	assert(FuzzyEquals(tmax,ramp.endTime,1e-5));
	for(int i=0;i<d;i++) {
	  assert(Abs(a[i]) <= vmax[i]);
	  assert(Abs(b[i]) <= vmax[i]);
	}
      }
    }
    //make sure it's reachable
    for(int i=0;i<d;i++) {
      Real vamin0,vamax0,vamin1,vamax1;
      vamax0 = Min(Sqrt(2.0*amax[i]*(xmax[i]-ramp.x0[i])),vmax[i]);
      vamin0 = Max(-Sqrt(2.0*amax[i]*(ramp.x0[i]-xmin[i])),-vmax[i]);
      vamax1 = Min(Sqrt(2.0*amax[i]*(ramp.x1[i]-xmin[i])),vmax[i]);
      vamin1 = Max(-Sqrt(2.0*amax[i]*(xmax[i]-ramp.x1[i])),-vmax[i]);
      ramp.dx0[i] = vamin0 + Rand()*(vamax0-vamin0);
      ramp.dx1[i] = vamin1 + Rand()*(vamax1-vamin1);
    }
    Real ttotal=SolveMinTimeBounded(ramp.x0,ramp.dx0,ramp.x1,ramp.dx1,
				    amax,vmax,xmin,xmax,
				    splitramps);
    if(ttotal >= 0.0) {
      numSolvedMinTimeBounded++;
      CombineRamps(splitramps,ramps);
      for(size_t i=0;i<ramps.size();i++) {
	const ParabolicRampND& ramp = ramps[i];
	assert(ramp.IsValid());
	ramp.DerivBounds(a,b);
	for(int k=0;k<d;k++) {
	  assert(Abs(a[k]) <= vmax[k]);
	  assert(Abs(b[k]) <= vmax[k]);
	  assert(Abs(ramp.ramps[k].a1) <= amax[k]);
	  assert(Abs(ramp.ramps[k].a2) <= amax[k]);
	}
	ramp.Bounds(a,b);
	for(int k=0;k<d;k++) {
	  assert(a[k] >= xmin[k]-1e-5);
	  assert(b[k] <= xmax[k]+1e-5);
	}
      }
    }
    if(ttotal >= 0.0) {
      Real tmax;
      if(Rand() < 0.01) tmax=ttotal; //test boundary condition
      else tmax = Max(Rand()*100.0,ttotal*(Rand()*10.0+1.0));

      res=SolveMinAccelBounded(ramp.x0,ramp.dx0,ramp.x1,ramp.dx1,
			       tmax,vmax,xmin,xmax,
			       splitramps);
      if(res) {
	numSolvedMinAccelBounded++;
	for(size_t i=0;i<splitramps.size();i++) {
	  Real t=0;
	  for(size_t j=0;j<splitramps[i].size();j++)
	    t += splitramps[i][j].ttotal;
	  assert(FuzzyEquals(t,tmax,1e-5));
	}

	CombineRamps(splitramps,ramps);
	Real t=0;
	for(size_t i=0;i<ramps.size();i++) {
	  const ParabolicRampND& ramp = ramps[i];
	  assert(ramp.IsValid());
	  ramp.DerivBounds(a,b);
	  for(int k=0;k<d;k++) {
	    assert(Abs(a[k]) <= vmax[k]);
	    assert(Abs(b[k]) <= vmax[k]);
	  }
	  t += ramp.endTime;
	  ramp.Bounds(a,b);
	  for(int k=0;k<d;k++) {
	    assert(a[k] >= xmin[k]-1e-5);
	    assert(b[k] <= xmax[k]+1e-5);
	  }
	}
	assert(FuzzyEquals(t,tmax,1e-5));
      }
    }
  }
  printf("\n");
  printf("Success rate: %g min time, %g min accel\n",Real(numSolvedMinTime)/Real(num),Real(numSolvedMinAccel)/Real(num));
  printf("   %g min time bounded, %g min accel bounded\n",Real(numSolvedMinTimeBounded)/Real(num),Real(numSolvedMinAccelBounded)/Real(num));
}

int main(int argc,char** argv)
{
  srand(time(NULL));
  printf("Performing monte carlo tests...\n");
  printf("1D, N=1000000\n");
  MonteCarlo1DTest(1000000);
  printf("2D, N=1000000\n");
  MonteCarloNDTest(2,1000000);
  printf("3D, N=1000000\n");
  MonteCarloNDTest(3,1000000);
  printf("100D, N=1000000\n");
  MonteCarloNDTest(100,1000000);
}
