#include <cmath>
#include <algorithm>
extern "C" {
static float sample(const float* f,int nx,int ny,double x,double y,double ox,double oy,double step){
 double fx=(x-ox)/step,fy=(y-oy)/step;int a=std::floor(fx),b=std::floor(fy);
 if(a<0||b<0||a>=nx-1||b>=ny-1)return -120;
 double tx=fx-a,ty=fy-b;int k=a*ny+b;
 return (f[k]*(1-ty)+f[k+1]*ty)*(1-tx)+(f[k+ny]*(1-ty)+f[k+ny+1]*ty)*tx;
}
// Rays use camera Z=1, so t is directly comparable to aligned RGB-D Z depth.
void render_surface(const float* f,int nx,int ny,const double* rays,const double* camera,
 const float* depth,const unsigned char* rgb,unsigned char* output,int count,
 double ox,double oy,double step,double z,bool occlusion,double strength){
 for(int i=0;i<count;i++){
  for(int c=0;c<3;c++)output[i*3+c]=rgb[i*3+c];
  const double *v=rays+i*3;
  if(std::abs(v[2])<1e-12)continue;
  double t=(z-camera[2])/v[2];
  if(!std::isfinite(t)||t<=50||t>=8000)continue;
  if(occlusion&&(!(depth[i]>0)||!std::isfinite(depth[i])||t>depth[i]+3))continue;
  double x=camera[0]+v[0]*t,y=camera[1]+v[1]*t;
  double distance=sample(f,nx,ny,x,y,ox,oy,step);
  if(distance<0)continue;
  // A calm, translucent water surface. No object outlines, grid or arrows.
  double sheen=.5+.5*std::sin(x/170.+y/230.);
  double alpha=(distance<3?.68:.42+.06*sheen)*strength;
  double color[3]={35,172,230};
  for(int c=0;c<3;c++)output[i*3+c]=std::round(rgb[i*3+c]*(1-alpha)+color[c]*alpha);
 }
}
}
