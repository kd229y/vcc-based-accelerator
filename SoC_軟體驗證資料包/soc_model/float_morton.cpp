// Floating-point Morton ablation; deliberately separate from the integer SoC model.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>
#ifdef _WIN32
#define API extern "C" __declspec(dllexport)
#else
#define API extern "C"
#endif
using Point=std::array<float,3>;
static uint32_t code(int x,int y,int z){uint32_t k=0;for(int i=0;i<7;i++)k|=((x>>i)&1)<<(3*i),k|=((y>>i)&1)<<(3*i+1),k|=((z>>i)&1)<<(3*i+2);return k;}
static long long ns(std::chrono::steady_clock::time_point t){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-t).count();}
struct Item{uint32_t key;Point p;float distance;};
struct Cell{uint32_t start,count;Point lo,hi;};
struct FloatMorton{
    std::vector<Item> points;std::vector<Cell>cells;std::array<int,32768>lookup;long long stats[6]{};
    FloatMorton(const float*raw,int n,float range){lookup.fill(-1);auto t=std::chrono::steady_clock::now();std::unordered_map<uint32_t,int> map;
        for(int i=0;i<n;i++){Point p{raw[3*i],raw[3*i+1],raw[3*i+2]};float norm=p[0]*p[0]+p[1]*p[1]+p[2]*p[2];if(norm>=range*range)continue;int g[3];float distance=0;bool valid=true;for(int a=0;a<3;a++){g[a]=int(std::floor((p[a]+2)*32));if(g[a]<0||g[a]>=128)valid=false;float d=p[a]-(-2+(g[a]+.5f)*.03125f);distance+=d*d;}if(!valid)continue;auto k=code(g[0],g[1],g[2]);auto it=map.find(k);if(it==map.end()){map[k]=points.size();points.push_back({k,p,distance});}else if(distance<points[it->second].distance)points[it->second]={k,p,distance};}stats[0]=ns(t);t=std::chrono::steady_clock::now();std::vector<Item>temp(points.size());
        for(int sh=0;sh<21;sh+=7){size_t h[128]{},c[128];for(auto&p:points)h[(p.key>>sh)&127]++;size_t sum=0;for(int i=0;i<128;i++){c[i]=sum;sum+=h[i];}for(auto&p:points)temp[c[(p.key>>sh)&127]++]=p;points.swap(temp);}stats[1]=ns(t);t=std::chrono::steady_clock::now();
        for(size_t i=0;i<points.size();){size_t j=i+1;auto key=points[i].key>>6;while(j<points.size()&&(points[j].key>>6)==key)j++;Cell cell{uint32_t(i),uint32_t(j-i),points[i].p,points[i].p};for(size_t a=i;a<j;a++)for(int b=0;b<3;b++){cell.lo[b]=std::min(cell.lo[b],points[a].p[b]);cell.hi[b]=std::max(cell.hi[b],points[a].p[b]);}lookup[key]=cells.size();cells.push_back(cell);i=j;}stats[2]=ns(t);stats[5]=65536+points.size()*12+cells.size()*32;
    }
    bool query(const float*c,float radius){stats[3]++;float r=radius+.0025f;int lo[3],hi[3];for(int a=0;a<3;a++){lo[a]=std::max(0,int(std::floor((c[a]-r+2)*8)));hi[a]=std::min(31,int(std::floor((c[a]+r+2)*8)));}
        for(int z=lo[2];z<=hi[2];z++)for(int y=lo[1];y<=hi[1];y++)for(int x=lo[0];x<=hi[0];x++){int ci=lookup[code(x,y,z)];if(ci<0)continue;auto&cell=cells[ci];float dist=0;for(int a=0;a<3;a++){float d=c[a]<cell.lo[a]?cell.lo[a]-c[a]:c[a]>cell.hi[a]?c[a]-cell.hi[a]:0;dist+=d*d;}if(dist>r*r)continue;for(unsigned i=cell.start;i<cell.start+cell.count;i++){stats[4]++;float d=0;for(int a=0;a<3;a++){float e=c[a]-points[i].p[a];d+=e*e;}if(d<=r*r)return true;}}return false;
    }
};
API void* float_create(const float*p,int n,float range){try{return new FloatMorton(p,n,range);}catch(...){return nullptr;}}
API void float_destroy(void*p){delete static_cast<FloatMorton*>(p);}
API int float_points(void*p,float*out){auto&v=static_cast<FloatMorton*>(p)->points;if(out)for(size_t i=0;i<v.size();i++)std::copy(v[i].p.begin(),v[i].p.end(),out+3*i);return v.size();}
API void float_stats(void*p,long long*out){auto&v=*static_cast<FloatMorton*>(p);std::copy(v.stats,v.stats+6,out);}
API int float_query(void*p,const float*c,float r){return static_cast<FloatMorton*>(p)->query(c,r);}
API long long float_query_many(void*p,const float*c,const float*r,int n,uint8_t*out){auto t=std::chrono::steady_clock::now();for(int i=0;i<n;i++)out[i]=static_cast<FloatMorton*>(p)->query(c+3*i,r[i]);return ns(t);}
