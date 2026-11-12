// Original pinned VCC CenterVox and MVT, with alignment-only Windows adapters.
#include <array>
#include <memory>
#include <cstdlib>
#include <malloc.h>
#include <chrono>
#include <cstdint>
#include <vector>
#ifdef _WIN32
inline int soc_compat_posix_memalign(void**p,size_t a,size_t n){*p=_aligned_malloc(n,a);return *p?0:12;}
inline void soc_compat_free(void*p){_aligned_free(p);}
inline void* soc_compat_aligned_alloc(size_t a,size_t n){return _aligned_malloc(n,a);}
#define API extern "C" __declspec(dllexport)
#else
inline int soc_compat_posix_memalign(void**p,size_t a,size_t n){return posix_memalign(p,a,n);}
inline void soc_compat_free(void*p){free(p);}
inline void* soc_compat_aligned_alloc(size_t a,size_t n){void*p=nullptr;return posix_memalign(&p,a,n)?nullptr:p;}
#define API extern "C"
#endif
#include <vamp/collision/filter_centervox.hh>
#include <vamp/collision/mvt.hh>
#include <vamp/planning/rrtc.hh>
#include <vamp/robots/panda.hh>
using namespace vamp::collision;
struct Baseline {std::vector<Point>filtered;std::unique_ptr<MVT> mvt;std::vector<std::array<float,7>>path;long long filter_ns=0,build_ns=0,plan_ns=0;bool overflow=false;};
API void* vcc_create(const float*raw,int n,float fine,float maxrange,float half,float coarse_radius){try{auto*b=new Baseline;std::vector<Point> points;for(int i=0;i<n;i++)points.push_back({raw[3*i],raw[3*i+1],raw[3*i+2]});Point lo{-half,-half,-half},hi{half,half,half};auto t=std::chrono::steady_clock::now();CenterSelectiveVoxelFilter f(fine,maxrange,{0,0,0},lo,hi);for(auto&p:points)f.try_insert_point(p);b->filtered=f.extract_points();b->overflow=f.allocated_voxel_count>=f.voxel_pool_size;b->filter_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-t).count();t=std::chrono::steady_clock::now();b->mvt=std::make_unique<MVT>(b->filtered,coarse_radius,lo,hi,0.0025f);b->build_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-t).count();return b;}catch(...){return nullptr;}}
API void vcc_destroy(void*p){delete static_cast<Baseline*>(p);}
API int vcc_points(void*p,float*out){auto&v=static_cast<Baseline*>(p)->filtered;if(out)for(size_t i=0;i<v.size();i++)for(int j=0;j<3;j++)out[3*i+j]=v[i][j];return int(v.size());}
API void vcc_stats(void*p,long long*out){auto&b=*static_cast<Baseline*>(p);out[0]=b.filter_ns;out[1]=b.build_ns;out[2]=b.overflow;out[3]=b.mvt->point_coord_pool_size;out[4]=b.mvt->hierarchy_pool_size_bytes;out[5]=b.mvt->voxel_storage.size();}
API int vcc_query(void*p,const float*c,float radius){auto&b=*static_cast<Baseline*>(p);if(b.filtered.empty())return false;std::array<vamp::FloatVector<>,3>center{vamp::FloatVector<>(c[0]),vamp::FloatVector<>(c[1]),vamp::FloatVector<>(c[2])};return b.mvt->collides_simd(center,vamp::FloatVector<>(radius));}
API long long vcc_query_many(void*p,const float*c,const float*r,int n,uint8_t*out){auto t=std::chrono::steady_clock::now();for(int i=0;i<n;i++)out[i]=vcc_query(p,c+3*i,r[i]);return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-t).count();}
struct SequenceRNG: vamp::rng::RNG<vamp::robots::Panda>{const float*values;int count,index=0;SequenceRNG(const float*v,int n):values(v),count(n){}void reset()noexcept override{index=0;}vamp::FloatVector<7> next()noexcept override{alignas(32) float buffer[8]{};for(int j=0;j<7;j++)buffer[j]=values[(index%count)*7+j];index++;return vamp::FloatVector<7>(buffer);}};
API int vcc_plan(void*p,const float*start,const float*goal,const float*samples,int n,int iterations,int maxnodes,float range,float*out,long long*metrics){if(n<=0)return -1;auto&b=*static_cast<Baseline*>(p);vamp::collision::Environment<vamp::FloatVector<8>>environment;if(!b.filtered.empty())environment.pointclouds_mvt.emplace_back(*b.mvt);alignas(32) float sa[8]{},ga[8]{};std::copy(start,start+7,sa);std::copy(goal,goal+7,ga);vamp::robots::Panda::Configuration a(sa),z(ga);vamp::robots::Panda::ConfigurationBlock<8> block;for(int j=0;j<7;j++)block[j]=vamp::FloatVector<8>(sa[j]);if(!vamp::robots::Panda::fkcc<8>(environment,block))return 2;for(int j=0;j<7;j++)block[j]=vamp::FloatVector<8>(ga[j]);if(!vamp::robots::Panda::fkcc<8>(environment,block))return 3;vamp::planning::RRTCSettings settings;settings.max_iterations=iterations;settings.max_samples=maxnodes;settings.range=range;settings.dynamic_domain=false;settings.balance=false;auto rng=std::make_shared<SequenceRNG>(samples,n);auto result=vamp::planning::RRTC<vamp::robots::Panda,8,32>::solve(a,z,environment,settings,rng);metrics[0]=result.nanoseconds;metrics[1]=result.iterations;metrics[2]=result.path.size();if(result.path.empty())return 1;for(size_t i=0;i<result.path.size();i++){alignas(32) float v[8]{};result.path[i].to_array(v);std::copy(v,v+7,out+i*7);}return 0;}
