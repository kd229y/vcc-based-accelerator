// Integer functional model. Offline generated constants are the only floating-point math source.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>
#include "panda_generated.hh"
#include "upstream_fk_scalar.hh"
#ifdef _WIN32
#define API extern "C" __declspec(dllexport)
#else
#define API extern "C"
#endif
using I=long long;
constexpr I CS=8192, AS=1LL<<24, RS=1LL<<30;
using P=std::array<int,3>;
using Q=std::array<int,7>;
enum {INPUT,KEPT,UPDATES,PROBES,MAXPROBE,RADIX_READ,RADIX_WRITE,QUERIES,GLOBAL_REJECT,CELLS,CELL_REJECT,DISTANCE,FKCALL,NNCALL,NNDIST,EDGECALL,EDGESAMPLE,NODES,ITERATIONS,SRAM_R,SRAM_W,READ_BYTES,WRITE_BYTES,BUILD_NS,QUERY_NS,FK_NS,PLAN_NS,PEAK_BYTES,TRACE_DROPPED,STATECALL,CULLED,NUM_STATS};
enum {BUILD_OP=1,FK_OP,QUERY_OP,NN_OP,EDGE_OP,SAMPLE_OP,INSERT_OP,CONNECT_OP,PLAN_OP,MEM_OP};
enum {SUCCESS=0,BUDGET=1,INVALID_START=2,INVALID_GOAL=3,CAPACITY=4,INVALID_INPUT=5,INTERNAL_ERROR=6};
static I elapsed(std::chrono::steady_clock::time_point t){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-t).count();}
static I floor_div(I n,I d){I q=n/d,r=n%d;return q-(r<0);}
static I round_div(I n,I d){return floor_div(n+d/2,d);}
static uint64_t isqrt(uint64_t n){uint64_t r=0,b=1ULL<<62;while(b>n)b>>=2;while(b){if(n>=r+b){n-=r+b;r=(r>>1)+b;}else r>>=1;b>>=2;}return r;}
static I ceil_sqrt(uint64_t n){I r=isqrt(n);return r*r==I(n)?r:r+1;}
static uint32_t morton(int x,int y,int z){uint32_t k=0;for(int b=0;b<7;b++)k|=((x>>b)&1)<<(3*b),k|=((y>>b)&1)<<(3*b+1),k|=((z>>b)&1)<<(3*b+2);return k;}
static int float_floor_q(float value,int frac,int &status){uint32_t u;std::memcpy(&u,&value,4);int e=(u>>23)&255;uint64_t m=u&0x7fffff;bool negative=u>>31;if(e==255){status=INVALID_INPUT;return 0;}if(e)m|=1U<<23;int sh=(e?e-127:1-127)-23+frac;uint64_t v=0,rem=0;if(sh>=0){if(sh>30 || m>(uint64_t(INT32_MAX)>>sh)){status=INVALID_INPUT;return 0;}v=m<<sh;}else{int r=-sh;if(r>=64){v=0;rem=m;}else{v=m>>r;rem=m&((1ULL<<r)-1);}}I result=negative?-I(v)-(rem?1:0):I(v);if(result<INT32_MIN||result>INT32_MAX)status=INVALID_INPUT;return int(result);}
static void cordic(int a,I &s,I &c){I z=I(a)*16;constexpr I PI=843314857,HALF=421657428;I sign=1;if(z>HALF){z-=PI;sign=-1;}else if(z<-HALF){z+=PI;sign=-1;}I x=652032874,y=0;for(int i=0;i<30;i++){I sx=floor_div(x,1LL<<i),sy=floor_div(y,1LL<<i);if(z>=0){I nx=x-sy;y+=sx;x=nx;z-=ATAN_Q28[i];}else{I nx=x+sy;y-=sx;x=nx;z+=ATAN_Q28[i];}}s=sign*y;c=sign*x;}
struct Transform {I r[9]{RS,0,0,0,RS,0,0,0,RS};I t[3]{};};
static Transform compose(const Transform&a,const Transform&b){Transform o;for(int i=0;i<3;i++){for(int j=0;j<3;j++){I v=0;for(int k=0;k<3;k++)v+=a.r[3*i+k]*b.r[3*k+j];o.r[3*i+j]=round_div(v,RS);}I v=0;for(int k=0;k<3;k++)v+=a.r[3*i+k]*b.t[k];o.t[i]=a.t[i]+round_div(v,RS);}return o;}
static std::array<P,NS> integer_fk(const Q&q){Transform links[NL];for(const auto&d:JOINTS){Transform local;std::copy(d.r,d.r+9,local.r);std::copy(d.t,d.t+3,local.t);if(d.index>=0){I s,c;cordic(q[d.index],s,c);Transform turn;turn.r[0]=turn.r[4]=c;turn.r[1]=-s;turn.r[3]=s;local=compose(local,turn);}links[d.child]=compose(links[d.parent],local);}std::array<P,NS> out;for(int n=0;n<NS;n++){auto &d=SPHERES[n];auto&t=links[d.link];for(int i=0;i<3;i++){I v=0;for(int k=0;k<3;k++)v+=t.r[3*i+k]*d.p[k];I p=t.t[i]+round_div(v,RS);out[n][i]=int(round_div(p,1LL<<11));}}return out;}
struct Rep {uint32_t key;P p,lo,hi;int d;uint32_t source;int cover=0;};
struct Cell {uint32_t key,start,count;P lo,hi;};
struct Node {Q q;int parent;int tree;};
struct Record {I kind,id,version,address,read_bytes,write_bytes,units,result;};
using EventCallback=void(*)(const I*,void*);
struct Model {
    I cfg[16]{};I stats[NUM_STATS]{};int error=SUCCESS;I version=1,qid=0;
    std::vector<Rep> points;std::vector<Cell> cells;std::vector<int> lookup;std::vector<uint32_t> offsets;
    std::vector<Record> records;std::vector<Q> path;int maxcover=0;P global_lo{},global_hi{};
    EventCallback callback=nullptr;void* callback_context=nullptr;
    explicit Model(const I*c){std::copy(c,c+16,cfg);lookup.assign(32768,-1);offsets.assign(32769,0);}
    void record(I kind,I id,I address,I rb,I wb,I units,I result=0){Record row{kind,id,version,address,rb,wb,units,result};if(callback)callback(reinterpret_cast<const I*>(&row),callback_context);if(cfg[10]>0&&records.size()<size_t(cfg[10]))records.push_back(row);else if(cfg[10]>0)stats[TRACE_DROPPED]++;}
    void qrecord(I kind,I id,const Q&q){record(kind,id,q[0],q[1],q[2],q[3],q[4]);record(kind+1,id,q[5],q[6],0,0,0);}
    void mem(I id,I address,I rb,I wb,I units=0){stats[SRAM_R]+=rb>0;stats[SRAM_W]+=wb>0;stats[READ_BYTES]+=rb;stats[WRITE_BYTES]+=wb;record(MEM_OP,id,address,rb,wb,units);}
    int build(const float*raw,int n){auto t=std::chrono::steady_clock::now();std::fill(stats,stats+NUM_STATS,0);points.clear();cells.clear();path.clear();records.clear();std::fill(lookup.begin(),lookup.end(),-1);std::fill(offsets.begin(),offsets.end(),0);maxcover=0;error=SUCCESS;stats[INPUT]=n;
        if(n<0||cfg[0]<2||(cfg[0]&(cfg[0]-1))||cfg[1]<1||cfg[1]>8){return error=INVALID_INPUT;}
        std::vector<int> slots(size_t(cfg[0]),-1);int shift=8-int(cfg[1]);
        for(int i=0;i<n;i++){P p;int status=0;for(int a=0;a<3;a++)p[a]=float_floor_q(raw[i*3+a],13,status);if(status){points.clear();return error=status;}uint64_t norm=0;for(auto v:p)norm+=uint64_t(I(v)*v);if(cfg[14]>0&&norm>=(cfg[14]+2)*(cfg[14]+2)){stats[CULLED]++;continue;}bool outside=false;for(auto v:p)if(v<-16384||v>=16384)outside=true;if(outside){points.clear();return error=INVALID_INPUT;}int f[3],d=0;for(int a=0;a<3;a++){int rel=p[a]+16384;f[a]=rel>>8;int o=(rel&255)>>shift;int e=2*o+1-(1<<cfg[1]);d+=e*e;}uint32_t k=morton(f[0],f[1],f[2]);uint32_t slot=(k*2654435761U)&(cfg[0]-1);int tries=0;while(true){stats[PROBES]++;tries++;if(slots[slot]<0){if(points.size()*4>=size_t(cfg[0]*3)){points.clear();return error=CAPACITY;}slots[slot]=int(points.size());points.push_back({k,p,p,p,d,uint32_t(i),0});stats[UPDATES]++;mem(0,0x100000+slot*32,0,32);break;}Rep&r=points[slots[slot]];mem(0,0x100000+slot*32,32,0);if(r.key==k){for(int a=0;a<3;a++){r.lo[a]=std::min(r.lo[a],p[a]);r.hi[a]=std::max(r.hi[a],p[a]);}if(d<r.d){r.p=p;r.d=d;r.source=i;stats[UPDATES]++;}mem(0,0x100000+slot*32,0,32);break;}slot=(slot+1)&(cfg[0]-1);if(tries>=cfg[0]){points.clear();return error=CAPACITY;}}stats[MAXPROBE]=std::max(stats[MAXPROBE],I(tries));}
        for(auto&r:points){uint64_t d=0;for(int a=0;a<3;a++){I v=std::max(std::abs(r.lo[a]-r.p[a]),std::abs(r.hi[a]-r.p[a]));d+=v*v;}r.cover=cfg[2]?int(ceil_sqrt(d))+2:2;maxcover=std::max(maxcover,r.cover);}
        std::vector<Rep> temp(points.size());for(int shiftbits=0;shiftbits<21;shiftbits+=7){size_t hist[128]{},cursor[128];for(auto&r:points)hist[(r.key>>shiftbits)&127]++;size_t s=0;for(int b=0;b<128;b++){cursor[b]=s;s+=hist[b];}for(auto&r:points)temp[cursor[(r.key>>shiftbits)&127]++]=r;points.swap(temp);stats[RADIX_READ]+=points.size();stats[RADIX_WRITE]+=points.size();}
        for(size_t i=0;i<points.size();){size_t j=i+1;uint32_t k=points[i].key>>6;Cell c{k,uint32_t(i),0,points[i].p,points[i].p};while(j<points.size()&&(points[j].key>>6)==k)j++;c.count=j-i;for(size_t a=i;a<j;a++)for(int b=0;b<3;b++){c.lo[b]=std::min(c.lo[b],points[a].p[b]);c.hi[b]=std::max(c.hi[b],points[a].p[b]);}lookup[k]=int(cells.size());offsets[k]=c.count;cells.push_back(c);i=j;}
        uint32_t sum=0;for(int i=0;i<32769;i++){uint32_t count=offsets[i];offsets[i]=sum;sum+=count;}
        if(!points.empty()){global_lo=global_hi=points[0].p;for(auto&r:points)for(int a=0;a<3;a++){global_lo[a]=std::min(global_lo[a],r.p[a]);global_hi[a]=std::max(global_hi[a],r.p[a]);}}
        stats[KEPT]=points.size();I envbytes=32768*2+32769*4+cells.size()*20+points.size()*8;stats[PEAK_BYTES]=cfg[0]*32+2*envbytes+2*cfg[3]*36+points.size()*16;
        if(stats[PEAK_BYTES]>cfg[8]){points.clear();cells.clear();return error=CAPACITY;}
        stats[BUILD_NS]=elapsed(t);record(BUILD_OP,0,0,n*12,envbytes,n+stats[PROBES]+6*points.size());return error;
    }
    bool query(const P&center,int radius){auto t=std::chrono::steady_clock::now();stats[QUERIES]++;I id=++qid;I before=stats[DISTANCE];if(radius<0){error=INVALID_INPUT;return true;}int r=radius+int(cfg[7])+maxcover;bool global=points.empty();for(int a=0;a<3;a++)if(I(center[a])+r<global_lo[a]||I(center[a])-r>global_hi[a])global=true;if(global){stats[GLOBAL_REJECT]++;record(QUERY_OP,id,0,0,0,1,0);stats[QUERY_NS]+=elapsed(t);return false;}
        int lo[3],hi[3];for(int a=0;a<3;a++){lo[a]=std::max<I>(0,floor_div(I(center[a])-r+16384,1024));hi[a]=std::min<I>(31,floor_div(I(center[a])+r+16384,1024));}
        bool hit=false;for(int z=lo[2];z<=hi[2]&&!hit;z++)for(int y=lo[1];y<=hi[1]&&!hit;y++)for(int x=lo[0];x<=hi[0]&&!hit;x++){auto k=morton(x,y,z);stats[CELLS]++;mem(id,0x200000+k*2,2,0);int ci=lookup[k];if(ci<0)continue;const Cell&c=cells[ci];mem(id,0x220000+ci*20,20,0);uint64_t ds=0;for(int a=0;a<3;a++){I d=center[a]<c.lo[a]?I(c.lo[a])-center[a]:center[a]>c.hi[a]?I(center[a])-c.hi[a]:0;ds+=d*d;}if(ds>uint64_t(I(r)*r)){stats[CELL_REJECT]++;continue;}for(uint32_t p=c.start;p<c.start+c.count;p++){const Rep&v=points[p];stats[DISTANCE]++;mem(id,0x300000+p*8,8,0,1);uint64_t d=0;for(int a=0;a<3;a++){I e=I(center[a])-v.p[a];d+=e*e;}I rr=I(radius)+cfg[7]+v.cover;if(d<=uint64_t(rr*rr)){hit=true;break;}}}
        record(QUERY_OP,id,0,0,0,stats[DISTANCE]-before,hit);stats[QUERY_NS]+=elapsed(t);return hit;
    }
    std::array<P,NS> fk(const Q&q){auto t=std::chrono::steady_clock::now();auto v=integer_fk(q);stats[FKCALL]++;record(FK_OP,stats[FKCALL],0,0,0,NJ*54+NS*12+7*30);stats[FK_NS]+=elapsed(t);return v;}
    bool state(const Q&q,int edge_margin){stats[STATECALL]++;if(stats[STATECALL]>cfg[11]){error=BUDGET;return false;}for(int i=0;i<7;i++)if(q[i]<ANGLE_LO[i]||q[i]>ANGLE_HI[i])return false;qrecord(30,stats[STATECALL],q);auto spheres=fk(q);int margin=int(cfg[6])+edge_margin;for(const auto&pair:SELF_PAIRS){int i=pair[0],j=pair[1];I rr=SPHERES[i].radius+SPHERES[j].radius+2*margin;uint64_t d=0;for(int a=0;a<3;a++){I e=I(spheres[i][a])-spheres[j][a];d+=e*e;}if(d<=uint64_t(rr*rr))return false;}bool hit=false;for(int i=0;i<NS;i++){hit=query(spheres[i],SPHERES[i].radius+margin)||hit;if(hit&&!cfg[13])return false;}return !hit;}
    bool edge(const Q&a,const Q&b){stats[EDGECALL]++;record(EDGE_OP,stats[EDGECALL],0,0,0,1);stats[EDGESAMPLE]+=2;if(!state(a,0)||!state(b,0))return false;
        struct Interval{Q a,b;int depth;};std::vector<Interval> stack{{a,b,0}};
        while(!stack.empty()){auto in=stack.back();stack.pop_back();I movement=0;Q mid;for(int j=0;j<7;j++){movement+=std::abs(I(in.b[j])-in.a[j]);mid[j]=int(round_div(I(in.a[j])+in.b[j],2));}if(movement==0)continue;
            I bound=(movement*CS+AS-1)/AS+1; // 2m reach times half interval plus one coordinate unit for rounded midpoint.
            if(bound<=cfg[5]){stats[EDGESAMPLE]++;if(state(mid,int(bound)))continue;if(error)return false;stats[EDGESAMPLE]++;if(!state(mid,0))return false;}
            if(in.depth>=24||mid==in.a||mid==in.b)return false;
            stack.push_back({mid,in.b,in.depth+1});stack.push_back({in.a,mid,in.depth+1});
        }return true;
    }
    int nearest(const std::vector<Node>&tree,const Q&q){stats[NNCALL]++;qrecord(34,stats[NNCALL],q);uint64_t best=UINT64_MAX;int result=0;for(size_t i=0;i<tree.size();i++){uint64_t d=0;for(int j=0;j<7;j++){I e=I(tree[i].q[j])-q[j];d+=e*e;}stats[NNDIST]++;if(d<best){best=d;result=int(i);}}record(NN_OP,stats[NNCALL],tree[0].tree,tree.size()*28,0,tree.size()*7,result);return result;}
    int extend(std::vector<Node>&tree,const Q&target,bool &reached){int parent=nearest(tree,target);const Q&a=tree[parent].q;uint64_t d=0;for(int j=0;j<7;j++){I e=I(target[j])-a[j];d+=e*e;}I norm=ceil_sqrt(d);Q q=target;reached=norm<=cfg[4];if(norm==0){reached=true;return parent;}if(!reached)for(int j=0;j<7;j++)q[j]=int(I(a[j])+round_div((I(target[j])-a[j])*cfg[4],norm));if(q==a)return -1;if(tree.size()>=size_t(cfg[3])){error=CAPACITY;return -1;}if(!edge(a,q))return -1;tree.push_back({q,parent,tree[0].tree});stats[NODES]++;I id=(I(tree[0].tree)<<32)|(tree.size()-1);record(INSERT_OP,id,parent,0,36,1);qrecord(32,id,q);return int(tree.size()-1);}
    static std::vector<Q> backtrack(const std::vector<Node>&tree,int i){std::vector<Q> p;int n=0;while(true){p.push_back(tree[i].q);if(tree[i].parent<0)break;i=tree[i].parent;if(++n>int(tree.size()))throw std::runtime_error("Parent cycle");}std::reverse(p.begin(),p.end());return p;}
    int plan(const Q&start,const Q&goal){auto time=std::chrono::steady_clock::now();path.clear();error=SUCCESS;stats[STATECALL]=stats[ITERATIONS]=stats[NODES]=0;auto finish=[&](int status){stats[PLAN_NS]=elapsed(time);record(PLAN_OP,0,0,0,0,stats[ITERATIONS],status);return error=status;};if(!state(start,0))return finish(error?error:INVALID_START);if(!state(goal,0))return finish(error?error:INVALID_GOAL);if(start==goal){path={start};return finish(SUCCESS);}if(edge(start,goal)){path={start,goal};return finish(SUCCESS);}if(error)return finish(error);
        std::vector<Node> a{{start,-1,0}},b{{goal,-1,1}};qrecord(38,0,start);qrecord(38,1,goal);bool a_is_start=true;stats[NODES]=2;uint32_t seed=uint32_t(cfg[9])|1U;auto random=[&](){seed^=seed<<13;seed^=seed>>17;seed^=seed<<5;return seed;};
        for(I it=0;it<cfg[12];it++){stats[ITERATIONS]++;Q sample;for(int j=0;j<7;j++)sample[j]=ANGLE_LO[j]+int((uint64_t(random())*uint64_t(I(ANGLE_HI[j])-ANGLE_LO[j]))>>32);if(it%10==0)sample=b[0].q;record(SAMPLE_OP,it,0,0,0,7);bool reached=false;int ai=extend(a,sample,reached);if(error)return finish(error);if(ai>=0){int bi=-1;bool connected=false;for(int k=0;k<1000;k++){bi=extend(b,a[ai].q,connected);if(error)return finish(error);if(bi<0||connected)break;}if(connected&&bi>=0){auto pa=backtrack(a,ai),pb=backtrack(b,bi);if(!a_is_start)std::swap(pa,pb);std::reverse(pb.begin(),pb.end());path=pa;path.insert(path.end(),pb.begin()+1,pb.end());record(CONNECT_OP,it,0,0,0,path.size());return finish(SUCCESS);}}a.swap(b);a_is_start=!a_is_start;}return finish(BUDGET);
    }
};
API void* soc_create(const I*cfg){try{return new Model(cfg);}catch(...){return nullptr;}}
API void soc_destroy(void*p){delete static_cast<Model*>(p);}
API void soc_set_version(void*p,I version){static_cast<Model*>(p)->version=version;}
API void soc_trace_config(void*p,I capacity,int clear){auto&m=*static_cast<Model*>(p);m.cfg[10]=capacity;if(clear){m.records.clear();m.stats[TRACE_DROPPED]=0;}}
API void soc_set_callback(void*p,EventCallback callback,void*context){auto&m=*static_cast<Model*>(p);m.callback=callback;m.callback_context=context;}
API int soc_build(void*p,const float*raw,int n){try{return static_cast<Model*>(p)->build(raw,n);}catch(...){return static_cast<Model*>(p)->error=INTERNAL_ERROR;}}
API int soc_query(void*p,const int*center,int r){try{return static_cast<Model*>(p)->query({center[0],center[1],center[2]},r);}catch(...){return -1;}}
API I soc_query_many(void*p,const int*centers,const int*radii,int n,uint8_t*out){auto t=std::chrono::steady_clock::now();auto&m=*static_cast<Model*>(p);for(int i=0;i<n;i++)out[i]=m.query({centers[3*i],centers[3*i+1],centers[3*i+2]},radii[i]);return elapsed(t);}
API int soc_plan(void*p,const int*start,const int*goal){try{Q a,b;std::copy(start,start+7,a.begin());std::copy(goal,goal+7,b.begin());return static_cast<Model*>(p)->plan(a,b);}catch(...){return INTERNAL_ERROR;}}
API void soc_stats(void*p,I*out){std::copy(static_cast<Model*>(p)->stats,static_cast<Model*>(p)->stats+NUM_STATS,out);}
API int soc_snapshot(void*p,I*out){auto&m=*static_cast<Model*>(p);if(!out)return int(m.points.size());for(size_t i=0;i<m.points.size();i++){auto&r=m.points[i];I row[10]={r.key,r.p[0],r.p[1],r.p[2],r.d,r.source,r.cover,r.key>>6,I(i),m.version};std::copy(row,row+10,out+i*10);}return int(m.points.size());}
API void soc_offsets(void*p,uint32_t*out){auto&m=*static_cast<Model*>(p);std::copy(m.offsets.begin(),m.offsets.end(),out);}
API int soc_cells(void*p,I*out){auto&m=*static_cast<Model*>(p);if(out)for(size_t i=0;i<m.cells.size();i++){auto&c=m.cells[i];I row[9]={c.key,c.start,c.count,c.lo[0],c.lo[1],c.lo[2],c.hi[0],c.hi[1],c.hi[2]};std::copy(row,row+9,out+i*9);}return int(m.cells.size());}
API int soc_path(void*p,int*out){auto&m=*static_cast<Model*>(p);if(out)for(size_t i=0;i<m.path.size();i++)std::copy(m.path[i].begin(),m.path[i].end(),out+7*i);return int(m.path.size());}
API int soc_trace(void*p,I*out){auto&m=*static_cast<Model*>(p);if(out)for(size_t i=0;i<m.records.size();i++)std::memcpy(out+8*i,&m.records[i],8*sizeof(I));return int(m.records.size());}
API void soc_fk(const int*q,int*out){Q a;std::copy(q,q+7,a.begin());auto r=integer_fk(a);for(int i=0;i<NS;i++)std::copy(r[i].begin(),r[i].end(),out+3*i);}
API void soc_upstream_fk(const double*q,double*out){upstream_fk_scalar(q,out);}
API uint32_t soc_morton(int x,int y,int z){return morton(x,y,z);}
API int soc_quantize(float x,int*status){*status=0;return float_floor_q(x,13,*status);}
API void soc_sincos(int q,I*out){cordic(q,out[0],out[1]);}
API int soc_nearest(const int*nodes,int n,const int*q){Q target;std::copy(q,q+7,target.begin());uint64_t best=UINT64_MAX;int index=-1;for(int i=0;i<n;i++){uint64_t d=0;for(int j=0;j<7;j++){I e=I(nodes[i*7+j])-target[j];d+=e*e;}if(d<best){best=d;index=i;}}return index;}
API int soc_depth(const uint16_t*depth,int width,int height,const I*camera,const I*transform,int*out){if(width<=0||height<=0||camera[0]<=0||camera[1]<=0||camera[4]<=0)return -1;int n=0;for(int v=0;v<height;v++)for(int u=0;u<width;u++){auto d=depth[v*width+u];if(!d)continue;I z=round_div(I(d)*CS,camera[4]);I p[3]={round_div((I(u)*65536-camera[2])*z,camera[0]),round_div((I(v)*65536-camera[3])*z,camera[1]),z};for(int a=0;a<3;a++){I sum=0;for(int b=0;b<3;b++)sum+=transform[a*4+b]*p[b];I result=round_div(sum,RS)+transform[a*4+3];if(result<-32768||result>32767)return -2;out[n*3+a]=int(result);}n++;}return n;}
