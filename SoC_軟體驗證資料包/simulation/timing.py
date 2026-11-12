"""Causally ordered timing annotations. Parameters are hypotheses, not silicon measurements."""
from collections import OrderedDict
import json
import math


class Memory:
    def __init__(self,p):
        self.p=p;self.banks=[0]*p['sram_banks'];self.bus=0;self.cache=OrderedDict()
        self.counts=dict(cache_hits=0,cache_misses=0,external_reads=0,external_writes=0,external_read_bytes=0,external_write_bytes=0,sram_reads=0,sram_writes=0)
    def external(self,ready,write=False):
        p=self.p;start=max(ready,self.bus);end=start+math.ceil(p['block_bytes']/p['external_bytes_per_cycle']);self.bus=end
        self.counts['external_writes' if write else 'external_reads']+=1
        self.counts['external_write_bytes' if write else 'external_read_bytes']+=p['block_bytes']
        return end+p['external_latency_cycles']
    def access(self,address,read_bytes,write_bytes,ready):
        p=self.p;size=max(read_bytes,write_bytes)
        if not size:return ready
        end=ready
        for line in range(address//p['block_bytes'],(address+size-1)//p['block_bytes']+1):
            bank=line%p['sram_banks'];start=max(end,self.banks[bank]);self.banks[bank]=start+1
            self.counts['sram_reads']+=read_bytes>0;self.counts['sram_writes']+=write_bytes>0
            if p.get('environment_storage')=='resident_sram':
                self.counts['cache_hits']+=1;end=start+p['sram_latency_cycles'];continue
            if line in self.cache:
                self.counts['cache_hits']+=1;dirty=self.cache.pop(line)
            else:
                self.counts['cache_misses']+=1;dirty=False;start=self.external(start)
                capacity=max(1,p['query_cache_bytes']//p['block_bytes'])
                if len(self.cache)>=capacity:
                    _,old_dirty=self.cache.popitem(last=False)
                    if old_dirty:start=max(start,self.external(start,write=True))
            self.cache[line]=dirty or write_bytes>0
            end=start+p['sram_latency_cycles']
        return end


class OnlineTiming:
    """Receives actual native events before the functional execution proceeds."""
    def __init__(self,stats,profile):
        if profile['clock_hz']<=0 or profile['external_bytes_per_cycle']<=0 or profile['sram_banks']<=0:raise ValueError('Invalid timing profile')
        self.stats=stats;self.profile=profile;self.memory=Memory(profile)
        self.cycles=math.ceil(stats['input_points']*12/profile['external_bytes_per_cycle'])+profile['external_latency_cycles']
        build_cycles=stats['input_points']+(stats['hash_probes']+stats['input_points']-stats.get('culled_points',0))*profile['sram_latency_cycles']+math.ceil((stats['radix_reads']+stats['radix_writes'])/profile['sort_records_per_cycle'])+32769
        self.cycles+=build_cycles;self.compute=build_cycles;self.mem_cycles=0;self.event_count=0
    def consume(self,row):
        profile=self.profile;memory=self.memory;self.event_count+=1
        kind,request,version,address,rb,wb,units,result=map(int,row)
        if kind==10:
            before=self.cycles;self.cycles=memory.access(address,rb,wb,self.cycles);self.mem_cycles+=self.cycles-before
        elif kind==2:
            delta=math.ceil(units/profile['fk_integer_ops_per_cycle']);self.cycles+=delta;self.compute+=delta
        elif kind==3:
            delta=1+math.ceil(units/profile['distance_lanes']);self.cycles+=delta;self.compute+=delta
        elif kind==4:
            self.cycles=memory.access(0x500000+address*0x100000,rb,0,self.cycles)
            delta=math.ceil(units/profile['nn_integer_ops_per_cycle']);self.cycles+=delta;self.compute+=delta
        elif kind in (5,6,7,8):self.cycles+=1;self.compute+=1
    def result(self):
        counts=self.memory.counts.copy();stats=self.stats;profile=self.profile
        counts['external_read_bytes']+=stats['input_points']*12
        counts['external_reads']+=math.ceil(stats['input_points']*12/profile['block_bytes'])
        return dict(status='ASSUMPTION_BASED_SERIAL_SCHEDULE',estimated_cycles=self.cycles,
                    estimated_seconds=self.cycles/profile['clock_hz'],compute_annotation_cycles=self.compute,memory_wait_annotation_cycles=self.mem_cycles,
                    memory=counts,profile=profile,event_count=self.event_count,
                    scope='Actual causally generated operations; single serial data-dependent schedule; no claimed cycle accuracy, multi-engine speculation, cycle-deadline behavior or achieved frequency')


def simulate_trace(trace,stats,profile):
    if stats.get('trace_dropped',0):return dict(status='INCOMPLETE_TRACE',estimated_cycles=None,reason='Increase trace capacity; truncated traces are not silently estimated')
    if len(trace)==0:return dict(status='NO_TRACE',estimated_cycles=None)
    scheduler=OnlineTiming(stats,profile)
    for row in trace:scheduler.consume(row)
    result=scheduler.result();result['execution_mode']='replay_of_actual_functional_trace';return result


def estimate_energy(stats,timing,coefficients):
    operations=dict(integer_ops=stats['hash_probes']*8+stats['distance_tests']*8+stats['nn_distance_tests']*20+stats['fk_calls']*(12*54+59*12+7*30),
                    logical_sram_read_bytes=stats['logical_read_bytes'],logical_sram_write_bytes=stats['logical_write_bytes'])
    if timing.get('memory'):
        operations.update(external_read_bytes=timing['memory']['external_read_bytes'],external_write_bytes=timing['memory']['external_write_bytes'])
    else:operations.update(external_read_bytes=stats['input_points']*12,external_write_bytes=0)
    required=['integer_op_pj','sram_read_byte_pj','sram_write_byte_pj','external_read_byte_pj','external_write_byte_pj','static_power_w']
    if any(coefficients.get(k) is None for k in required) or not timing.get('estimated_seconds'):
        return dict(status='UNCHARACTERIZED',energy_j=None,operations=operations,
                    reason='Target memory and technology coefficients are not available. Operation counts are reported without invented watts.')
    values=[operations[k] for k in ['integer_ops','logical_sram_read_bytes','logical_sram_write_bytes','external_read_bytes','external_write_bytes']]
    dynamic=sum(v*coefficients[k] for v,k in zip(values,required[:5]))*1e-12
    static=coefficients['static_power_w']*timing['estimated_seconds']
    return dict(status='COEFFICIENT_BASED_ESTIMATE',energy_j=dynamic+static,dynamic_energy_j=dynamic,static_energy_j=static,operations=operations,
                coefficient_source=coefficients.get('source','unspecified'))
