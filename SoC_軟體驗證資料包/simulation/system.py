"""Double-buffer and asynchronous request semantics around actual model results."""
import heapq

from runner.model import Model


class Busy(RuntimeError):pass


class EnvironmentManager:
    def __init__(self,config=None):
        self.config=config;self.slots=[None,None];self.active=None;self.staging=None
        self.version=0;self.clock=0;self.next_id=1;self.pending={};self.events=[];self.responses={}
    def close(self):
        if self.pending:raise Busy('Outstanding requests still pin environment buffers')
        for slot in self.slots:
            if slot:slot['model'].close()
        self.slots=[None,None]
    def prepare(self,points):
        if self.staging is not None:raise Busy('A completed build is waiting for publication')
        candidates=[i for i,s in enumerate(self.slots) if i!=self.active and (s is None or s['pins']==0)]
        if not candidates:raise Busy('Both buffers are active or pinned')
        i=candidates[0];model=Model(self.config);candidate_version=self.version+1
        model.lib.soc_set_version(model.handle,candidate_version);status=model.build(points)
        if status!='SUCCESS':model.close();return status
        if self.slots[i]:self.slots[i]['model'].close()
        self.version=candidate_version
        self.slots[i]=dict(model=model,version=self.version,pins=0);self.staging=i
        return status
    def publish(self):
        if self.staging is None:raise Busy('No successfully completed build to publish')
        self.active=self.staging;self.staging=None
        return self.slots[self.active]['version']
    def submit(self,center,radius,delay=0):
        if self.active is None:raise Busy('No published environment')
        if delay<0:raise ValueError('Negative memory latency')
        slot=self.slots[self.active];request_id=self.next_id;self.next_id+=1
        # The synchronous functional result is hidden until its scheduled completion.
        hit=slot['model'].query(center,radius);slot['pins']+=1
        self.pending[request_id]=dict(slot=self.active,version=slot['version'],hit=hit,cancelled=False)
        heapq.heappush(self.events,(self.clock+delay,request_id))
        return request_id
    def cancel(self,request_id):
        if request_id not in self.pending:return False
        entry=self.pending[request_id]
        if entry['cancelled']:return False
        entry['cancelled']=True
        self.responses[request_id]=dict(status='CANCELLED',request_id=request_id,environment_version=entry['version'])
        # Do not release the physical buffer until the outstanding operation drains.
        return True
    def advance(self,until):
        if until<self.clock:raise ValueError('Cannot run time backwards')
        while self.events and self.events[0][0]<=until:
            self.clock,request_id=heapq.heappop(self.events);entry=self.pending.pop(request_id)
            slot=self.slots[entry['slot']]
            if slot['version']!=entry['version']:raise AssertionError('Pinned buffer was overwritten')
            slot['pins']-=1
            if not entry['cancelled']:
                if request_id in self.responses:raise AssertionError('Duplicate completion')
                self.responses[request_id]=dict(status='COMPLETE',request_id=request_id,environment_version=entry['version'],hit=entry['hit'])
        self.clock=until
    def drain(self):
        if self.events:self.advance(max(t for t,_ in self.events))
        return self.responses
    def plan(self,start,goal):
        if self.active is None:raise Busy('No published environment')
        slot=self.slots[self.active];slot['pins']+=1;request_id=self.next_id;self.next_id+=1
        try:
            result=slot['model'].plan(start,goal)
            return {**result,'request_id':request_id,'environment_version':slot['version']}
        finally:slot['pins']-=1
