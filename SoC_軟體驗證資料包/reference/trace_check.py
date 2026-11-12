"""Replay native RRT decisions against arbitrary-precision nearest/steering rules."""
import math
import numpy as np


def validate_rrt_trace(trace,stats,config):
    if stats.get('trace_dropped'):return dict(valid=False,status='INCOMPLETE_TRACE')
    vectors={};trees={0:[],1:[]};parents={};last_nn={};checks=0;inserts=0
    step=math.floor(config['rrt_range_rad']*(1<<24))
    for row in trace:
        kind,ident,version,address,rb,wb,units,result=map(int,row)
        if kind in (30,32,34,38):vectors[(kind,ident)]=[address,rb,wb,units,result]
        elif kind in (31,33,35,39):
            base=kind-1;key=(base,ident)
            if key not in vectors:return dict(valid=False,status='MISSING_VECTOR_PREFIX')
            vectors[key]+=[address,rb]
            if base==38:trees[ident]=[vectors[key]]
            if base==32:
                tree_id=ident>>32;index=ident&0xffffffff
                if index!=len(trees[tree_id]) or ident not in parents:return dict(valid=False,status='INSERT_ORDER')
                parent=parents[ident]
                if parent<0 or parent>=index:return dict(valid=False,status='PARENT_RANGE')
                target,nearest=last_nn[tree_id]
                if parent!=nearest:return dict(valid=False,status='PARENT_NEAREST_MISMATCH')
                a=trees[tree_id][parent];squared=sum((int(x)-int(y))**2 for x,y in zip(target,a));norm=math.isqrt(squared);norm+=norm*norm!=squared
                expected=target if norm<=step else [int(x)+((int(y)-int(x))*step+norm//2)//norm for x,y in zip(a,target)]
                if expected!=vectors[key]:return dict(valid=False,status='STEERING_MISMATCH',node=ident)
                trees[tree_id].append(vectors[key]);inserts+=1
        elif kind==4:
            target=vectors.get((34,ident))
            if target is None or len(target)!=7:return dict(valid=False,status='MISSING_NN_TARGET')
            tree=trees[address]
            if not tree:return dict(valid=False,status='MISSING_ROOT')
            distances=[sum((int(x)-int(y))**2 for x,y in zip(q,target)) for q in tree]
            expected=min(range(len(tree)),key=lambda i:distances[i])
            if expected!=result:return dict(valid=False,status='NEAREST_MISMATCH',query=ident)
            last_nn[address]=(target,result);checks+=1
        elif kind==7:parents[ident]=address
    if 'nn_calls' in stats and checks!=stats['nn_calls']:return dict(valid=False,status='MISSING_NN_EVENTS',actual=checks,expected=stats['nn_calls'])
    roots=sum(bool(t) for t in trees.values())
    if 'rrt_nodes' in stats and inserts+roots!=stats['rrt_nodes']:return dict(valid=False,status='MISSING_INSERT_EVENTS',actual=inserts+roots,expected=stats['rrt_nodes'])
    return dict(valid=True,status='REPLAYED_NEAREST_STEERING_PARENT_RULES',nearest_checks=checks,inserts=inserts,
                scope='Structural replay and exact nearest/steering rules; collision and FK are checked separately')
