"""Fail-closed window/constraint consumption gate, without any motion transport."""
import hashlib
import math
import numpy as np
from osmag_constraint_mask import ConsumptionGate,ConstraintError
from gcu_plan_validation import validate_path


class WindowAdmission(ConsumptionGate):
    def __init__(self,window,snapshot,published_at,free_probes):
        if (snapshot.mask.shape!=window.grid.shape or snapshot.origin!=window.origin
                or snapshot.resolution!=window.resolution or snapshot.mask_revision!=window.revision
                or snapshot.physical_sha256!=hashlib.sha256(np.ascontiguousarray(window.grid,dtype=np.int16).tobytes()).hexdigest()):
            raise ConstraintError('window_constraint_snapshot_mismatch')
        super().__init__(snapshot,published_at,free_probes)
        self.window=window
        self.last_source_stamp=published_at

    def note_source_publication(self,stamp,window_sha256):
        if (not math.isfinite(stamp) or stamp<self.last_source_stamp
                or window_sha256!=self.window.sha256):
            raise ConstraintError('obsolete_or_invalid_source_publication')
        self.last_source_stamp=stamp

    def ready(self,now):
        return 0<=now-self.last_source_stamp<=.5 and super().ready(now)

    def can_replace(self,now):
        """Reserve .35s of the original .5s lease for overlapping updates."""
        return self.ready(now) and now-self.last_source_stamp<=.15 and all(
            0<=now-v['stamp']<=.15 for v in self.observations.values())

    def observe(self,source,grid,resolution,origin,stamp,now,**kwargs):
        self.observations.pop(source,None)
        if source=='global' and (np.asarray(grid).shape!=self.window.grid.shape
                or any(abs(a-b)>1e-7 for a,b in zip(origin,self.window.origin))):
            raise ConstraintError('native_global_window_geometry_mismatch')
        return super().observe(source,grid,resolution,origin,stamp,now,**kwargs)

    def admit(self,goal,now):
        if goal.window_sha256!=self.window.sha256 or goal.window_revision!=self.window.revision:
            raise ConstraintError('obsolete_proxy_window')
        if not self.ready(now):raise ConstraintError('window_constraints_not_freshly_consumed')
        check=validate_path(self.snapshot.combined,self.snapshot.resolution,self.snapshot.origin,
                            goal.prefix,self.snapshot.radius+self.snapshot.padding)
        if not check['valid']:raise ConstraintError('proxy_prefix_blocked_by_constraint')
        return dict(window_sha256=self.window.sha256,window_revision=self.window.revision,
                    mask_sha256=self.snapshot.mask_sha256,observed_at=now,
                    source_last_publication=self.last_source_stamp,
                    scope='Admission only; not an action dispatch')
