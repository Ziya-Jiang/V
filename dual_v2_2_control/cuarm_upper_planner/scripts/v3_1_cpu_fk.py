"""Read-only CPU FK from the SAME tensors used by CuRobo. No GPU at runtime."""
import numpy as np
from scipy.spatial.transform import Rotation


class CpuKinematics:
    def __init__(self, config):
        def array(name):
            return getattr(config, name).detach().cpu().numpy().copy()
        self.fixed = array('fixed_transforms')
        self.parent = array('link_map')
        self.joint = array('joint_map')
        self.kind = array('joint_map_type')
        self.offset = array('joint_offset_map').reshape(-1, 2)
        self.store = dict(zip(config.link_names, array('store_link_map')))
        self.dof = config.n_dof
        if any(p >= i for i, p in enumerate(self.parent) if i):
            raise ValueError('CPU FK requires topologically ordered links')

    def poses(self, q, names):
        result = self.transforms(q, [int(self.store[n]) for n in names])
        return result[:, :, :3, 3], Rotation.from_matrix(
            result[:, :, :3, :3].reshape(-1, 3, 3)).as_quat()[:, [3, 0, 1, 2]].reshape(len(result), len(names), 4)

    def transforms(self, q, indices):
        q = np.atleast_2d(np.asarray(q, float))
        if q.shape[1] > self.dof or not np.isfinite(q).all():
            raise ValueError('Invalid CPU FK joint vector')
        # Only compute the ancestors needed by these links (no fingers).
        needed = set()
        for index in indices:
            i = int(index)
            while i not in needed:
                needed.add(i)
                if i == 0: break
                i = int(self.parent[i])
        if any(self.kind[i] != -1 and not 0 <= self.joint[i] < q.shape[1] for i in needed):
            raise ValueError('CPU FK missing a joint required by the requested link')
        transforms = {0: np.broadcast_to(self.fixed[0], (len(q), 4, 4))}
        for i in sorted(needed - {0}):
            motion = np.broadcast_to(np.eye(4), (len(q), 4, 4)).copy()
            kind = int(self.kind[i])
            if kind != -1:
                value = q[:, self.joint[i]] * self.offset[i, 0] + self.offset[i, 1]
                if kind >= 6:
                    raise ValueError('Expected CuRobo normalized joint axis and offset')
                if kind < 3:
                    motion[:, kind, 3] = value
                elif kind <= 5:
                    axis = np.zeros((len(q), 3))
                    axis[:, kind - 3] = value
                    motion[:, :3, :3] = Rotation.from_rotvec(axis).as_matrix()
                else:
                    raise ValueError('Unsupported CPU FK joint type')
            transforms[i] = transforms[int(self.parent[i])] @ self.fixed[i] @ motion
        return np.stack([transforms[int(i)] for i in indices], axis=1)
