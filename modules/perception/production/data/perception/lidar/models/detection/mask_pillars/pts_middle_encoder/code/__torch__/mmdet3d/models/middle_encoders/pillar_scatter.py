class PointPillarsScatter(Module):
  __parameters__ = []
  training : bool
  def forward(self: __torch__.mmdet3d.models.middle_encoders.pillar_scatter.PointPillarsScatter,
    voxel_features: Tensor,
    coors: Tensor,
    batch_size: Tensor) -> Tensor:
    _0 = int(batch_size)
    canvas = torch.zeros([64, 219024], dtype=6, layout=0, device=torch.device("cuda:0"), pin_memory=False)
    _1 = torch.slice(coors, 0, 0, 9223372036854775807, 1)
    batch_mask = torch.eq(torch.select(_1, 1, 0), 0)
    _2 = torch.slice(coors, 1, 0, 9223372036854775807, 1)
    batch_mask0 = torch.to(batch_mask, dtype=11, layout=0, device=torch.device("cuda:0"), pin_memory=False, non_blocking=False, copy=False, memory_format=None)
    _3 = annotate(List[Optional[Tensor]], [batch_mask0])
    this_coors = torch.index(_2, _3)
    _4 = torch.slice(this_coors, 0, 0, 9223372036854775807, 1)
    _5 = torch.slice(this_coors, 0, 0, 9223372036854775807, 1)
    _6 = torch.mul(torch.select(_4, 1, 2), CONSTANTS.c0)
    indices = torch.add(_6, torch.select(_5, 1, 3), alpha=1)
    indices0 = torch.to(indices, torch.device("cuda:0"), 4, False, False, None)
    _7 = torch.slice(voxel_features, 1, 0, 9223372036854775807, 1)
    batch_mask1 = torch.to(batch_mask0, dtype=11, layout=0, device=torch.device("cuda:0"), pin_memory=False, non_blocking=False, copy=False, memory_format=None)
    _8 = annotate(List[Optional[Tensor]], [batch_mask1])
    voxels = torch.index(_7, _8)
    voxels0 = torch.t(voxels)
    _9 = torch.slice(canvas, 0, 0, 9223372036854775807, 1)
    indices1 = torch.to(indices0, dtype=4, layout=0, device=torch.device("cuda:0"), pin_memory=False, non_blocking=False, copy=False, memory_format=None)
    _10 = annotate(List[Optional[Tensor]], [None, indices1])
    _11 = torch.index_put_(_9, _10, voxels0, False)
    batch_canvas = torch.stack([canvas], 0)
    _12 = torch.view(batch_canvas, [_0, 64, 468, 468])
    return _12
