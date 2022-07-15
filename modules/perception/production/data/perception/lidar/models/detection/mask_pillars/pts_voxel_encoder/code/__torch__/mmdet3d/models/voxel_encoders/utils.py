class VFELayer(Module):
  __parameters__ = []
  training : bool
  norm : __torch__.mmdet3d.ops.norm.NaiveSyncBatchNorm1d
  linear : __torch__.torch.nn.modules.linear.Linear
  def forward(self: __torch__.mmdet3d.models.voxel_encoders.utils.VFELayer,
    inputs: Tensor) -> Tensor:
    _0 = self.norm
    _1 = torch.permute((self.linear).forward(inputs, ), [0, 2, 1])
    input = torch.contiguous(_1, memory_format=0)
    _2 = torch.permute((_0).forward(input, ), [0, 2, 1])
    input0 = torch.contiguous(_2, memory_format=0)
    pointwise = torch.relu(input0)
    aggregated, _3 = torch.max(pointwise, 1, True)
    return torch.squeeze(aggregated, 1)
